/*
 * Copyright (c) 2023, Jisheng Zhang <jszhang@kernel.org>. All rights reserved.
 *
 * SPDX-License-Identifier: BSD-3-Clause
 */

#include <string.h>
#include <stdbool.h>
#include <inttypes.h>
#include <unistd.h>

#include "sdkconfig.h"
#include "rv32_diagnostics.h"
#include "esp_system.h"
#include "esp_rom_sys.h"
#include "soc/lp_system_reg.h"
#include "soc/soc.h"
#include "esp_flash.h"
#include "esp_cache.h"
#include "esp_heap_caps.h"
#include "esp_memory_utils.h"
#include "esp_partition.h"
#include "esp_random.h"
#include "esp_timer.h"
#include "esp32p4/rom/cache.h"
#include "bootloader_random.h"
#include "driver/uart.h"
#include "driver/uart_vfs.h"
#include "freertos/FreeRTOS.h"
#include "freertos/semphr.h"
#include "freertos/stream_buffer.h"
#include "freertos/task.h"
#include "hal/uart_ll.h"
#include "hal/brownout_hal.h"
#include "hal/brownout_ll.h"
#include "esp_intr_alloc.h"
#include "soc/lp_analog_peri_struct.h"
#include "soc/power_supply_periph.h"
#include "soc/hp_sys_clkrst_struct.h"
#include "soc/uart_struct.h"
#include "esp_rom_serial_output.h"
#include "esp_private/system_internal.h"
#include "virtio_net_bridge.h"
#if CONFIG_RV32_EMULATOR_PERF_STATS
#include "soc/cache_reg.h"
#include "soc/soc.h"
#endif
#include "display_bridge.h"
#include "dwc2_passthrough.h"
#include "port.h"
#include "usb_hid_input.h"
#include "psram.h"

#define FLASH_COPY_CHUNK_SIZE (64u * 1024u)
#define FDT_BEGIN_NODE 1u
#define FDT_END_NODE   2u
#define FDT_PROP       3u
#define FDT_NOP        4u
#define FDT_END        9u
#define LINUX_RNG_SEED_SIZE 64u
#define HOST_UART_TX_QUEUE_SIZE (32u * 1024u)
#define HOST_UART_BITS_PER_BYTE 10u
#define HOST_UART_TX_CHUNK_SIZE 1024u
#define HOST_UART_TX_STACK_SIZE 4096u
/* UART output is best-effort console traffic. Keep it below the display and
 * network services, and feed the hardware no more than one line-rate quota per
 * RTOS tick. This prevents a continuous ttyS0 stream from occupying CPU1. */
#define HOST_UART_TX_PRIORITY   (tskIDLE_PRIORITY + 2u)
#define HOST_UART_TX_TICK_BUDGET                                           \
  (CONFIG_ESP_CONSOLE_UART_BAUDRATE /                                    \
   (HOST_UART_BITS_PER_BYTE * CONFIG_FREERTOS_HZ))
#define HOST_UART_DRIVER_RX_SIZE 2048u
#define HOST_UART_DRIVER_TX_SIZE (8u * 1024u)

_Static_assert(HOST_UART_TX_TICK_BUDGET > 0u,
               "UART baud rate is too low for the FreeRTOS tick rate");
_Static_assert(HOST_UART_TX_TICK_BUDGET <= HOST_UART_TX_CHUNK_SIZE,
               "UART tick quota exceeds the TX worker buffer");

static StaticStreamBuffer_t host_uart_tx_stream_control;
#if CONFIG_RV32_HOST_BULK_BUFFERS_PSRAM
static uint8_t *host_uart_tx_stream_storage;
#else
static uint8_t host_uart_tx_stream_storage[HOST_UART_TX_QUEUE_SIZE];
#endif
static StreamBufferHandle_t host_uart_tx_stream;
static uint32_t host_uart_tx_dropped;
static bool host_uart_driver_ready;
static int host_uart_init_result = -1;

static uint32_t read_be32(const uint8_t *value);

uint64_t GetTimeMicroseconds() { return esp_timer_get_time(); }

void HostEnableGuestRamPrefetch(const void *base, size_t size)
{
#if CONFIG_RV32_GUEST_RAM_PREFETCH
  const uintptr_t line_size = CONFIG_CACHE_L2_CACHE_LINE_SIZE;
  uintptr_t start;
  uintptr_t end;

  if (base == NULL || size == 0)
    return;
  start = (uintptr_t)base & ~(line_size - 1u);
  end = ((uintptr_t)base + size + line_size - 1u) & ~(line_size - 1u);

  /*
   * Guest instruction fetches are ordinary P4 data reads from PSRAM. Ask the
   * hardware for the following L2 line while the interpreter consumes the
   * current one. This is deliberately scoped to the caller-provided active
   * Linux RAM window; it does not include the reserved PSRAM prefix or LCD
   * scanout allocation.
   */
  const struct l1_dcache_l2_autoload_config autoload = {
    .gid = 0,
    .order = CACHE_AUTOLOAD_POSITIVE,
    .trigger = CACHE_AUTOLOAD_BOTH_TRIGGER,
    .ena0 = 1,
    .addr0 = start,
    .size0 = end - start,
  };

  Cache_Disable_L2_Cache_Autoload();
  Cache_Config_L2_Cache_Autoload(&autoload);
  Cache_Enable_L2_Cache_Autoload();
  RV32_DIAG("L2 next-line prefetch: host [%p, %p)\n", (void *)start,
         (void *)end);
#else
  Cache_Disable_L2_Cache_Autoload();
  RV32_DIAG("L2 next-line prefetch: disabled\n");
  (void)base;
  (void)size;
#endif
}

void HostCacheStatsReset(void)
{
#if CONFIG_RV32_EMULATOR_PERF_STATS
	REG_SET_BIT(CACHE_L1_CACHE_ACS_CNT_CTRL_REG, CACHE_L1_DBUS0_CNT_CLR);
	REG_SET_BIT(CACHE_L1_CACHE_ACS_CNT_CTRL_REG, CACHE_L1_DBUS0_CNT_ENA);
	REG_SET_BIT(CACHE_L2_CACHE_ACS_CNT_CTRL_REG, CACHE_L2_DBUS0_CNT_CLR);
	REG_SET_BIT(CACHE_L2_CACHE_ACS_CNT_CTRL_REG, CACHE_L2_DBUS0_CNT_ENA);
#endif
}

void HostCacheStatsReadAndReset(struct host_cache_stats *stats)
{
#if CONFIG_RV32_EMULATOR_PERF_STATS
	stats->l1_hits = REG_READ(CACHE_L1_DBUS0_ACS_HIT_CNT_REG);
	stats->l1_misses = REG_READ(CACHE_L1_DBUS0_ACS_MISS_CNT_REG);
	stats->l1_conflicts = REG_READ(CACHE_L1_DBUS0_ACS_CONFLICT_CNT_REG);
	stats->l1_next_reads = REG_READ(CACHE_L1_DBUS0_ACS_NXTLVL_RD_CNT_REG);
	stats->l1_next_writes = REG_READ(CACHE_L1_DBUS0_ACS_NXTLVL_WR_CNT_REG);
	stats->l2_hits = REG_READ(CACHE_L2_DBUS0_ACS_HIT_CNT_REG);
	stats->l2_misses = REG_READ(CACHE_L2_DBUS0_ACS_MISS_CNT_REG);
	stats->l2_conflicts = REG_READ(CACHE_L2_DBUS0_ACS_CONFLICT_CNT_REG);
	stats->l2_next_reads = REG_READ(CACHE_L2_DBUS0_ACS_NXTLVL_RD_CNT_REG);
	stats->l2_next_writes = REG_READ(CACHE_L2_DBUS0_ACS_NXTLVL_WR_CNT_REG);
	HostCacheStatsReset();
#else
	memset(stats, 0, sizeof(*stats));
#endif
}

static void write_be32(uint8_t *destination, uint32_t value)
{
  destination[0] = (uint8_t)(value >> 24);
  destination[1] = (uint8_t)(value >> 16);
  destination[2] = (uint8_t)(value >> 8);
  destination[3] = (uint8_t)value;
}

#if CONFIG_RV32_DISPLAY_ACCEL

/* Each display DTB describes one panel size, and U8g2 panels of every size
 * share the 128x64 one. Write the geometry this firmware renders into the
 * framebuffer node, so Linux and the display bridge always agree. */
static bool prepare_linux_framebuffer_property(const char *name,
                                               uint8_t *value,
                                               uint32_t length)
{
  static const struct {
    const char *name;
    uint32_t value;
  } cells[] = {
    { "width", DISPLAY_FB_WIDTH },
    { "height", DISPLAY_FB_HEIGHT },
    { "stride", DISPLAY_FB_STRIDE },
    { "mini-rv32ima,staging-offset",
      DISPLAY_ACCEL_STAGE_GUEST_BASE - DISPLAY_FB_GUEST_BASE },
    { "mini-rv32ima,fence-offset",
      DISPLAY_FB_COMMIT_GUEST_BASE - DISPLAY_FB_GUEST_BASE },
    { "mini-rv32ima,accel-offset",
      DISPLAY_PV_REG_GUEST_BASE - DISPLAY_FB_GUEST_BASE },
  };

  /* reg = <0 base 0 size>: two address and two size cells. */
  if (strcmp(name, "reg") == 0 && length == 4u * sizeof(uint32_t)) {
    write_be32(value + 3u * sizeof(uint32_t), DISPLAY_FB_APERTURE_SIZE);
    return true;
  }
  for (size_t index = 0; index < sizeof(cells) / sizeof(cells[0]); index++) {
    if (length == sizeof(uint32_t) && strcmp(name, cells[index].name) == 0) {
      write_be32(value, cells[index].value);
      return true;
    }
  }
  return false;
}
#endif

const uint8_t *rv32_xip_host;
uint32_t rv32_xip_size;

/* The flash MMU (its own 64 MB window at 0x40000000, separate from the
 * PSRAM MMU) is otherwise unused: the app runs from PSRAM.  Map the whole
 * "xip" partition once; it stays mapped for the uptime.  SPI1 reads by the
 * rofs worker keep the cache enabled (esp_flash's mmap lock); erases and
 * writes elsewhere in flash disable it for their duration. */
void psram_map_xip_partition(void)
{
#if CONFIG_RV32_FLASH_XIP
  const esp_partition_t *partition = esp_partition_find_first(
      ESP_PARTITION_TYPE_DATA, ESP_PARTITION_SUBTYPE_ANY, "xip");
  esp_partition_mmap_handle_t handle;
  const void *mapped;

  if (partition == NULL) {
    printf("WARNING: no 'xip' partition; files are not mapped in place\n");
    return;
  }
  if (partition->address + partition->size > 0x1000000u) {
    printf("WARNING: the 'xip' partition ends above 16 MiB, beyond the "
           "quad-flash cache mapping\n");
    return;
  }
  esp_err_t err = esp_partition_mmap(partition, 0, partition->size,
                                     ESP_PARTITION_MMAP_DATA, &mapped,
                                     &handle);
  if (err != ESP_OK) {
    printf("WARNING: mapping the 'xip' partition failed: %s\n",
           esp_err_to_name(err));
    return;
  }
  /* cramfs superblock magic, little endian. */
  if (*(const volatile uint32_t *)mapped != 0x28cd3d45u) {
    printf("WARNING: the 'xip' partition holds no cramfs image\n");
    esp_partition_munmap(handle);
    return;
  }
  rv32_xip_host = mapped;
  rv32_xip_size = partition->size;
  RV32_DIAG("xip: %" PRIu32 " KB of flash at guest 0x%08x (host %p)\n",
            rv32_xip_size / 1024u, (unsigned int)GUEST_XIP_BASE, mapped);
#endif
}

/* The "mtd-rom" node gets the mapped size, or a compatible that no driver
 * matches (same length, so the DTB keeps its layout).  Linux's physmap rounds
 * its window down to a power of two, so reg names the next power of two;
 * cramfs reads only its image, which lies in the mapped part. */
static bool prepare_linux_xip_property(const char *name, uint8_t *value,
                                       uint32_t length)
{
  if (strcmp(name, "reg") == 0 && length == 4u * sizeof(uint32_t) &&
      read_be32(value + sizeof(uint32_t)) == GUEST_XIP_BASE) {
    uint32_t window = rv32_xip_size;

    if (window & (window - 1u))
      window = UINT32_C(1) << (32 - __builtin_clz(window));
    write_be32(value + 3u * sizeof(uint32_t), window);
    return true;
  }
  if (rv32_xip_size == 0u && strcmp(name, "compatible") == 0 &&
      length == sizeof("mtd-rom") &&
      memcmp(value, "mtd-rom", sizeof("mtd-rom")) == 0) {
    memcpy(value, "rv32,no", sizeof("rv32,no"));
    return true;
  }
  return false;
}

static bool prepare_linux_dtb(uint8_t *dtb, size_t dtb_size)
{
  uint32_t struct_offset;
  uint32_t strings_offset;
  uint32_t struct_size;
  uint32_t strings_size;
  size_t cursor;
  size_t struct_end;
  bool rng_seeded = false;
  uint32_t depth = 0;
#if CONFIG_RV32_USB_FULL_SPEED_ONLY
  bool usb_speed_limited = false;
#endif
#if CONFIG_RV32_USB_PORT_HID
  uint32_t usb_depth = 0;
  bool usb_hidden = false;
#endif
#if CONFIG_RV32_DISPLAY_ACCEL
  uint32_t framebuffer_depth = 0;
  bool framebuffer_described = false;
#endif
  uint32_t xip_depth = 0;

  if (dtb_size < 40 || read_be32(dtb) != 0xd00dfeedu)
    return false;

  struct_offset = read_be32(dtb + 8);
  strings_offset = read_be32(dtb + 12);
  strings_size = read_be32(dtb + 32);
  struct_size = read_be32(dtb + 36);
  if (struct_offset > dtb_size || struct_size > dtb_size - struct_offset ||
      strings_offset > dtb_size || strings_size > dtb_size - strings_offset)
    return false;

  cursor = struct_offset;
  struct_end = struct_offset + struct_size;
  while (cursor + sizeof(uint32_t) <= struct_end) {
    uint32_t token = read_be32(dtb + cursor);

    cursor += sizeof(uint32_t);
    if (token == FDT_BEGIN_NODE) {
      const void *terminator = memchr(dtb + cursor, '\0', struct_end - cursor);

      if (terminator == NULL)
        return false;
      depth++;
#if CONFIG_RV32_USB_PORT_HID
      if (usb_depth == 0u &&
          strcmp((const char *)(dtb + cursor), "usb@50000000") == 0)
        usb_depth = depth;
#endif
#if CONFIG_RV32_DISPLAY_ACCEL
      if (framebuffer_depth == 0u &&
          strncmp((const char *)(dtb + cursor), "framebuffer@",
                  sizeof("framebuffer@") - 1u) == 0)
        framebuffer_depth = depth;
#endif
      if (xip_depth == 0u &&
          strcmp((const char *)(dtb + cursor), "flash@40000000") == 0)
        xip_depth = depth;
      cursor = ((const uint8_t *)terminator - dtb + 1u + 3u) & ~3u;
    } else if (token == FDT_PROP) {
      uint32_t length;
      uint32_t name_offset;
      const char *name;
      const void *name_end;

      if (cursor + 8u > struct_end)
        return false;
      length = read_be32(dtb + cursor);
      name_offset = read_be32(dtb + cursor + 4u);
      cursor += 8u;
      if (length > struct_end - cursor || name_offset >= strings_size)
        return false;

      name = (const char *)(dtb + strings_offset + name_offset);
      name_end = memchr(name, '\0', strings_size - name_offset);
      if (name_end == NULL)
        return false;

      if (strcmp(name, "rng-seed") == 0) {
        if (length != LINUX_RNG_SEED_SIZE)
          return false;

        bootloader_random_enable();
        esp_fill_random(dtb + cursor, length);
        bootloader_random_disable();
        RV32_DIAG("Seeded Linux CRNG with %" PRIu32
               " bytes of fresh P4 hardware entropy\n", length);
        rng_seeded = true;
      }
#if CONFIG_RV32_USB_FULL_SPEED_ONLY
      if (strcmp(name, "maximum-speed") == 0 &&
          length >= sizeof("full-speed")) {
        memset(dtb + cursor, 0, length);
        memcpy(dtb + cursor, "full-speed", sizeof("full-speed"));
        usb_speed_limited = true;
      }
#endif
#if CONFIG_RV32_USB_PORT_HID
      /* CPU1 owns the USB controller: no Linux driver may bind it.  The
       * replacement has the same length, so the DTB keeps its layout. */
      if (usb_depth != 0u && depth == usb_depth &&
          strcmp(name, "compatible") == 0 && length == sizeof("snps,dwc2") &&
          memcmp(dtb + cursor, "snps,dwc2", sizeof("snps,dwc2")) == 0) {
        memcpy(dtb + cursor, "rv32,none", sizeof("rv32,none"));
        usb_hidden = true;
      }
#endif
#if CONFIG_RV32_DISPLAY_ACCEL
      if (framebuffer_depth != 0u && depth == framebuffer_depth &&
          prepare_linux_framebuffer_property(name, dtb + cursor, length))
        framebuffer_described = true;
#endif
      if (xip_depth != 0u && depth == xip_depth)
        prepare_linux_xip_property(name, dtb + cursor, length);
      cursor = (cursor + length + 3u) & ~3u;
    } else if (token == FDT_END_NODE) {
#if CONFIG_RV32_USB_PORT_HID
      if (depth == usb_depth)
        usb_depth = 0;
#endif
#if CONFIG_RV32_DISPLAY_ACCEL
      if (depth == framebuffer_depth)
        framebuffer_depth = 0;
#endif
      if (depth == xip_depth)
        xip_depth = 0;
      if (depth != 0u)
        depth--;
      continue;
    } else if (token == FDT_NOP) {
      continue;
    } else if (token == FDT_END) {
      break;
    } else {
      return false;
    }
  }

#if CONFIG_RV32_USB_FULL_SPEED_ONLY
  if (usb_speed_limited)
    RV32_DIAG("Linux DWC2 maximum-speed limited to full-speed (12 Mbit/s)\n");
  else
    printf("WARNING: USB full-speed limit requested, but the DTB has no "
           "maximum-speed property\n");
#endif
  (void)depth;
#if CONFIG_RV32_USB_PORT_HID
  if (!usb_hidden)
    printf("WARNING: the DTB has no snps,dwc2 USB node to hide from Linux\n");
#endif
#if CONFIG_RV32_DISPLAY_ACCEL
  if (framebuffer_described)
    RV32_DIAG("Linux framebuffer described as %ux%u RGB565\n",
              (unsigned int)DISPLAY_FB_WIDTH, (unsigned int)DISPLAY_FB_HEIGHT);
#endif
  return rng_seeded;
}

/* An interrupt is serviced by the CPU that allocated it, and CPU0 belongs
 * to the emulator: device setup that allocates one runs here on CPU1. */
static struct {
  int (*init)(void);
  int result;
} host_cpu1_call;
static StaticSemaphore_t host_cpu1_done_storage;

static void host_cpu1_call_task(void *argument) {
  host_cpu1_call.result = host_cpu1_call.init();
  xSemaphoreGive((SemaphoreHandle_t)argument);
  vTaskDelete(NULL);
}

int HostRunOnCpu1(int (*init)(void)) {
#if CONFIG_FREERTOS_NUMBER_OF_CORES > 1
  SemaphoreHandle_t done =
      xSemaphoreCreateBinaryStatic(&host_cpu1_done_storage);

  host_cpu1_call.init = init;
  host_cpu1_call.result = -1;
  if (xTaskCreatePinnedToCore(host_cpu1_call_task, "cpu1_init", 8192, done,
                              tskIDLE_PRIORITY + 5u, NULL, 1) == pdPASS) {
    xSemaphoreTake(done, portMAX_DELAY);
    return host_cpu1_call.result;
  }
#endif
  return init();
}

int HostInputInit(void) {
#if CONFIG_RV32_USB_PORT_HID
  return usb_hid_input_init();
#else
  return dwc2_passthrough_init();
#endif
}

static void host_uart_hw_write(const uint8_t *buffer, size_t length)
{
  uart_dev_t *console_uart = UART_LL_GET_HW(CONFIG_ESP_CONSOLE_UART_NUM);

  if (__atomic_load_n(&host_uart_driver_ready, __ATOMIC_ACQUIRE)) {
    while (length != 0u) {
      int written = uart_write_bytes(CONFIG_ESP_CONSOLE_UART_NUM, buffer,
                                     length);

      if (written <= 0)
        return;
      buffer += written;
      length -= (size_t)written;
    }
    return;
  }

  /* Early/failure fallback.  Normal operation uses the interrupt-driven path
   * above, so its pacing is independent of CONFIG_FREERTOS_HZ. */
  while (length != 0u) {
    uint32_t available = uart_ll_get_txfifo_len(console_uart);

    if (available == 0u) {
      vTaskDelay(1);
      continue;
    }
    if (available > length)
      available = length;
    uart_ll_write_txfifo(console_uart, buffer, available);
    buffer += available;
    length -= available;
  }
}

static void host_uart_report_drops(void)
{
  uint32_t dropped = __atomic_exchange_n(&host_uart_tx_dropped, 0u,
                                          __ATOMIC_RELAXED);

  if (dropped != 0u) {
    char message[64];
    int length = snprintf(message, sizeof(message),
                          "\r\n[UART TX queue dropped %" PRIu32
                          " bytes]\r\n", dropped);

    if (length > 0)
      host_uart_hw_write((const uint8_t *)message, (size_t)length);
  }
}

/* Supply dips.  Hot-plugging a USB device on this board can pull the 3.3 V
 * rail below even the lowest brownout threshold, and ESP-IDF's handler always
 * restarts.  Keep running when the dip left the chip intact.  A dip deep
 * enough to reset on-chip peripherals shows up as lost UART settings, and a
 * reset Wi-Fi co-processor as a link that does not come back; then restart
 * cleanly.  Unlike a power-on reset, a software restart keeps the latched
 * boot strapping, so it never lands in download mode. */
#define HOST_BROWNOUT_LEVEL     7u /* 2.51 V, the lowest threshold */
#define HOST_BROWNOUT_SETTLE_MS 200u
#define HOST_BROWNOUT_LINK_MS   15000u
#define HOST_POWER_STACK_SIZE   4096u
#define HOST_POWER_PRIORITY     (tskIDLE_PRIORITY + 3u)
/* UART0's fields in the clock registers it shares with LCD and UART1. */
#define HOST_UART0_CLOCK_SELECT_MASK  0x07000000u
#define HOST_UART0_CLOCK_DIVIDER_MASK 0x00ffffffu
/* Bits the driver pulses at run time: FIFO and core resets. */
#define HOST_UART_CONF0_PULSE_MASK    0x00c00000u
#define HOST_UART_CLK_CONF_PULSE_MASK 0x0c000000u

_Static_assert(CONFIG_ESP_CONSOLE_UART_NUM == 0,
               "the dip check reads UART0's clock fields");

struct host_uart_state {
  uint32_t clock_select;
  uint32_t clock_divider;
  uint32_t clk_conf;
  uint32_t clkdiv;
  uint32_t conf0;
};

static volatile uint32_t host_brownout_events;
static TaskHandle_t host_power_task_handle;
static struct host_uart_state host_uart_expected;

static void IRAM_ATTR host_brownout_isr(void *argument)
{
  BaseType_t woken = pdFALSE;

  (void)argument;
  /* One event per dip; the power task re-arms after checking it. */
  LP_ANA_PERI.int_ena.bod_mode0_int_ena = 0;
  brownout_ll_intr_clear();
  host_brownout_events = host_brownout_events + 1u;
  if (host_power_task_handle != NULL)
    vTaskNotifyGiveFromISR(host_power_task_handle, &woken);
  if (woken)
    portYIELD_FROM_ISR();
}

static void host_uart_capture(struct host_uart_state *state)
{
  state->clock_select =
      HP_SYS_CLKRST.peri_clk_ctrl110.val & HOST_UART0_CLOCK_SELECT_MASK;
  state->clock_divider =
      HP_SYS_CLKRST.peri_clk_ctrl111.val & HOST_UART0_CLOCK_DIVIDER_MASK;
  state->clk_conf = UART0.clk_conf.val & ~HOST_UART_CLK_CONF_PULSE_MASK;
  state->clkdiv = UART0.clkdiv_sync.val;
  state->conf0 = UART0.conf0_sync.val & ~HOST_UART_CONF0_PULSE_MASK;
}

static void host_uart_restore(const struct host_uart_state *state)
{
  HP_SYS_CLKRST.peri_clk_ctrl110.val =
      (HP_SYS_CLKRST.peri_clk_ctrl110.val & ~HOST_UART0_CLOCK_SELECT_MASK) |
      state->clock_select;
  HP_SYS_CLKRST.peri_clk_ctrl111.val =
      (HP_SYS_CLKRST.peri_clk_ctrl111.val & ~HOST_UART0_CLOCK_DIVIDER_MASK) |
      state->clock_divider;
  UART0.clk_conf.val = state->clk_conf;
  UART0.clkdiv_sync.val = state->clkdiv;
  UART0.conf0_sync.val = state->conf0;
  uart_ll_update(&UART0);
}

static void __attribute__((noreturn)) host_power_restart(const char *damage)
{
  /* ROM output polls the FIFO, so it works even if the dip cleared the UART
   * driver's interrupt setup. */
  esp_rom_printf("\r\n[Host] supply dip below 2.51 V %s; restarting\r\n",
                 damage);
  esp_rom_output_tx_wait_idle(CONFIG_ESP_CONSOLE_UART_NUM);
  esp_reset_reason_set_hint(ESP_RST_BROWNOUT);
  esp_rom_software_reset_system();
  __builtin_unreachable();
}

static void host_power_task(void *argument)
{
  (void)argument;
  for (;;) {
    struct host_uart_state found;
    bool link_was_up;
    char message[112];
    int length;

    ulTaskNotifyTake(pdTRUE, portMAX_DELAY);
    link_was_up = virtio_net_bridge_link_up();
    vTaskDelay(pdMS_TO_TICKS(HOST_BROWNOUT_SETTLE_MS));
    host_uart_capture(&found);
    if (memcmp(&found, &host_uart_expected, sizeof(found)) != 0) {
      host_uart_restore(&host_uart_expected);
      host_power_restart("reset on-chip peripherals");
    }

    length = snprintf(message, sizeof(message),
                      "\r\n[Host] supply dipped below 2.51 V; kept running "
                      "(%" PRIu32 " dips since boot)\r\n",
                      host_brownout_events);
    if (length > 0)
      host_uart_hw_write((const uint8_t *)message, (size_t)length);
    brownout_ll_intr_clear();
    brownout_ll_intr_enable(true);

    /* A co-processor reset drops the link for good; an AP hiccup does not. */
    if (link_was_up) {
      TickType_t started = xTaskGetTickCount();

      while (!virtio_net_bridge_link_up() &&
             xTaskGetTickCount() - started <
                 pdMS_TO_TICKS(HOST_BROWNOUT_LINK_MS))
        vTaskDelay(pdMS_TO_TICKS(500));
      if (!virtio_net_bridge_link_up())
        host_power_restart("reset the Wi-Fi co-processor");
    }
  }
}

int HostBrownoutInit(void)
{
  const brownout_hal_config_t config = {
    .threshold = HOST_BROWNOUT_LEVEL,
    .enabled = true,
    .reset_enabled = false,
    .flash_power_down = false,
    .rf_power_down = false,
  };
  BaseType_t power_core = CONFIG_FREERTOS_NUMBER_OF_CORES > 1 ? 1 : 0;

  /* The console UART is fully configured by HostConsoleInit(). */
  host_uart_capture(&host_uart_expected);
  if (xTaskCreatePinnedToCore(host_power_task, "power", HOST_POWER_STACK_SIZE,
                              NULL, HOST_POWER_PRIORITY,
                              &host_power_task_handle, power_core) != pdPASS)
    return -1;

  /* Also disarms the BOD mode1 reset that the bootloader enabled. */
  brownout_hal_config(&config);
  brownout_ll_intr_clear();
  if (esp_intr_alloc_intrstatus(power_supply_periph_signal.irq,
                                ESP_INTR_FLAG_IRAM | ESP_INTR_FLAG_SHARED,
                                (uint32_t)brownout_ll_intr_get_status_reg(),
                                BROWNOUT_DETECTOR_LL_INTERRUPT_MASK,
                                host_brownout_isr, NULL, NULL) != ESP_OK)
    return -1;
  brownout_ll_intr_enable(true);
  return 0;
}

static void host_uart_tx_task(void *argument)
{
  TaskHandle_t init_waiter = (TaskHandle_t)argument;
  uint8_t buffer[HOST_UART_TX_CHUNK_SIZE];
  esp_err_t error = ESP_OK;

  /* Interrupt allocation is core-local.  Install from this pinned task so RX
   * and TX IRQ handling stays on CPU1 with the other host I/O services. */
  if (!uart_is_driver_installed(CONFIG_ESP_CONSOLE_UART_NUM))
    error = uart_driver_install(CONFIG_ESP_CONSOLE_UART_NUM,
                                HOST_UART_DRIVER_RX_SIZE,
                                HOST_UART_DRIVER_TX_SIZE, 0, NULL, 0);
  if (error == ESP_OK) {
    uart_vfs_dev_use_driver(CONFIG_ESP_CONSOLE_UART_NUM);
    __atomic_store_n(&host_uart_driver_ready, true, __ATOMIC_RELEASE);
    host_uart_init_result = 0;
  }
  xTaskNotifyGive(init_waiter);

  for (;;) {
    /* Do not wake CPU1 immediately for every guest newline. The producer may
     * fill the stream continuously, but this background worker drains at most
     * the physical UART's one-tick capacity and always yields a full tick. */
    vTaskDelay(1);
    size_t length = xStreamBufferReceive(host_uart_tx_stream, buffer,
                                         HOST_UART_TX_TICK_BUDGET, 0);

    if (length != 0u)
      host_uart_hw_write(buffer, length);
    if (xStreamBufferBytesAvailable(host_uart_tx_stream) == 0u)
      host_uart_report_drops();
  }
}

int HostConsoleInit(void)
{
  BaseType_t uart_core = CONFIG_FREERTOS_NUMBER_OF_CORES > 1 ? 1 : 0;

#if CONFIG_RV32_HOST_BULK_BUFFERS_PSRAM
  host_uart_tx_stream_storage = psram_host_aux_alloc(
      HOST_UART_TX_QUEUE_SIZE, CONFIG_CACHE_L2_CACHE_LINE_SIZE);
  if (host_uart_tx_stream_storage == NULL) {
    printf("WARNING: Failed to allocate UART TX stream in reserved PSRAM\n");
    return -1;
  }
#endif
  host_uart_tx_stream = xStreamBufferCreateStatic(
      HOST_UART_TX_QUEUE_SIZE, 1u,
      host_uart_tx_stream_storage, &host_uart_tx_stream_control);
  if (host_uart_tx_stream == NULL)
    return -1;

  if (xTaskCreatePinnedToCore(host_uart_tx_task, "uart_tx",
                              HOST_UART_TX_STACK_SIZE,
                              xTaskGetCurrentTaskHandle(),
                              HOST_UART_TX_PRIORITY, NULL,
                              uart_core) != pdPASS) {
    host_uart_tx_stream = NULL;
    return -1;
  }

  ulTaskNotifyTake(pdTRUE, portMAX_DELAY);
  if (host_uart_init_result < 0)
    printf("WARNING: UART interrupt driver unavailable; using polled TX\n");

  RV32_DIAG("UART%d bridge: %d baud, interrupt-buffered RX; "
         "TX %u bytes/tick at priority %u on CPU%d\n",
         CONFIG_ESP_CONSOLE_UART_NUM, CONFIG_ESP_CONSOLE_UART_BAUDRATE,
         (unsigned int)HOST_UART_TX_TICK_BUDGET,
         (unsigned int)HOST_UART_TX_PRIORITY, (int)uart_core);
  return 0;
}

int HostConsoleWrite(const void *buffer, size_t length) {
  size_t queued;

  if (host_uart_tx_stream == NULL)
    return fwrite(buffer, 1, length, stdout);

  queued = xStreamBufferSend(host_uart_tx_stream, buffer, length, 0);
  if (queued != length)
    __atomic_fetch_add(&host_uart_tx_dropped, length - queued,
                       __ATOMIC_RELAXED);
  return (int)queued;
}

int ReadKBByte(void) {
  uint8_t value;
  uart_dev_t *console_uart = UART_LL_GET_HW(CONFIG_ESP_CONSOLE_UART_NUM);

  if (__atomic_load_n(&host_uart_driver_ready, __ATOMIC_ACQUIRE))
    return uart_read_bytes(CONFIG_ESP_CONSOLE_UART_NUM, &value, 1, 0) == 1
               ? value
               : -1;

  if (uart_ll_get_rxfifo_len(console_uart) == 0)
    return -1;

  uart_ll_read_rxfifo(console_uart, &value, 1);
  return value;
}

int IsKBHit(void) {
  uart_dev_t *console_uart = UART_LL_GET_HW(CONFIG_ESP_CONSOLE_UART_NUM);
  size_t buffered = 0;

  if (__atomic_load_n(&host_uart_driver_ready, __ATOMIC_ACQUIRE))
    return uart_get_buffered_data_len(CONFIG_ESP_CONSOLE_UART_NUM, &buffered) ==
                   ESP_OK &&
           buffered != 0u;

  return uart_ll_get_rxfifo_len(console_uart) != 0;
}

static uint8_t *psram_allocation = NULL;
static uint8_t *psram_base = NULL;
static size_t psram_size = 0;
static size_t psram_host_cache_reserved;
static bool psram_host_cache_exported;
#if CONFIG_RV32_HOST_BULK_BUFFERS_PSRAM
static size_t psram_host_aux_used;
#endif

int psram_init(void) {
  /*
   * ESP32-P4 PSRAM is external-DMA capable, but IDF deliberately does not
   * attach MALLOC_CAP_DMA to PSRAM heap regions.  Requiring both flags in a
   * plain heap query therefore reports zero bytes.  Reserve the SPIRAM region
   * here and validate the identity-mapped DMA aperture explicitly below.
   */
  const uint32_t guest_heap_caps = MALLOC_CAP_SPIRAM;
  size_t available_psram;
  size_t largest_psram;
  size_t alloc_size;

  available_psram = heap_caps_get_free_size(guest_heap_caps);
  largest_psram = heap_caps_get_largest_free_block(guest_heap_caps);

  if (available_psram == 0) {
    printf("ERROR: No PSRAM available!\n");
    printf("Check menuconfig: Component config -> ESP PSRAM\n");
    return -1;
  }

  RV32_DIAG("Available PSRAM: %zu bytes (%.2f MB)\n", available_psram,
         available_psram / (1024.0 * 1024.0));
  RV32_DIAG("Largest PSRAM block: %zu bytes (%.2f MB)\n", largest_psram,
         largest_psram / (1024.0 * 1024.0));

  alloc_size = GUEST_RAM_SIZE;

  if (largest_psram < alloc_size) {
    printf("ERROR: Guest RAM requires %zu contiguous PSRAM bytes, but the "
           "largest block is %zu bytes\n",
           alloc_size, largest_psram);
    return -1;
  }

  RV32_DIAG("Attempting to allocate %zu bytes (%.2f MB)...\n", alloc_size,
         alloc_size / (1024.0 * 1024.0));

  psram_allocation = (uint8_t *)heap_caps_malloc(
      alloc_size, guest_heap_caps);

  if (psram_allocation == NULL) {
    printf("ERROR: Failed to allocate PSRAM!\n");
    return -1;
  }

  uintptr_t allocation_start = (uintptr_t)psram_allocation;
  uintptr_t allocation_end = allocation_start + alloc_size;

  if (allocation_start > GUEST_DMA_BASE ||
      allocation_end < GUEST_PHYS_END) {
    printf("ERROR: PSRAM allocation [%p, %p) does not contain the "
           "identity-mapped guest DMA range [0x%08x, 0x%08x)\n",
           psram_allocation, (void *)allocation_end,
           GUEST_DMA_BASE, GUEST_PHYS_END);
    heap_caps_free(psram_allocation);
    psram_allocation = NULL;
    return -1;
  }

  if (!esp_ptr_dma_ext_capable((void *)(uintptr_t)GUEST_DMA_BASE) ||
      !esp_ptr_dma_ext_capable((void *)(uintptr_t)(GUEST_PHYS_END - 1u))) {
    printf("ERROR: Guest DMA range is not accessible to external DMA\n");
    heap_caps_free(psram_allocation);
    psram_allocation = NULL;
    return -1;
  }

  /* Guest physical addresses are real ESP32-P4 PSRAM bus addresses. */
  psram_base = (uint8_t *)(uintptr_t)GUEST_PHYS_BASE;
  psram_size = alloc_size;
  RV32_DIAG("SUCCESS: PSRAM allocated at %p, size: %zu bytes (%.2f MB)\n",
         psram_allocation, psram_size, psram_size / (1024.0 * 1024.0));
  RV32_DIAG("Identity-mapped Linux DMA RAM: 0x%08x-0x%08x\n",
         GUEST_DMA_BASE, GUEST_PHYS_END - 1u);

  /*
   * Do not clear all 30 MiB here.  The Image's effective area is cleared by
   * load_images() (including BSS), the DTB is overwritten, and Linux zeros
   * pages before exposing them to userspace.  Avoiding this redundant pass
   * also leaves more memory bandwidth for LCD initialization.
   */
  RV32_DIAG("PSRAM guest aperture reserved successfully!\n");
#if CONFIG_RV32_HOST_BULK_BUFFERS_PSRAM
  RV32_DIAG("Reserved host PSRAM auxiliary pool: %u KiB\n",
         (unsigned int)(HOST_AUXILIARY_SIZE / 1024u));
#endif

  return 0;
}

int psram_read(uint32_t addr, void *buf, int len) {
  if (psram_base == NULL) {
    printf("ERROR: psram_read called before psram_init!\n");
    return -1;
  }

  if (len < 0 || addr < KERNEL_LOAD_OFFSET || addr > psram_size ||
      (size_t)len > psram_size - addr) {
    printf("ERROR: psram_read out of bounds: addr=0x%lx, len=%d, size=%zu\n",
           (unsigned long)addr, len, psram_size);
    return -1;
  }

  memcpy(buf, psram_base + addr, len);
  return len;
}

int psram_write(uint32_t addr, void *buf, int len) {
  if (psram_base == NULL) {
    printf("ERROR: psram_write called before psram_init!\n");
    return -1;
  }

  if (len < 0 || addr < KERNEL_LOAD_OFFSET || addr > psram_size ||
      (size_t)len > psram_size - addr) {
    printf("ERROR: psram_write out of bounds: addr=0x%lx, len=%d, size=%zu\n",
           (unsigned long)addr, len, psram_size);
    return -1;
  }

  memcpy(psram_base + addr, buf, len);
  return len;
}

void *psram_get_base(void) { return psram_base; }

size_t psram_get_size(void) { return psram_size; }

static bool psram_get_host_cache_bounds(uintptr_t *region_start,
                                        uintptr_t *region_end) {
  const uintptr_t alignment = 64u;
  uintptr_t start;
  uintptr_t end = GUEST_DMA_BASE;

  if (region_start == NULL || region_end == NULL || psram_allocation == NULL)
    return false;

  start = ((uintptr_t)psram_allocation + alignment - 1u) &
          ~(alignment - 1u);
#if CONFIG_RV32_NATIVE_JIT
  if (start >= end ||
      end - start <= HOST_JIT_CODE_SIZE + HOST_JIT_METADATA_SIZE)
    return false;
  end -= HOST_JIT_CODE_SIZE + HOST_JIT_METADATA_SIZE;
#endif
#if CONFIG_RV32_HOST_BULK_BUFFERS_PSRAM
  if (start >= end || end - start <= HOST_AUXILIARY_SIZE)
    return false;
  end -= HOST_AUXILIARY_SIZE;
#endif
  if (start >= end) {
    return false;
  }

  *region_start = start;
  *region_end = end;
  return true;
}

bool psram_get_host_cache_region(void **base, size_t *size) {
  uintptr_t start;
  uintptr_t end;

  if (base == NULL || size == NULL ||
      !psram_get_host_cache_bounds(&start, &end) ||
      psram_host_cache_reserved > end - start) {
    if (base != NULL)
      *base = NULL;
    if (size != NULL)
      *size = 0;
    return false;
  }
  end -= psram_host_cache_reserved;
  if (start >= end) {
    *base = NULL;
    *size = 0;
    return false;
  }

  psram_host_cache_exported = true;
  *base = (void *)start;
  *size = end - start;
  return true;
}

void *psram_host_cache_alloc(size_t size, size_t alignment) {
  uintptr_t start;
  uintptr_t end;
  uintptr_t reservation_end;
  uintptr_t reservation_start;

  if (size == 0u || alignment == 0u ||
      (alignment & (alignment - 1u)) != 0u ||
      psram_host_cache_exported ||
      !psram_get_host_cache_bounds(&start, &end) ||
      psram_host_cache_reserved > end - start)
    return NULL;

  reservation_end = end - psram_host_cache_reserved;
  if (size > reservation_end - start)
    return NULL;
  reservation_start = (reservation_end - size) & ~(uintptr_t)(alignment - 1u);
  if (reservation_start < start)
    return NULL;

  psram_host_cache_reserved = end - reservation_start;
  return (void *)reservation_start;
}

bool psram_get_host_display_region(void **base, size_t *size) {
#if defined(CONFIG_RV32_ST7703_DISPLAY) && CONFIG_RV32_ST7703_DISPLAY
  const uintptr_t start = HOST_DISPLAY_RESERVED_BASE;
  const uintptr_t end = start + HOST_DISPLAY_RESERVED_SIZE;
  uintptr_t allocation_start;
  uintptr_t allocation_end;

  if (base == NULL || size == NULL || psram_allocation == NULL)
    return false;
  allocation_start = (uintptr_t)psram_allocation;
  allocation_end = allocation_start + psram_size;
  if (start < allocation_start || end < start || end > allocation_end ||
      end > GUEST_PHYS_END ||
      !esp_ptr_dma_ext_capable((void *)start) ||
      !esp_ptr_dma_ext_capable((void *)(end - 1u))) {
    *base = NULL;
    *size = 0u;
    return false;
  }

  *base = (void *)start;
  *size = end - start;
  return true;
#else
  if (base != NULL)
    *base = NULL;
  if (size != NULL)
    *size = 0u;
  return false;
#endif
}

bool psram_get_host_jit_region(void **base, size_t *size) {
#if CONFIG_RV32_NATIVE_JIT
  const uintptr_t alignment = 128u;
  uintptr_t allocation_start;
  uintptr_t end = GUEST_DMA_BASE;
  uintptr_t start = end - HOST_JIT_CODE_SIZE;

  if (base == NULL || size == NULL || psram_allocation == NULL)
    return false;
  allocation_start = ((uintptr_t)psram_allocation + alignment - 1u) &
                     ~(alignment - 1u);
  if ((start & (alignment - 1u)) != 0u || start < allocation_start) {
    *base = NULL;
    *size = 0;
    return false;
  }
  *base = (void *)start;
  *size = HOST_JIT_CODE_SIZE;
  return true;
#else
  if (base != NULL)
    *base = NULL;
  if (size != NULL)
    *size = 0;
  return false;
#endif
}

bool psram_get_host_jit_metadata_region(void **base, size_t *size) {
#if CONFIG_RV32_NATIVE_JIT
  const uintptr_t alignment = 128u;
  uintptr_t allocation_start;
  uintptr_t end = GUEST_DMA_BASE - HOST_JIT_CODE_SIZE;
  uintptr_t start = end - HOST_JIT_METADATA_SIZE;

  if (base == NULL || size == NULL || psram_allocation == NULL)
    return false;
  allocation_start = ((uintptr_t)psram_allocation + alignment - 1u) &
                     ~(alignment - 1u);
  if ((start & (alignment - 1u)) != 0u || start < allocation_start) {
    *base = NULL;
    *size = 0;
    return false;
  }
  *base = (void *)start;
  *size = HOST_JIT_METADATA_SIZE;
  return true;
#else
  if (base != NULL)
    *base = NULL;
  if (size != NULL)
    *size = 0;
  return false;
#endif
}

void *psram_host_aux_alloc(size_t size, size_t alignment) {
#if CONFIG_RV32_HOST_BULK_BUFFERS_PSRAM
  uintptr_t allocation_start;
  uintptr_t pool_end = GUEST_DMA_BASE;
  uintptr_t pool_start;
  uintptr_t current;
  uintptr_t aligned;

  if (psram_allocation == NULL || size == 0u || alignment == 0u ||
      (alignment & (alignment - 1u)) != 0u)
    return NULL;
#if CONFIG_RV32_NATIVE_JIT
  if (pool_end < HOST_JIT_CODE_SIZE + HOST_JIT_METADATA_SIZE)
    return NULL;
  pool_end -= HOST_JIT_CODE_SIZE + HOST_JIT_METADATA_SIZE;
#endif
  if (pool_end < HOST_AUXILIARY_SIZE)
    return NULL;
  pool_start = pool_end - HOST_AUXILIARY_SIZE;
  allocation_start = (uintptr_t)psram_allocation;
  if (pool_start < allocation_start || psram_host_aux_used > HOST_AUXILIARY_SIZE)
    return NULL;

  current = pool_start + psram_host_aux_used;
  if (current > UINTPTR_MAX - (alignment - 1u))
    return NULL;
  aligned = (current + alignment - 1u) & ~(uintptr_t)(alignment - 1u);
  if (aligned < pool_start || aligned > pool_end || size > pool_end - aligned)
    return NULL;

  psram_host_aux_used = (size_t)(aligned - pool_start) + size;
  return (void *)aligned;
#else
  (void)size;
  (void)alignment;
  return NULL;
#endif
}

int HostDmaCacheSync(uint32_t guest_physical_address, size_t length,
                     enum host_dma_sync_op operation) {
  int flags = ESP_CACHE_MSYNC_FLAG_TYPE_DATA;

  if (length == 0 || guest_physical_address < GUEST_DMA_BASE ||
      guest_physical_address > GUEST_PHYS_END ||
      length > GUEST_PHYS_END - guest_physical_address)
    return -1;

  switch (operation) {
  case HOST_DMA_SYNC_CLEAN:
    flags |= ESP_CACHE_MSYNC_FLAG_DIR_C2M;
    break;
  case HOST_DMA_SYNC_INVALIDATE:
    flags |= ESP_CACHE_MSYNC_FLAG_DIR_M2C;
    break;
  case HOST_DMA_SYNC_FLUSH:
    flags |= ESP_CACHE_MSYNC_FLAG_DIR_C2M |
             ESP_CACHE_MSYNC_FLAG_INVALIDATE;
    break;
  default:
    return -1;
  }

  return esp_cache_msync((void *)(uintptr_t)guest_physical_address,
                         length, flags) == ESP_OK ? 0 : -1;
}

void HostRestart(bool download) {
  if (download) {
    /* Volatile boot selection, not an eFuse change. A full system reset
     * lets ROM reinitialize the USB download peripheral after Linux DWC2. */
    REG_SET_BIT(LP_SYSTEM_REG_SYS_CTRL_REG, LP_SYSTEM_REG_FORCE_DOWNLOAD_BOOT);
    esp_rom_software_reset_system();
    __builtin_unreachable();
  }
  esp_restart();
}

bool verify_kernel_header(void) {
  static uint8_t header[64];

  RV32_DIAG("\n=== Verifying Kernel Header ===\n");
  if (psram_read(KERNEL_LOAD_OFFSET, header, sizeof(header)) < 0)
    return false;

  RV32_DIAG("First 64 bytes of loaded kernel:\n");
  for (int i = 0; i < 64; i++) {
    RV32_DIAG("%02x ", header[i]);
    if ((i + 1) % 16 == 0)
      RV32_DIAG("\n");
  }
  RV32_DIAG("\n");

  uint32_t first_instr = *(uint32_t *)header;
  RV32_DIAG("First instruction: 0x%08lx\n", (unsigned long)first_instr);
  return (!memcmp(&header[0x30], "RISCV", 5));
}

static uint32_t read_be32(const uint8_t *value) {
  return ((uint32_t)value[0] << 24) | ((uint32_t)value[1] << 16) |
         ((uint32_t)value[2] << 8) | value[3];
}

int load_images(int ram_size, int *kern_len) {
  const esp_partition_t *kernel_partition;
  const esp_partition_t *dtb_partition;
  esp_err_t err;
  uint32_t addr;
  size_t partition_size;
  size_t kernel_payload_size = KERNEL_FLASH_SIZE;

  RV32_DIAG("\n=== Loading Kernel from Flash ===\n");

  kernel_partition = esp_partition_find_first(
      ESP_PARTITION_TYPE_DATA, ESP_PARTITION_SUBTYPE_ANY, "kernel");
  if (kernel_partition == NULL) {
    printf("ERROR: 'kernel' partition not found!\n");
    printf("Make sure partition table has 'kernel' partition\n");
    return -1;
  }

  partition_size = kernel_partition->size;

  uint64_t text_offset = 0;
  uint64_t image_size = 0;

  err = esp_partition_read(kernel_partition, 8,
                           &text_offset, sizeof(text_offset));
  if (err != ESP_OK)
    return -1;

  err = esp_partition_read(kernel_partition, 16,
                           &image_size, sizeof(image_size));
  if (err != ESP_OK)
    return -1;

  RV32_DIAG("Kernel text offset: 0x%llx\n",
         (unsigned long long)text_offset);
  RV32_DIAG("Kernel image size:   %llu bytes\n",
         (unsigned long long)image_size);
  RV32_DIAG("Kernel flash payload: %zu bytes\n", kernel_payload_size);

  if (text_offset != KERNEL_LOAD_OFFSET) {
    printf("ERROR: Expected a 32-bit S-mode Image with text offset "
           "0x%08x\n", KERNEL_LOAD_OFFSET);
    return -1;
  }

  if (image_size == 0 || image_size > UINT32_MAX ||
      (uint64_t)KERNEL_LOAD_OFFSET + image_size > DTB_LOAD_OFFSET) {
    printf("ERROR: Kernel overlaps DTB area\n");
    return -1;
  }

  if (kernel_payload_size > kernel_partition->size ||
      kernel_payload_size > image_size) {
    printf("ERROR: Kernel payload size is inconsistent with its partition or "
           "Image header\n");
    return -1;
  }

  partition_size = kernel_payload_size;

  RV32_DIAG("Found kernel partition:\n");
  RV32_DIAG("  Label: %s\n", kernel_partition->label);
  RV32_DIAG("  Address: 0x%lx\n", (unsigned long)kernel_partition->address);
  RV32_DIAG("  Size: %zu bytes (%.2f MB)\n", partition_size,
         partition_size / (1024.0 * 1024.0));

  if ((uint64_t)KERNEL_LOAD_OFFSET + image_size > (size_t)ram_size) {
    printf("ERROR: Kernel does not fit in guest RAM\n");
    return -1;
  }

  if ((uint64_t)KERNEL_LOAD_OFFSET + image_size > psram_get_size()) {
    printf("ERROR: Kernel does not fit in allocated PSRAM\n");
    return -1;
  }

  if (kern_len)
    *kern_len = (int)image_size;

  /* Clear the complete effective Image area, including BSS, on every boot. */
  memset(psram_base + KERNEL_LOAD_OFFSET, 0, (size_t)image_size);

  RV32_DIAG("\nLoading kernel from flash to PSRAM...\n");
  RV32_DIAG("This will take a moment...\n");

  addr = 0;
  size_t remaining = partition_size;

  while (remaining > 0) {
      size_t chunk = remaining < FLASH_COPY_CHUNK_SIZE
                       ? remaining
                       : FLASH_COPY_CHUNK_SIZE;

      err = esp_partition_read(kernel_partition, addr,
                               psram_base + KERNEL_LOAD_OFFSET + addr,
                               chunk);
      if (err != ESP_OK) {
          return -1;
      }

      addr += chunk;
      remaining -= chunk;
  }

  RV32_DIAG("\nKernel loaded successfully from flash!\n");
  RV32_DIAG("Total loaded: %zu bytes (%.2f MB)\n", partition_size,
         partition_size / (1024.0 * 1024.0));

  RV32_DIAG("\n=== Loading DTB from Flash ===\n");

  dtb_partition = esp_partition_find_first(
      ESP_PARTITION_TYPE_DATA,
      ESP_PARTITION_SUBTYPE_ANY,
      "dtb");

  if (dtb_partition == NULL) {
    printf("ERROR: 'dtb' partition not found!\n");
    return -1;
  }

  uint8_t dtb_header[8];
  err = esp_partition_read(dtb_partition, 0, dtb_header,
                           sizeof(dtb_header));
  if (err != ESP_OK) {
    printf("ERROR: Failed reading DTB header: %s\n", esp_err_to_name(err));
    return -1;
  }

  if (read_be32(dtb_header) != 0xd00dfeedu) {
    printf("ERROR: Invalid DTB magic; flash a compiled DTB to the 'dtb' "
           "partition\n");
    return -1;
  }

  size_t dtb_size = read_be32(&dtb_header[4]);
  if (dtb_size < 40 || dtb_size > dtb_partition->size) {
    printf("ERROR: Invalid DTB size: %zu bytes\n", dtb_size);
    return -1;
  }
#if defined(CONFIG_RV32_ST7703_DISPLAY) && CONFIG_RV32_ST7703_DISPLAY
  if (dtb_size > HOST_DISPLAY_DTB_GAP) {
    printf("ERROR: DTB requires %zu bytes, exceeding the %u-byte gap before "
           "the reserved display surface\n",
           dtb_size, (unsigned int)HOST_DISPLAY_DTB_GAP);
    return -1;
  }
#endif

  RV32_DIAG("DTB size: %zu bytes\n", dtb_size);

  if ((uint64_t)DTB_LOAD_OFFSET + dtb_size > (size_t)ram_size) {
    printf("ERROR: DTB does not fit in guest RAM\n");
    return -1;
  }

  if ((uint64_t)DTB_LOAD_OFFSET + dtb_size > psram_get_size()) {
    printf("ERROR: DTB does not fit in allocated PSRAM\n");
    return -1;
  }

  addr = 0;
  remaining = dtb_size;

  while (remaining > 0) {
    size_t chunk = remaining < FLASH_COPY_CHUNK_SIZE
                    ? remaining
                    : FLASH_COPY_CHUNK_SIZE;

    err = esp_partition_read(
        dtb_partition, addr, psram_base + DTB_LOAD_OFFSET + addr, chunk);

    if (err != ESP_OK) {
      printf("ERROR: Failed reading DTB: %s\n",
            esp_err_to_name(err));
      return -1;
    }

    addr += chunk;
    remaining -= chunk;
  }

  psram_map_xip_partition();
  if (!prepare_linux_dtb(psram_base + DTB_LOAD_OFFSET, dtb_size)) {
    printf("ERROR: DTB has no valid 64-byte /chosen/rng-seed property\n");
    return -1;
  }

  RV32_DIAG("DTB loaded at guest address 0x%08x\n",
        GUEST_PHYS_BASE + DTB_LOAD_OFFSET);


  if (!verify_kernel_header())
    printf("WARNING: Couldn't find RISCV magic header!\n");
  else
    RV32_DIAG("Verified RISCV magic header\n");

  return 0;
}
