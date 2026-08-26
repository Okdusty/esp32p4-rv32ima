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
#include "freertos/stream_buffer.h"
#include "freertos/task.h"
#include "hal/uart_ll.h"
#if CONFIG_RV32_EMULATOR_PERF_STATS
#include "soc/cache_reg.h"
#include "soc/soc.h"
#endif
#include "dwc2_passthrough.h"
#include "port.h"
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
static uint8_t host_uart_tx_stream_storage[HOST_UART_TX_QUEUE_SIZE];
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
  printf("L2 next-line prefetch: host [%p, %p)\n", (void *)start,
         (void *)end);
#else
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

static bool seed_linux_rng(uint8_t *dtb, size_t dtb_size)
{
  uint32_t struct_offset;
  uint32_t strings_offset;
  uint32_t struct_size;
  uint32_t strings_size;
  size_t cursor;
  size_t struct_end;

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
        printf("Seeded Linux CRNG with %" PRIu32
               " bytes of fresh P4 hardware entropy\n", length);
        return true;
      }
      cursor = (cursor + length + 3u) & ~3u;
    } else if (token == FDT_END_NODE || token == FDT_NOP) {
      continue;
    } else if (token == FDT_END) {
      break;
    } else {
      return false;
    }
  }

  return false;
}

int HostInputInit(void) {
  return dwc2_passthrough_init();
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

  host_uart_tx_stream = xStreamBufferCreateStatic(
      sizeof(host_uart_tx_stream_storage), 1u,
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

  printf("UART%d bridge: %d baud, interrupt-buffered RX; "
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

  printf("Available PSRAM: %zu bytes (%.2f MB)\n", available_psram,
         available_psram / (1024.0 * 1024.0));
  printf("Largest PSRAM block: %zu bytes (%.2f MB)\n", largest_psram,
         largest_psram / (1024.0 * 1024.0));

  alloc_size = GUEST_RAM_SIZE;

  if (largest_psram < alloc_size) {
    printf("ERROR: Guest RAM requires %zu contiguous PSRAM bytes, but the "
           "largest block is %zu bytes\n",
           alloc_size, largest_psram);
    return -1;
  }

  printf("Attempting to allocate %zu bytes (%.2f MB)...\n", alloc_size,
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
  printf("SUCCESS: PSRAM allocated at %p, size: %zu bytes (%.2f MB)\n",
         psram_allocation, psram_size, psram_size / (1024.0 * 1024.0));
  printf("Identity-mapped Linux DMA RAM: 0x%08x-0x%08x\n",
         GUEST_DMA_BASE, GUEST_PHYS_END - 1u);

  /*
   * Do not clear all 30 MiB here.  The Image's effective area is cleared by
   * load_images() (including BSS), the DTB is overwritten, and Linux zeros
   * pages before exposing them to userspace.  Avoiding this redundant pass
   * also leaves more memory bandwidth for LCD initialization.
   */
  printf("PSRAM guest aperture reserved successfully!\n");

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

bool verify_kernel_header(void) {
  static uint8_t header[64];

  printf("\n=== Verifying Kernel Header ===\n");
  if (psram_read(KERNEL_LOAD_OFFSET, header, sizeof(header)) < 0)
    return false;

  printf("First 64 bytes of loaded kernel:\n");
  for (int i = 0; i < 64; i++) {
    printf("%02x ", header[i]);
    if ((i + 1) % 16 == 0)
      printf("\n");
  }
  printf("\n");

  uint32_t first_instr = *(uint32_t *)header;
  printf("First instruction: 0x%08lx\n", (unsigned long)first_instr);
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

  printf("\n=== Loading Kernel from Flash ===\n");

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

  printf("Kernel text offset: 0x%llx\n",
         (unsigned long long)text_offset);
  printf("Kernel image size:   %llu bytes\n",
         (unsigned long long)image_size);
  printf("Kernel flash payload: %zu bytes\n", kernel_payload_size);

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

  printf("Found kernel partition:\n");
  printf("  Label: %s\n", kernel_partition->label);
  printf("  Address: 0x%lx\n", (unsigned long)kernel_partition->address);
  printf("  Size: %zu bytes (%.2f MB)\n", partition_size,
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

  printf("\nLoading kernel from flash to PSRAM...\n");
  printf("This will take a moment...\n");

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

  printf("\nKernel loaded successfully from flash!\n");
  printf("Total loaded: %zu bytes (%.2f MB)\n", partition_size,
         partition_size / (1024.0 * 1024.0));

  printf("\n=== Loading DTB from Flash ===\n");

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

  printf("DTB size: %zu bytes\n", dtb_size);

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

  if (!seed_linux_rng(psram_base + DTB_LOAD_OFFSET, dtb_size)) {
    printf("ERROR: DTB has no valid 64-byte /chosen/rng-seed property\n");
    return -1;
  }

  printf("DTB loaded at guest address 0x%08x\n",
        GUEST_PHYS_BASE + DTB_LOAD_OFFSET);


  printf("%s\n",
          verify_kernel_header()
              ? "Verified RISCV magic header"
              : "Couldn't find RISCV magic header!");

  return 0;
}
