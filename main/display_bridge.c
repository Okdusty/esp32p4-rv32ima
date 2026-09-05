/*
 * Backend-neutral paravirtual RGB565 framebuffer bridge.
 *
 * Linux sees a logical RGB565 simple-framebuffer at DISPLAY_FB_GUEST_BASE.
 * Guest reads and writes are redirected by the emulator's MMIO callbacks.
 * Backends with continuously scanned memory use a host-owned render surface
 * and publish it with the ESP32-P4 PPA; other backends render directly.  A
 * separate generic staging aperture can also scale and center smaller packed
 * surfaces without changing tty0's native geometry.
 *
 * SPDX-License-Identifier: BSD-3-Clause
 */

#include "display_bridge.h"
#include "display_backend.h"

#include <stdio.h>
#include <string.h>

#include "driver/ppa.h"
#include "esp_attr.h"
#include "esp_cache.h"
#if CONFIG_RV32_HOST_PERF_STATS
#include "esp_cpu.h"
#endif
#include "esp_err.h"
#include "esp_heap_caps.h"
#include "esp_private/esp_cache_private.h"
#include "esp_timer.h"
#include "freertos/FreeRTOS.h"
#include "freertos/semphr.h"
#include "freertos/stream_buffer.h"
#include "freertos/task.h"
#include "psram.h"

/*
 * Dirty framebuffer writes accumulate until the panel's VSYNC interrupt. The
 * CPU1 service merges wide regions and cleans sparse glyph rows independently.
 */
#define LCD_TASK_STACK_SIZE 6144u
/* SDIO and virtio use priorities 20..23 on CPU1.  Keep display work below
 * that event-driven packet path: framebuffer cache cleans can be long, while
 * disconnected SDIO workers block and therefore do not starve this task. */
#define LCD_TASK_PRIORITY 19u
#define LCD_NOTIFY_VSYNC (1u << 0)
#define LCD_NOTIFY_PV_COMMAND (1u << 1)
#define LCD_NOTIFY_TERMINAL (1u << 2)

#define DISPLAY_PV_FIFO_DEPTH CONFIG_RV32_DISPLAY_COMMAND_FIFO_DEPTH
#define DISPLAY_PV_PAYLOAD_SIZE CONFIG_RV32_DISPLAY_PAYLOAD_BLOCK_SIZE
#define DISPLAY_PV_PAYLOAD_BLOCK_COUNT CONFIG_RV32_DISPLAY_PAYLOAD_BLOCK_COUNT
#define DISPLAY_PV_INLINE_PAYLOAD_SIZE 128u
#define DISPLAY_TILE_MAX_COUNT 256u
/* Keep the font cache independent from the streaming-write transfer size.
 * A 16 KiB cache preserves the previous maximum font payload while allowing
 * larger bulk blocks to reduce guest/host mailbox traffic without reserving
 * otherwise unused font storage. */
#define DISPLAY_TILE_FONT_SIZE (16u * 1024u)
#define DISPLAY_TERMINAL_TILE_WIDTH 8u
#if DISPLAY_FB_HEIGHT < 400u
/* Linux get_default_font() prefers VGA8x8 whenever yres is below 400. Keep
 * the direct HVC renderer on the same tile geometry as fbcon's font upload. */
#define DISPLAY_TERMINAL_TILE_HEIGHT 8u
#else
#define DISPLAY_TERMINAL_TILE_HEIGHT 16u
#endif
#define DISPLAY_TERMINAL_COLUMNS \
  (DISPLAY_FB_WIDTH / DISPLAY_TERMINAL_TILE_WIDTH)
#define DISPLAY_TERMINAL_ROWS \
  (DISPLAY_FB_HEIGHT / DISPLAY_TERMINAL_TILE_HEIGHT)
#define DISPLAY_TERMINAL_STREAM_SIZE 8192u
#define DISPLAY_TERMINAL_SERVICE_BYTES DISPLAY_TERMINAL_STREAM_SIZE
#define DISPLAY_TERMINAL_READ_CHUNK 512u
#define DISPLAY_TERMINAL_PARAM_COUNT 16u
#define DISPLAY_HOST_BULK_STORAGE_SIZE                                  \
  (DISPLAY_PV_PAYLOAD_SIZE * DISPLAY_PV_PAYLOAD_BLOCK_COUNT +          \
   DISPLAY_TILE_FONT_SIZE + DISPLAY_TERMINAL_STREAM_SIZE)
/*
 * One cache writeback command has a much larger fixed cost than scanning the
 * stride gap between adjacent glyph rows. Merge at most one maximum-height
 * tile band at a time: this collapses row writebacks while bounding the extra
 * PSRAM range cleaned for a narrow update.
 */
#define DISPLAY_CACHE_MSYNC_MERGE_GAP DISPLAY_FB_STRIDE
#define DISPLAY_CACHE_MSYNC_MAX_ROWS 32u
#define DISPLAY_GPU_STAGE_SIZE (64u * 1024u)

_Static_assert(DISPLAY_PV_PAYLOAD_SIZE <= DISPLAY_ACCEL_STAGE_SIZE,
               "FIFO payload must fit in the guest staging aperture");
_Static_assert(DISPLAY_PV_INLINE_PAYLOAD_SIZE <= DISPLAY_PV_PAYLOAD_SIZE,
               "inline FIFO payload must fit in a payload block");
_Static_assert(DISPLAY_TILE_FONT_SIZE <= DISPLAY_PV_PAYLOAD_SIZE,
               "tile font cache must fit in one payload block");
_Static_assert(DISPLAY_PV_PAYLOAD_SIZE *
                       (DISPLAY_PV_PAYLOAD_BLOCK_COUNT + 1u) <=
                   160u * 1024u,
               "FIFO payload pool and tile cache exceed the RAM budget");
_Static_assert(DISPLAY_PV_PAYLOAD_BLOCK_COUNT <= 32u,
               "payload free mask is 32 bits");

typedef struct {
  uint32_t sequence;
  uint32_t operation;
  uint32_t args[8];
} display_pv_command_t;

typedef struct {
  display_pv_command_t command;
  uint32_t payload_length;
  uint8_t *payload;
  uint32_t payload_free_bit;
  /* Most tty updates carry one or a few 32-bit tile indices. Keep them in the
   * descriptor's cache-line padding instead of occupying a 16 KiB block. */
  uint8_t inline_payload[DISPLAY_PV_INLINE_PAYLOAD_SIZE];
} __attribute__((aligned(CONFIG_CACHE_L2_CACHE_LINE_SIZE)))
    display_pv_queue_entry_t;

typedef struct {
  uint16_t pixels[4];
} display_glyph_nibble_t;

typedef struct {
  uint16_t foreground;
  uint16_t background;
  uint8_t glyph;
  uint8_t dirty;
} display_terminal_cell_t;

typedef enum {
  DISPLAY_TERMINAL_GROUND,
  DISPLAY_TERMINAL_ESCAPE,
  DISPLAY_TERMINAL_CSI,
  DISPLAY_TERMINAL_OSC,
  DISPLAY_TERMINAL_OSC_ESCAPE,
} display_terminal_parse_state_t;

typedef struct {
  display_terminal_cell_t *cells;
  uint16_t column;
  uint16_t row;
  uint16_t saved_column;
  uint16_t saved_row;
  uint16_t scroll_top;
  uint16_t scroll_bottom;
  uint16_t row_origin;
  uint16_t foreground;
  uint16_t background;
  uint16_t default_foreground;
  uint16_t default_background;
  uint16_t parameters[DISPLAY_TERMINAL_PARAM_COUNT];
  uint8_t parameter_count;
  uint8_t utf8_remaining;
  uint32_t utf8_codepoint;
  bool parameter_present;
  bool private_csi;
  bool bold;
  bool inverse;
  bool cursor_visible;
  bool wrap_pending;
  bool active;
  bool pixels_valid;
  display_terminal_parse_state_t parse_state;
} display_terminal_state_t;

/* Rendering targets a host-only staging surface when the backend exposes a
 * live scanout allocation. scanout_framebuffer remains owned by the backend;
 * framebuffer is the logical surface mapped into the guest. */
static uint8_t *framebuffer;
static uint8_t *scanout_framebuffer;
static bool framebuffer_present_pending;
static uint16_t framebuffer_present_first_column = DISPLAY_FB_WIDTH;
static uint16_t framebuffer_present_last_column;
static uint16_t framebuffer_present_first_row = DISPLAY_FB_HEIGHT;
static uint16_t framebuffer_present_last_row;
/* CPU0 owns the uncommitted producer range; CPU1 only touches the pending
 * range. The mutex is uncontended unless CPU1 is taking a coherent snapshot
 * of the logical render surface. */
static bool producer_dirty;
/* Direct mmap writers can span many emulator execution slices. Linux timer
 * interrupts and task switches create store-free slices inside a scanline
 * copy, so a slice boundary is not an end-of-frame signal. Legacy clients
 * which do not issue the explicit fence are published only after a bounded
 * wall-clock quiet period. */
static int64_t producer_last_write_us;
static size_t producer_dirty_start = DISPLAY_FB_SIZE;
static size_t producer_dirty_end;
static bool pending_dirty;
static size_t pending_dirty_start = DISPLAY_FB_SIZE;
static size_t pending_dirty_end;
static StaticSemaphore_t framebuffer_mutex_storage;
static SemaphoreHandle_t framebuffer_mutex;
static uint8_t *ppa_staging;
#if defined(CONFIG_RV32_VIRTIO_GPU) && CONFIG_RV32_VIRTIO_GPU
static uint8_t *gpu_staging;
static StaticSemaphore_t scanout_mutex_storage;
static SemaphoreHandle_t scanout_mutex;
#endif
static ppa_client_handle_t ppa_srm;
static ppa_client_handle_t ppa_fill;
static bool ppa_ready;
static bool ppa_fill_ready;
/* A PPA fill reaches the logical framebuffer immediately. If the same
 * command adds CPU-drawn pixels, publish those pixels as one composite. */
static bool ppa_fill_touched_framebuffer;
static bool ppa_frame_pending;
static bool ppa_stop_pending;
static bool ppa_accel_active;
static uint32_t ppa_source_size;
static uint16_t ppa_pending_width;
static uint16_t ppa_pending_height;
static uint16_t ppa_active_width;
static uint16_t ppa_active_height;
static uint32_t frame_sync_requested;
static uint32_t frame_sync_completed;
static display_pv_command_t pv_registers;
static display_pv_queue_entry_t pv_queue_entries[DISPLAY_PV_FIFO_DEPTH];
/* Payload blocks are independent of command descriptors.  Control-only work
 * can still enter the FIFO while bitmap producers are applying backpressure. */
#if CONFIG_RV32_HOST_BULK_BUFFERS_PSRAM
static uint8_t (*pv_payload_blocks)[DISPLAY_PV_PAYLOAD_SIZE];
#else
static uint8_t pv_payload_blocks[DISPLAY_PV_PAYLOAD_BLOCK_COUNT]
                                [DISPLAY_PV_PAYLOAD_SIZE]
    __attribute__((aligned(CONFIG_CACHE_L2_CACHE_LINE_SIZE)));
#endif
/* CPU0 is the only producer and CPU1 is the only consumer. Monotonic indices
 * therefore provide a bounded SPSC ring without the three FreeRTOS queues and
 * their cross-core locks on every framebuffer command. Keep independently
 * written indices on different P4 cache lines to avoid false sharing. */
static uint32_t pv_producer_head
    __attribute__((aligned(CONFIG_CACHE_L2_CACHE_LINE_SIZE)));
static uint32_t pv_consumer_tail
    __attribute__((aligned(CONFIG_CACHE_L2_CACHE_LINE_SIZE)));
static uint32_t pv_payload_free_mask
    __attribute__((aligned(CONFIG_CACHE_L2_CACHE_LINE_SIZE)));
static bool pv_fifo_ready;
static uint32_t pv_accepted_sequence;
static uint32_t pv_accepted_status = DISPLAY_PV_STATUS_FAILED;
static uint32_t pv_completed_sequence;
static uint32_t pv_completed_status;
static uint32_t pv_fifo_errors;
static uint8_t *guest_memory_host;
static uint32_t guest_memory_base;
static size_t guest_memory_size;
static uint32_t pv_shared_command_address;
#if CONFIG_RV32_HOST_BULK_BUFFERS_PSRAM
static uint8_t *tile_font;
#else
static uint8_t tile_font[DISPLAY_TILE_FONT_SIZE];
#endif
static uint16_t tile_width;
static uint16_t tile_height;
static uint16_t tile_count;
static uint16_t tile_bytes;
static bool tile_font_valid;
static uint8_t tile_blank[DISPLAY_TILE_MAX_COUNT];
static bool tile_cursor_active;
static bool tile_cursor_drawn;
static uint16_t tile_cursor_x;
static uint16_t tile_cursor_y;
static uint8_t tile_cursor_shape;
static uint16_t tile_cursor_color;
/* Accelerated CPU writes are tracked per row.  This avoids cleaning the
 * untouched 1424-byte stride gap sixteen times over for an 8x16 glyph. */
static uint16_t accel_dirty_start[DISPLAY_FB_HEIGHT];
static uint16_t accel_dirty_end[DISPLAY_FB_HEIGHT];
static uint16_t accel_dirty_first = DISPLAY_FB_HEIGHT;
static uint16_t accel_dirty_last;
static display_glyph_nibble_t glyph_nibble_lut[16];
static uint16_t glyph_lut_foreground;
static uint16_t glyph_lut_background;
static bool glyph_lut_valid;
static portMUX_TYPE dirty_lock = portMUX_INITIALIZER_UNLOCKED;
static int display_init_result = -1;
static TaskHandle_t display_task_handle;
static StaticStreamBuffer_t terminal_stream_control;
#if CONFIG_RV32_HOST_BULK_BUFFERS_PSRAM
static uint8_t *terminal_stream_storage;
#else
static uint8_t terminal_stream_storage[DISPLAY_TERMINAL_STREAM_SIZE];
#endif
static StreamBufferHandle_t terminal_stream;
static display_terminal_state_t terminal;
static uint32_t terminal_cursor_vsyncs;
static uint32_t terminal_cursor_period_frames = 30u;
/* The accelerated terminal and ordinary /dev/fb0 mmap both target the same
 * logical render surface. CPU0 only requests an ownership change; CPU1
 * applies it without waiting for either core. */
static bool raw_framebuffer_claim_pending;
static bool terminal_owns_framebuffer;

#if CONFIG_RV32_HOST_PERF_STATS
static struct display_bridge_perf_stats display_perf;

#define DISPLAY_PERF_ADD(field, value)                                      \
  ((void)__atomic_fetch_add(&display_perf.field, (uint32_t)(value),         \
                            __ATOMIC_RELAXED))

static void display_perf_update_fifo_high_water(uint32_t depth) {
  uint32_t previous =
      __atomic_load_n(&display_perf.fifo_high_water, __ATOMIC_RELAXED);

  while (depth > previous &&
         !__atomic_compare_exchange_n(&display_perf.fifo_high_water,
                                      &previous, depth, true,
                                      __ATOMIC_RELAXED,
                                      __ATOMIC_RELAXED)) {
  }
}

static void display_perf_update_max(uint32_t *field, uint32_t value) {
  uint32_t previous = __atomic_load_n(field, __ATOMIC_RELAXED);

  while (value > previous &&
         !__atomic_compare_exchange_n(field, &previous, value, true,
                                      __ATOMIC_RELAXED,
                                      __ATOMIC_RELAXED)) {
  }
}

static void display_perf_record_frame(uint32_t duration_us) {
  uint32_t previous =
      __atomic_load_n(&display_perf.frame_max_us, __ATOMIC_RELAXED);

  DISPLAY_PERF_ADD(frame_total_us, duration_us);
  DISPLAY_PERF_ADD(frame_samples, 1u);
  while (duration_us > previous &&
         !__atomic_compare_exchange_n(&display_perf.frame_max_us, &previous,
                                      duration_us, true, __ATOMIC_RELAXED,
                                      __ATOMIC_RELAXED)) {
  }
}
#else
#define DISPLAY_PERF_ADD(field, value) do { } while (0)
#define display_perf_update_fifo_high_water(depth) do { } while (0)
#define display_perf_record_frame(duration_us) do { } while (0)
#endif

static inline esp_err_t display_cache_clean(void *address, size_t length) {
#if CONFIG_RV32_HOST_PERF_STATS
  int64_t started = esp_timer_get_time();
#endif
  esp_err_t err = esp_cache_msync(address, length,
                                  ESP_CACHE_MSYNC_FLAG_DIR_C2M |
                                      ESP_CACHE_MSYNC_FLAG_UNALIGNED);

#if CONFIG_RV32_HOST_PERF_STATS
  DISPLAY_PERF_ADD(cache_us, esp_timer_get_time() - started);
  if (err == ESP_OK) {
    DISPLAY_PERF_ADD(cache_syncs, 1u);
    DISPLAY_PERF_ADD(cache_bytes, length);
  }
#endif
  return err;
}

static bool IRAM_ATTR display_vsync_callback(void *context, bool from_isr) {
  TaskHandle_t task = display_task_handle;

  (void)context;
  if (task == NULL)
    return false;
  if (from_isr) {
    BaseType_t higher_priority_task_woken = pdFALSE;

    xTaskNotifyFromISR(task, LCD_NOTIFY_VSYNC, eSetBits,
                      &higher_priority_task_woken);
    return higher_priority_task_woken == pdTRUE;
  }
  (void)xTaskNotify(task, LCD_NOTIFY_VSYNC, eSetBits);
  return false;
}

static uint32_t display_pv_queue_count(void) {
  uint32_t head = __atomic_load_n(&pv_producer_head, __ATOMIC_ACQUIRE);
  uint32_t tail = __atomic_load_n(&pv_consumer_tail, __ATOMIC_ACQUIRE);

  return head - tail;
}

static uint32_t display_pv_queue_free(void) {
  uint32_t used = display_pv_queue_count();

  return used < DISPLAY_PV_FIFO_DEPTH ? DISPLAY_PV_FIFO_DEPTH - used : 0;
}

static bool display_pv_commands_pending(void) {
  uint32_t tail =
      __atomic_load_n(&pv_consumer_tail, __ATOMIC_RELAXED);
  uint32_t head =
      __atomic_load_n(&pv_producer_head, __ATOMIC_ACQUIRE);

  return tail != head;
}

static uint32_t display_pv_payload_free_count(void) {
  return (uint32_t)__builtin_popcount(
      __atomic_load_n(&pv_payload_free_mask, __ATOMIC_ACQUIRE));
}

static uint8_t *display_pv_payload_take(uint32_t *free_bit) {
  uint32_t available =
      __atomic_load_n(&pv_payload_free_mask, __ATOMIC_ACQUIRE);

  while (available != 0) {
    uint32_t selected = available & (0u - available);
    uint32_t remaining = available & ~selected;

    if (__atomic_compare_exchange_n(&pv_payload_free_mask, &available,
                                    remaining, false, __ATOMIC_ACQ_REL,
                                    __ATOMIC_ACQUIRE)) {
      *free_bit = selected;
      return pv_payload_blocks[__builtin_ctz(selected)];
    }
  }
  *free_bit = 0;
  return NULL;
}

static bool display_pv_queue_init(void) {
  pv_producer_head = 0;
  pv_consumer_tail = 0;
  pv_payload_free_mask =
      DISPLAY_PV_PAYLOAD_BLOCK_COUNT == 32u
          ? UINT32_MAX
          : (1u << DISPLAY_PV_PAYLOAD_BLOCK_COUNT) - 1u;
  pv_fifo_ready = true;

  for (uint32_t row = 0; row < DISPLAY_FB_HEIGHT; row++)
    accel_dirty_start[row] = DISPLAY_FB_WIDTH;
  memset(pv_queue_entries, 0, sizeof(pv_queue_entries));
  return true;
}

static bool display_bulk_storage_init(void) {
#if CONFIG_RV32_HOST_BULK_BUFFERS_PSRAM
  uint8_t *storage = psram_host_aux_alloc(
      DISPLAY_HOST_BULK_STORAGE_SIZE, CONFIG_CACHE_L2_CACHE_LINE_SIZE);

  if (storage == NULL)
    return false;
  pv_payload_blocks =
      (uint8_t (*)[DISPLAY_PV_PAYLOAD_SIZE])(void *)storage;
  storage += DISPLAY_PV_PAYLOAD_SIZE * DISPLAY_PV_PAYLOAD_BLOCK_COUNT;
  tile_font = storage;
  storage += DISPLAY_TILE_FONT_SIZE;
  terminal_stream_storage = storage;
  printf("Display bulk storage: %u KiB reserved PSRAM\n",
         (unsigned int)(DISPLAY_HOST_BULK_STORAGE_SIZE / 1024u));
#endif
  return true;
}

static bool display_terminal_init(uint32_t refresh_hz);

static esp_err_t display_ppa_init(void) {
  const ppa_client_config_t srm_config = {
      .oper_type = PPA_OPERATION_SRM,
      .max_pending_trans_num = 1,
  };
  const ppa_client_config_t fill_config = {
      .oper_type = PPA_OPERATION_FILL,
      .max_pending_trans_num = 1,
  };
  esp_err_t srm_err;
  esp_err_t fill_err;
  esp_err_t alignment_err;
  size_t framebuffer_alignment;

  alignment_err = esp_cache_get_alignment(MALLOC_CAP_SPIRAM | MALLOC_CAP_DMA,
                                          &framebuffer_alignment);
  if (alignment_err != ESP_OK)
    return alignment_err;
  if (framebuffer_alignment == 0u ||
      (uintptr_t)framebuffer % framebuffer_alignment != 0u ||
      DISPLAY_FB_SIZE % framebuffer_alignment != 0u) {
    printf("WARNING: PPA disabled: framebuffer %p/%u is not aligned to "
           "%u-byte PSRAM cache lines\n",
           framebuffer, (unsigned int)DISPLAY_FB_SIZE,
           (unsigned int)framebuffer_alignment);
    return ESP_ERR_INVALID_SIZE;
  }

  ppa_staging = heap_caps_calloc(1, DISPLAY_ACCEL_STAGE_SIZE,
                                 MALLOC_CAP_SPIRAM | MALLOC_CAP_DMA);
  if (ppa_staging == NULL)
    return ESP_ERR_NO_MEM;

  srm_err = ppa_register_client(&srm_config, &ppa_srm);
  fill_err = ppa_register_client(&fill_config, &ppa_fill);
  ppa_ready = srm_err == ESP_OK;
  ppa_fill_ready = fill_err == ESP_OK;
  printf("Display acceleration ready: RGB565 staging %u bytes, "
         "PPA scale=%s fill=%s\n",
         DISPLAY_ACCEL_STAGE_SIZE, ppa_ready ? "yes" : "no",
         ppa_fill_ready ? "yes" : "no");
  return ESP_OK;
}

static esp_err_t display_gpu_import_init(void) {
#if defined(CONFIG_RV32_VIRTIO_GPU) && CONFIG_RV32_VIRTIO_GPU
  gpu_staging = heap_caps_aligned_calloc(
      64u, 1u, DISPLAY_GPU_STAGE_SIZE, MALLOC_CAP_SPIRAM | MALLOC_CAP_DMA);
  if (gpu_staging == NULL)
    return ESP_ERR_NO_MEM;
#endif
  return ESP_OK;
}

static int display_hardware_init(void) {
  display_backend_surface_t surface = { 0 };
  esp_err_t err;
  size_t framebuffer_alignment = CONFIG_CACHE_L2_CACHE_LINE_SIZE;

  printf("\n=== Initializing RGB565 display backend ===\n");
  err = display_backend_init(display_vsync_callback, NULL, &surface);
  if (err != ESP_OK)
    goto fail;
  if (surface.framebuffer == NULL || surface.width != DISPLAY_FB_WIDTH ||
      surface.height != DISPLAY_FB_HEIGHT ||
      surface.stride != DISPLAY_FB_STRIDE ||
      surface.format != DISPLAY_BACKEND_FORMAT_RGB565 ||
      surface.framebuffer_size < DISPLAY_FB_SIZE) {
    err = ESP_ERR_INVALID_SIZE;
    goto fail;
  }
  scanout_framebuffer = surface.framebuffer;
  framebuffer = scanout_framebuffer;
  if (surface.framebuffer_is_live_scanout) {
    const char *render_surface_source = "host-cache PSRAM";
    void *reserved_surface = NULL;
    size_t reserved_surface_size = 0u;
    size_t required_alignment;

    err = esp_cache_get_alignment(MALLOC_CAP_SPIRAM, &required_alignment);
    if (err == ESP_OK && required_alignment != 0u)
      framebuffer_alignment = required_alignment;
    if (psram_get_host_display_region(&reserved_surface,
                                      &reserved_surface_size) &&
        reserved_surface_size >= DISPLAY_FB_SIZE &&
        (uintptr_t)reserved_surface % framebuffer_alignment == 0u) {
      framebuffer = reserved_surface;
      render_surface_source = "Linux-reserved PSRAM";
    } else {
      framebuffer = psram_host_cache_alloc(DISPLAY_FB_SIZE,
                                           framebuffer_alignment);
    }
    if (framebuffer == NULL) {
      framebuffer = scanout_framebuffer;
      printf("WARNING: Host render surface unavailable; guest writes will "
             "target live scanout\n");
    } else {
      printf("Display buffering: %s render surface %p -> live scanout %p "
             "(%u bytes, %u-byte alignment)\n",
             render_surface_source, framebuffer, scanout_framebuffer,
             (unsigned int)DISPLAY_FB_SIZE,
             (unsigned int)framebuffer_alignment);
    }
  }

  memset(framebuffer, 0, DISPLAY_FB_SIZE);
  err = display_cache_clean(framebuffer, DISPLAY_FB_SIZE);
  if (err != ESP_OK)
    goto fail;
  if (scanout_framebuffer != framebuffer) {
    memset(scanout_framebuffer, 0, DISPLAY_FB_SIZE);
    err = display_cache_clean(scanout_framebuffer, DISPLAY_FB_SIZE);
    if (err != ESP_OK)
      goto fail;
  }

  err = display_ppa_init();
  if (err != ESP_OK)
    printf("WARNING: PPA display acceleration unavailable: %s\n",
           esp_err_to_name(err));
  err = display_gpu_import_init();
  if (err != ESP_OK)
    printf("WARNING: virtio-gpu scanout staging unavailable: %s\n",
           esp_err_to_name(err));
  if (!display_terminal_init(surface.refresh_hz))
    printf("WARNING: CPU1 terminal renderer unavailable\n");

  err = display_backend_start();
  if (err != ESP_OK)
    goto fail;

  printf("%s ready: %ux%u packed RGB565 at %u Hz, "
         "render %p -> scanout %p mapped at guest 0x%08x (%u bytes), "
         "VSYNC cache-clean delay %d ms, async FIFO %u commands + "
         "%u x %u-byte payloads, "
         "service on CPU%d\n",
         surface.name != NULL ? surface.name : "Display backend",
         DISPLAY_FB_WIDTH, DISPLAY_FB_HEIGHT,
         (unsigned int)surface.refresh_hz, framebuffer, scanout_framebuffer,
         DISPLAY_FB_GUEST_BASE, (unsigned int)DISPLAY_FB_SIZE,
         CONFIG_RV32_DISPLAY_FLUSH_INTERVAL_MS,
         DISPLAY_PV_FIFO_DEPTH, DISPLAY_PV_PAYLOAD_BLOCK_COUNT,
         DISPLAY_PV_PAYLOAD_SIZE, (int)xPortGetCoreID());
  return 0;

fail:
  printf("WARNING: Display backend initialization failed: %s; continuing with "
         "serial console only\n",
         esp_err_to_name(err));
  framebuffer = NULL;
  scanout_framebuffer = NULL;
  return -1;
}

static bool display_rect_valid(uint32_t x, uint32_t y, uint32_t width,
                               uint32_t height);

static const uint8_t *display_guest_pointer(uint32_t address, size_t length) {
  size_t offset;

  if (guest_memory_host == NULL || address < guest_memory_base)
    return NULL;
  offset = address - guest_memory_base;
  if (offset > guest_memory_size || length > guest_memory_size - offset)
    return NULL;
  return guest_memory_host + offset;
}

static void display_mark_present_rect(uint32_t x, uint32_t y,
                                      uint32_t width, uint32_t height) {
  if (framebuffer == scanout_framebuffer ||
      !display_rect_valid(x, y, width, height))
    return;
  if (x < framebuffer_present_first_column)
    framebuffer_present_first_column = x;
  if (x + width > framebuffer_present_last_column)
    framebuffer_present_last_column = x + width;
  if (y < framebuffer_present_first_row)
    framebuffer_present_first_row = y;
  if (y + height > framebuffer_present_last_row)
    framebuffer_present_last_row = y + height;
  framebuffer_present_pending = true;
}

static bool display_flush_dirty(void) {
  size_t start;
  size_t end;

  portENTER_CRITICAL(&dirty_lock);
  if (!pending_dirty) {
    portEXIT_CRITICAL(&dirty_lock);
    return true;
  }

  start = pending_dirty_start;
  end = pending_dirty_end;
  pending_dirty = false;
  pending_dirty_start = DISPLAY_FB_SIZE;
  pending_dirty_end = 0;
  portEXIT_CRITICAL(&dirty_lock);

  if (start >= end || end > DISPLAY_FB_SIZE)
    return true;

  esp_err_t err = ESP_OK;

  if (framebuffer != scanout_framebuffer) {
    size_t first_pixel = start / sizeof(uint16_t);
    size_t last_pixel = (end - 1u) / sizeof(uint16_t);
    uint32_t first_row = first_pixel / DISPLAY_FB_WIDTH;
    uint32_t last_row = last_pixel / DISPLAY_FB_WIDTH;

    if (first_row == last_row) {
      uint32_t first_column = first_pixel % DISPLAY_FB_WIDTH;
      uint32_t last_column = last_pixel % DISPLAY_FB_WIDTH + 1u;

      display_mark_present_rect(first_column, first_row,
                                last_column - first_column, 1u);
    } else {
      /* A contiguous byte range crossing a row boundary touches the right
       * edge of its first row and the left edge of its last. Its rectangular
       * union is therefore full-width; intermediate rows are full as well. */
      display_mark_present_rect(0u, first_row, DISPLAY_FB_WIDTH,
                                last_row - first_row + 1u);
    }
  } else {
    err = display_cache_clean(framebuffer + start, end - start);
  }

  if (err != ESP_OK) {
    /* Preserve the range so a transient synchronization failure retries. */
    portENTER_CRITICAL(&dirty_lock);
    if (start < pending_dirty_start)
      pending_dirty_start = start;
    if (end > pending_dirty_end)
      pending_dirty_end = end;
    pending_dirty = true;
    portEXIT_CRITICAL(&dirty_lock);
    return false;
  }
  DISPLAY_PERF_ADD(framebuffer_publish_events, 1u);
  DISPLAY_PERF_ADD(framebuffer_publish_bytes, end - start);
  return true;
}

static bool display_flush_accel_dirty(void) {
  size_t run_start = 0;
  size_t run_end = 0;
  uint32_t run_first_row = 0;
  bool have_run = false;
  esp_err_t err = ESP_OK;

  if (accel_dirty_first >= DISPLAY_FB_HEIGHT)
    return true;

  if (framebuffer == scanout_framebuffer) {
    for (uint32_t row = accel_dirty_first; row <= accel_dirty_last; row++) {
      uint32_t first = accel_dirty_start[row];
      uint32_t last = accel_dirty_end[row];
      size_t start;
      size_t end;

      if (first >= last)
        continue;
      start = ((size_t)row * DISPLAY_FB_WIDTH + first) * sizeof(uint16_t);
      end = ((size_t)row * DISPLAY_FB_WIDTH + last) * sizeof(uint16_t);
      /* Full-width rectangles are contiguous in memory and can be published
       * by one cache operation regardless of height. Keep the row bound only
       * when merging across untouched stride gaps in narrow glyph updates. */
      if (have_run &&
          (start <= run_end ||
           (row - run_first_row < DISPLAY_CACHE_MSYNC_MAX_ROWS &&
            start <= run_end + DISPLAY_CACHE_MSYNC_MERGE_GAP))) {
        run_end = end;
        continue;
      }
      if (have_run) {
        err = display_cache_clean(framebuffer + run_start,
                                  run_end - run_start);
        if (err != ESP_OK)
          return false;
      }
      run_start = start;
      run_end = end;
      run_first_row = row;
      have_run = true;
    }
    if (have_run)
      err = display_cache_clean(framebuffer + run_start, run_end - run_start);
    if (err != ESP_OK)
      return false;
  } else {
    uint32_t first_column = DISPLAY_FB_WIDTH;
    uint32_t last_column = 0u;

    for (uint32_t row = accel_dirty_first; row <= accel_dirty_last; row++) {
      if (accel_dirty_start[row] < first_column)
        first_column = accel_dirty_start[row];
      if (accel_dirty_end[row] > last_column)
        last_column = accel_dirty_end[row];
    }
    if (first_column < last_column)
      display_mark_present_rect(first_column, accel_dirty_first,
                                last_column - first_column,
                                accel_dirty_last - accel_dirty_first + 1u);
  }

  for (uint32_t row = accel_dirty_first; row <= accel_dirty_last; row++) {
    accel_dirty_start[row] = DISPLAY_FB_WIDTH;
    accel_dirty_end[row] = 0;
  }
  accel_dirty_first = DISPLAY_FB_HEIGHT;
  accel_dirty_last = 0;
  return true;
}

static void display_mark_accel_rect(uint32_t x, uint32_t y, uint32_t width,
                                    uint32_t height) {
  if (!display_rect_valid(x, y, width, height))
    return;

  if (y < accel_dirty_first)
    accel_dirty_first = y;
  if (y + height - 1u > accel_dirty_last)
    accel_dirty_last = y + height - 1u;
  for (uint32_t row = y; row < y + height; row++) {
    if (x < accel_dirty_start[row])
      accel_dirty_start[row] = x;
    if (x + width > accel_dirty_end[row])
      accel_dirty_end[row] = x + width;
  }
}

static void display_mark_accel_byte_range(size_t start, size_t length) {
  size_t first_pixel;
  size_t last_pixel;
  uint32_t first_row;
  uint32_t last_row;

  if (length == 0u || start >= DISPLAY_FB_SIZE ||
      length > DISPLAY_FB_SIZE - start)
    return;
  first_pixel = start / sizeof(uint16_t);
  last_pixel = (start + length - 1u) / sizeof(uint16_t);
  first_row = first_pixel / DISPLAY_FB_WIDTH;
  last_row = last_pixel / DISPLAY_FB_WIDTH;
  for (uint32_t row = first_row; row <= last_row; row++) {
    uint32_t first = row == first_row
                         ? first_pixel % DISPLAY_FB_WIDTH
                         : 0u;
    uint32_t last = row == last_row
                        ? last_pixel % DISPLAY_FB_WIDTH + 1u
                        : DISPLAY_FB_WIDTH;

    display_mark_accel_rect(first, row, last - first, 1u);
  }
}

static bool display_rect_valid(uint32_t x, uint32_t y, uint32_t width,
                               uint32_t height) {
  return width != 0 && height != 0 && x < DISPLAY_FB_WIDTH &&
         y < DISPLAY_FB_HEIGHT && width <= DISPLAY_FB_WIDTH - x &&
         height <= DISPLAY_FB_HEIGHT - y;
}

static bool display_ppa_fill_rect(uint32_t x, uint32_t y, uint32_t width,
                                  uint32_t height, uint16_t color) {
  esp_err_t err;
  const color_pixel_argb8888_data_t fill_color = {
      .a = 0xff,
      .r = (uint8_t)(((color >> 11) & 0x1fu) * 255u / 31u),
      .g = (uint8_t)(((color >> 5) & 0x3fu) * 255u / 63u),
      .b = (uint8_t)((color & 0x1fu) * 255u / 31u),
  };
#if CONFIG_RV32_HOST_PERF_STATS
  int64_t started;
#endif

  if (!ppa_fill_ready || (uint64_t)width * height < 1024u)
    return false;

  const ppa_fill_oper_config_t operation = {
      .out =
          {
              .buffer = framebuffer,
              .buffer_size = DISPLAY_FB_SIZE,
              .pic_w = DISPLAY_FB_WIDTH,
              .pic_h = DISPLAY_FB_HEIGHT,
              .block_offset_x = x,
              .block_offset_y = y,
              .fill_cm = PPA_FILL_COLOR_MODE_RGB565,
      },
      .fill_block_w = width,
      .fill_block_h = height,
      /* The PPA fill interface accepts component-wise ARGB8888 here and
       * converts it to the selected RGB565 output format.  A packed RGB565
       * value would instead be interpreted as its low B/G bytes. */
      .fill_argb_color = fill_color,
      .mode = PPA_TRANS_MODE_BLOCKING,
  };

  /* ppa_do_fill() performs the cache-line-aligned C2M writeback and
   * invalidation required for its output window before starting DMA. */
  #if CONFIG_RV32_HOST_PERF_STATS
  started = esp_timer_get_time();
  #endif
  err = ppa_do_fill(ppa_fill, &operation);
  #if CONFIG_RV32_HOST_PERF_STATS
  DISPLAY_PERF_ADD(ppa_us, esp_timer_get_time() - started);
  if (err == ESP_OK) {
    DISPLAY_PERF_ADD(ppa_fills, 1u);
    DISPLAY_PERF_ADD(ppa_fill_pixels, (uint64_t)width * height);
  }
  #endif
  if (err == ESP_OK) {
    ppa_fill_touched_framebuffer = true;
    display_mark_present_rect(x, y, width, height);
  }
  return err == ESP_OK;
}

static bool display_fill_rect(const display_pv_command_t *command) {
  uint32_t x = command->args[0];
  uint32_t y = command->args[1];
  uint32_t width = command->args[2];
  uint32_t height = command->args[3];
  uint16_t color = (uint16_t)command->args[4];
  uint32_t rop = command->args[5];

  if (!display_rect_valid(x, y, width, height) ||
      (rop != DISPLAY_PV_ROP_COPY && rop != DISPLAY_PV_ROP_XOR))
    return false;
  if (rop == DISPLAY_PV_ROP_COPY &&
      display_ppa_fill_rect(x, y, width, height, color))
    return true;

  for (uint32_t row = 0; row < height; row++) {
    uint16_t *destination = (uint16_t *)(framebuffer +
        ((size_t)(y + row) * DISPLAY_FB_WIDTH + x) * sizeof(uint16_t));

    if (rop == DISPLAY_PV_ROP_XOR) {
      for (uint32_t column = 0; column < width; column++)
        destination[column] ^= color;
    } else {
      for (uint32_t column = 0; column < width; column++)
        destination[column] = color;
    }
  }
  DISPLAY_PERF_ADD(cpu_fill_pixels, (uint64_t)width * height);
  display_mark_accel_rect(x, y, width, height);
  return true;
}

static bool display_copy_rect(const display_pv_command_t *command) {
  uint32_t source_x = command->args[0];
  uint32_t source_y = command->args[1];
  uint32_t destination_x = command->args[2];
  uint32_t destination_y = command->args[3];
  uint32_t width = command->args[4];
  uint32_t height = command->args[5];

  if (!display_rect_valid(source_x, source_y, width, height) ||
      !display_rect_valid(destination_x, destination_y, width, height))
    return false;

  if (destination_y > source_y) {
    for (uint32_t row = height; row-- > 0;) {
      uint16_t *destination = (uint16_t *)(framebuffer +
          ((size_t)(destination_y + row) * DISPLAY_FB_WIDTH + destination_x) *
              sizeof(uint16_t));
      const uint16_t *source = (const uint16_t *)(framebuffer +
          ((size_t)(source_y + row) * DISPLAY_FB_WIDTH + source_x) *
              sizeof(uint16_t));
      memmove(destination, source, (size_t)width * sizeof(uint16_t));
    }
  } else {
    for (uint32_t row = 0; row < height; row++) {
      uint16_t *destination = (uint16_t *)(framebuffer +
          ((size_t)(destination_y + row) * DISPLAY_FB_WIDTH + destination_x) *
              sizeof(uint16_t));
      const uint16_t *source = (const uint16_t *)(framebuffer +
          ((size_t)(source_y + row) * DISPLAY_FB_WIDTH + source_x) *
              sizeof(uint16_t));
      memmove(destination, source, (size_t)width * sizeof(uint16_t));
    }
  }
  DISPLAY_PERF_ADD(copy_pixels, (uint64_t)width * height);
  display_mark_accel_rect(destination_x, destination_y, width, height);
  return true;
}

static void display_prepare_glyph_lut(uint16_t foreground,
                                      uint16_t background) {
  if (glyph_lut_valid && foreground == glyph_lut_foreground &&
      background == glyph_lut_background)
    return;

  for (uint32_t pattern = 0; pattern < 16; pattern++) {
    for (uint32_t pixel = 0; pixel < 4; pixel++) {
      glyph_nibble_lut[pattern].pixels[pixel] =
          (pattern & (8u >> pixel)) ? foreground : background;
    }
  }
  glyph_lut_foreground = foreground;
  glyph_lut_background = background;
  glyph_lut_valid = true;
}

static void display_expand_mono_row(uint16_t *destination,
                                    const uint8_t *source, uint32_t width) {
  uint32_t full_bytes = width / 8u;

  for (uint32_t column = 0; column < full_bytes; column++) {
    uint8_t bits = source[column];

    memcpy(destination + column * 8u, glyph_nibble_lut[bits >> 4].pixels,
           sizeof(glyph_nibble_lut[0].pixels));
    memcpy(destination + column * 8u + 4u,
           glyph_nibble_lut[bits & 0x0fu].pixels,
           sizeof(glyph_nibble_lut[0].pixels));
  }
  for (uint32_t column = full_bytes * 8u; column < width; column++)
    destination[column] =
        (source[column >> 3] & (0x80u >> (column & 7u)))
            ? glyph_lut_foreground
            : glyph_lut_background;
}

static bool display_tile_description_valid(
    const display_pv_command_t *command, uint32_t payload_length) {
  uint32_t width = command->args[0];
  uint32_t height = command->args[1];
  uint32_t depth = command->args[2];
  uint32_t count = command->args[3];
  uint32_t data_length = command->args[4];
  uint32_t pitch;
  uint32_t bytes_per_tile;

  if (depth != 1 || width == 0 || width > 32u || height == 0 ||
      height > 32u || count == 0 || count > DISPLAY_TILE_MAX_COUNT)
    return false;
  pitch = (width + 7u) / 8u;
  bytes_per_tile = pitch * height;
  if (bytes_per_tile > UINT16_MAX || count > UINT32_MAX / bytes_per_tile ||
      data_length != bytes_per_tile * count || data_length != payload_length ||
      data_length > DISPLAY_TILE_FONT_SIZE)
    return false;

  return true;
}

static bool display_set_tile(const display_pv_command_t *command,
                             const uint8_t *payload,
                             uint32_t payload_length) {
  uint32_t width = command->args[0];
  uint32_t height = command->args[1];
  uint32_t count = command->args[3];
  uint32_t pitch;
  uint32_t bytes_per_tile;

  if (payload == NULL ||
      !display_tile_description_valid(command, payload_length))
    return false;
  pitch = (width + 7u) / 8u;
  bytes_per_tile = pitch * height;

  __atomic_store_n(&tile_font_valid, false, __ATOMIC_RELEASE);
  memcpy(tile_font, payload, payload_length);
  tile_width = width;
  tile_height = height;
  tile_count = count;
  tile_bytes = bytes_per_tile;
  memset(tile_blank, 0, sizeof(tile_blank));
  for (uint32_t index = 0; index < count; index++) {
    const uint8_t *source = tile_font + index * bytes_per_tile;
    bool blank = true;

    for (uint32_t byte = 0; byte < bytes_per_tile; byte++) {
      if (source[byte] != 0) {
        blank = false;
        break;
      }
    }
    tile_blank[index] = blank;
  }
  __atomic_store_n(&tile_font_valid, true, __ATOMIC_RELEASE);
  return true;
}

static bool display_draw_tile(uint32_t pixel_x, uint32_t pixel_y,
                              uint32_t index, uint16_t foreground,
                              uint16_t background) {
  uint32_t pitch;
  const uint8_t *source;

  if (!__atomic_load_n(&tile_font_valid, __ATOMIC_ACQUIRE) ||
      index >= tile_count ||
      !display_rect_valid(pixel_x, pixel_y, tile_width, tile_height))
    return false;
  pitch = (tile_width + 7u) / 8u;
  source = tile_font + index * tile_bytes;
  display_prepare_glyph_lut(foreground, background);
  for (uint32_t row = 0; row < tile_height; row++) {
    uint16_t *destination = (uint16_t *)(framebuffer +
        ((size_t)(pixel_y + row) * DISPLAY_FB_WIDTH + pixel_x) *
            sizeof(uint16_t));

    display_expand_mono_row(destination, source + row * pitch, tile_width);
  }
  DISPLAY_PERF_ADD(tile_pixels, (uint64_t)tile_width * tile_height);
  return true;
}

static bool display_tile_is_blank(uint32_t index) {
  if (!__atomic_load_n(&tile_font_valid, __ATOMIC_ACQUIRE) ||
      index >= tile_count)
    return false;
  return tile_blank[index] != 0;
}

static bool display_tile_fill(const display_pv_command_t *command) {
  uint32_t x = command->args[0];
  uint32_t y = command->args[1];
  uint32_t width = command->args[2];
  uint32_t height = command->args[3];
  uint32_t index = command->args[4];
  uint16_t foreground = (uint16_t)command->args[5];
  uint16_t background = (uint16_t)command->args[6];
  uint32_t rop = command->args[7];
  uint32_t pixel_x;
  uint32_t pixel_y;
  uint32_t pixel_width;
  uint32_t pixel_height;

  if (!__atomic_load_n(&tile_font_valid, __ATOMIC_ACQUIRE) ||
      rop != DISPLAY_PV_ROP_COPY || width == 0 || height == 0 ||
      index >= tile_count ||
      x > UINT32_MAX / tile_width || y > UINT32_MAX / tile_height ||
      width > UINT32_MAX / tile_width || height > UINT32_MAX / tile_height)
    return false;
  pixel_x = x * tile_width;
  pixel_y = y * tile_height;
  pixel_width = width * tile_width;
  pixel_height = height * tile_height;
  if (!display_rect_valid(pixel_x, pixel_y, pixel_width, pixel_height))
    return false;

  if (display_tile_is_blank(index) &&
      display_ppa_fill_rect(pixel_x, pixel_y, pixel_width, pixel_height,
                            background))
    return true;
  for (uint32_t row = 0; row < height; row++) {
    for (uint32_t column = 0; column < width; column++) {
      if (!display_draw_tile(pixel_x + column * tile_width,
                             pixel_y + row * tile_height, index,
                             foreground, background))
        return false;
    }
  }
  display_mark_accel_rect(pixel_x, pixel_y, pixel_width, pixel_height);
  return true;
}

static uint32_t display_tile_index(const uint8_t *payload, uint32_t tile,
                                   uint32_t index_bytes,
                                   uint32_t charmask) {
  if (index_bytes == sizeof(uint16_t)) {
    uint16_t cell;

    memcpy(&cell, payload + tile * sizeof(cell), sizeof(cell));
    return cell & charmask;
  }

  uint32_t index;

  memcpy(&index, payload + tile * sizeof(index), sizeof(index));
  return index;
}

static bool display_tile_blit_indices(const display_pv_command_t *command,
                                      const uint8_t *payload,
                                      uint32_t payload_length,
                                      uint32_t index_bytes,
                                      uint32_t charmask) {
  uint32_t x = command->args[0];
  uint32_t y = command->args[1];
  uint32_t width = command->args[2];
  uint32_t height = command->args[3];
  uint16_t foreground = (uint16_t)command->args[4];
  uint16_t background = (uint16_t)command->args[5];
  uint32_t count = command->args[6];
  uint32_t pixel_x;
  uint32_t pixel_y;
  uint32_t pixel_width;
  uint32_t pixel_height;
  uint32_t blank_count = 0;
  uint32_t first_index = 0;
  bool uniform = true;
  bool background_prefilled = false;

  if (!__atomic_load_n(&tile_font_valid, __ATOMIC_ACQUIRE) ||
      payload == NULL || width == 0 || height == 0 ||
      width > UINT32_MAX / height || count != width * height ||
      (index_bytes != sizeof(uint16_t) && index_bytes != sizeof(uint32_t)) ||
      count > UINT32_MAX / index_bytes ||
      payload_length != count * index_bytes ||
      x > UINT32_MAX / tile_width || y > UINT32_MAX / tile_height ||
      width > UINT32_MAX / tile_width || height > UINT32_MAX / tile_height)
    return false;
  pixel_x = x * tile_width;
  pixel_y = y * tile_height;
  pixel_width = width * tile_width;
  pixel_height = height * tile_height;
  if (!display_rect_valid(pixel_x, pixel_y, pixel_width, pixel_height))
    return false;

  for (uint32_t tile = 0; tile < count; tile++) {
    uint32_t index =
        display_tile_index(payload, tile, index_bytes, charmask);

    if (index >= tile_count)
      return false;
    if (tile == 0)
      first_index = index;
    else if (index != first_index)
      uniform = false;
    if (display_tile_is_blank(index))
      blank_count++;
  }
  if (uniform) {
    display_pv_command_t fill = {
        .args = {x, y, width, height, first_index, foreground, background,
                 DISPLAY_PV_ROP_COPY},
    };

    return display_tile_fill(&fill);
  }
  /* Large text updates are usually mostly spaces. Fill their background once
   * with PPA, then expand only visible glyphs on CPU1. This preserves the
   * standard tileblit contract while avoiding thousands of identical stores. */
  if (blank_count != 0 && blank_count * 2u >= count)
    background_prefilled = display_ppa_fill_rect(
        pixel_x, pixel_y, pixel_width, pixel_height, background);
  for (uint32_t row = 0, tile = 0; row < height; row++) {
    uint32_t destination_y = pixel_y + row * tile_height;

    for (uint32_t column = 0; column < width; column++, tile++) {
      uint32_t index =
          display_tile_index(payload, tile, index_bytes, charmask);
      uint32_t destination_x = pixel_x + column * tile_width;

      if (background_prefilled && display_tile_is_blank(index))
        continue;
      if (!display_draw_tile(destination_x, destination_y, index,
                             foreground, background))
        return false;
      if (background_prefilled)
        display_mark_accel_rect(destination_x, destination_y,
                                tile_width, tile_height);
    }
  }
  if (!background_prefilled)
    display_mark_accel_rect(pixel_x, pixel_y, pixel_width, pixel_height);
  return true;
}

static bool display_tile_blit(const display_pv_command_t *command,
                              const uint8_t *payload,
                              uint32_t payload_length) {
  return display_tile_blit_indices(command, payload, payload_length,
                                   sizeof(uint32_t), UINT32_MAX);
}

static bool display_text_run16(const display_pv_command_t *command,
                               const uint8_t *payload,
                               uint32_t payload_length) {
  uint32_t charmask = command->args[7];

  if (charmask != 0xffu && charmask != 0x1ffu)
    return false;
  return display_tile_blit_indices(command, payload, payload_length,
                                   sizeof(uint16_t), charmask);
}

static void display_toggle_tile_cursor(void) {
  uint32_t start_row;
  uint32_t pixel_x;
  uint32_t pixel_y;
  uint32_t cursor_height;
#if CONFIG_RV32_HOST_PERF_STATS
  uint32_t started;
#endif

  if (!tile_cursor_active || tile_cursor_shape == 0 || tile_height == 0)
    return;
  switch (tile_cursor_shape) {
  case 1:
    start_row = tile_height > 2u ? tile_height - 2u : tile_height - 1u;
    break;
  case 2:
    start_row = tile_height * 2u / 3u;
    break;
  case 3:
    start_row = tile_height / 2u;
    break;
  case 4:
    start_row = tile_height / 3u;
    break;
  default:
    start_row = 0;
    break;
  }
  pixel_x = tile_cursor_x * tile_width;
  pixel_y = tile_cursor_y * tile_height + start_row;
  cursor_height = tile_height - start_row;
  if (!display_rect_valid(pixel_x, pixel_y, tile_width, cursor_height))
    return;
#if CONFIG_RV32_HOST_PERF_STATS
  started = esp_cpu_get_cycle_count();
#endif
  for (uint32_t row = 0; row < cursor_height; row++) {
    uint16_t *destination = (uint16_t *)(framebuffer +
        ((size_t)(pixel_y + row) * DISPLAY_FB_WIDTH + pixel_x) *
            sizeof(uint16_t));

    for (uint32_t column = 0; column < tile_width; column++)
      destination[column] ^= tile_cursor_color;
  }
  display_mark_accel_rect(pixel_x, pixel_y, tile_width, cursor_height);
  tile_cursor_drawn = !tile_cursor_drawn;
#if CONFIG_RV32_HOST_PERF_STATS
  uint32_t elapsed = esp_cpu_get_cycle_count() - started;

  DISPLAY_PERF_ADD(cursor_toggles, 1u);
  DISPLAY_PERF_ADD(cursor_toggle_cycles, elapsed);
  display_perf_update_max(&display_perf.cursor_toggle_max_cycles, elapsed);
#endif
}

static bool display_set_tile_cursor(const display_pv_command_t *command) {
  uint32_t x = command->args[0];
  uint32_t y = command->args[1];
  uint32_t mode = command->args[2];
  uint32_t shape = command->args[3];
  uint16_t foreground = (uint16_t)command->args[4];
  uint16_t background = (uint16_t)command->args[5];

  /* tty0/fbcon continues to blink its pvfb cursor even after the interactive
   * shell has moved to hvc0.  Both paths target the same scanout, but hvc0
   * owns the visible terminal once it has produced output.  A late fbcon
   * cursor command must therefore be consumed without replacing the HVC
   * cursor coordinates. */
  if (terminal.active)
    return true;
  if (!__atomic_load_n(&tile_font_valid, __ATOMIC_ACQUIRE) || mode > 1u ||
      shape > 5u || x > UINT32_MAX / tile_width ||
      y > UINT32_MAX / tile_height ||
      x * tile_width >= DISPLAY_FB_WIDTH ||
      y * tile_height >= DISPLAY_FB_HEIGHT)
    return false;
  tile_cursor_active = mode != 0 && shape != 0;
  tile_cursor_x = x;
  tile_cursor_y = y;
  tile_cursor_shape = shape;
  tile_cursor_color = foreground ^ background;
  if (tile_cursor_color == 0)
    tile_cursor_color = UINT16_MAX;
  return true;
}

static uint16_t display_terminal_rgb565(uint8_t red, uint8_t green,
                                        uint8_t blue) {
  return (uint16_t)(((uint16_t)(red >> 3) << 11) |
                    ((uint16_t)(green >> 2) << 5) | (blue >> 3));
}

static uint16_t display_terminal_palette(uint32_t index) {
  static const uint8_t ansi[16][3] = {
      {0x00, 0x00, 0x00}, {0xaa, 0x00, 0x00}, {0x00, 0xaa, 0x00},
      {0xaa, 0x55, 0x00}, {0x00, 0x00, 0xaa}, {0xaa, 0x00, 0xaa},
      {0x00, 0xaa, 0xaa}, {0xaa, 0xaa, 0xaa}, {0x55, 0x55, 0x55},
      {0xff, 0x55, 0x55}, {0x55, 0xff, 0x55}, {0xff, 0xff, 0x55},
      {0x55, 0x55, 0xff}, {0xff, 0x55, 0xff}, {0x55, 0xff, 0xff},
      {0xff, 0xff, 0xff},
  };

  if (index < 16u)
    return display_terminal_rgb565(ansi[index][0], ansi[index][1],
                                   ansi[index][2]);
  if (index < 232u) {
    static const uint8_t cube[6] = {0, 95, 135, 175, 215, 255};
    uint32_t value = index - 16u;

    return display_terminal_rgb565(cube[value / 36u],
                                   cube[(value / 6u) % 6u],
                                   cube[value % 6u]);
  }
  if (index < 256u) {
    uint8_t level = (uint8_t)(8u + (index - 232u) * 10u);

    return display_terminal_rgb565(level, level, level);
  }
  return terminal.default_foreground;
}

static inline display_terminal_cell_t *display_terminal_cell(uint32_t column,
                                                              uint32_t row) {
  uint32_t physical_row = terminal.row_origin + row;

  if (physical_row >= DISPLAY_TERMINAL_ROWS)
    physical_row -= DISPLAY_TERMINAL_ROWS;
  return &terminal.cells[physical_row * DISPLAY_TERMINAL_COLUMNS + column];
}

static uint16_t display_terminal_foreground(void) {
  return terminal.inverse ? terminal.background : terminal.foreground;
}

static uint16_t display_terminal_background(void) {
  return terminal.inverse ? terminal.foreground : terminal.background;
}

static void display_terminal_mark_rows(uint32_t first, uint32_t last) {
  if (terminal.cells == NULL || first >= DISPLAY_TERMINAL_ROWS)
    return;
  if (last >= DISPLAY_TERMINAL_ROWS)
    last = DISPLAY_TERMINAL_ROWS - 1u;
  for (uint32_t row = first; row <= last; row++) {
    for (uint32_t column = 0; column < DISPLAY_TERMINAL_COLUMNS; column++)
      display_terminal_cell(column, row)->dirty = 1u;
  }
  terminal.pixels_valid = false;
}

static void display_terminal_hide_cursor(void) {
  if (tile_cursor_drawn)
    display_toggle_tile_cursor();
}

static void display_terminal_release_framebuffer(void) {
  bool released_terminal = terminal_owns_framebuffer;

  /* A raw writer may already have replaced the pixels below the XOR cursor.
   * Do not XOR them a second time: forget the overlay and let the raw frame
   * replace it.  The next terminal byte forces a complete clean redraw. */
  tile_cursor_active = false;
  tile_cursor_drawn = false;
  terminal.pixels_valid = false;
  terminal_owns_framebuffer = false;
  if (released_terminal)
    printf("Display diagnostic: terminal release at %u ms\n",
           (unsigned int)(esp_timer_get_time() / 1000));
}

static void display_terminal_apply_cursor(void) {
  if (!terminal_owns_framebuffer || !terminal.active ||
      !terminal.cursor_visible ||
      !terminal.pixels_valid ||
      !__atomic_load_n(&tile_font_valid, __ATOMIC_ACQUIRE)) {
    tile_cursor_active = false;
    return;
  }

  tile_cursor_active = true;
  tile_cursor_x = terminal.column;
  tile_cursor_y = terminal.row;
  tile_cursor_shape = 1u;
  tile_cursor_color = display_terminal_foreground() ^
                      display_terminal_background();
  if (tile_cursor_color == 0)
    tile_cursor_color = UINT16_MAX;
  if (!tile_cursor_drawn)
    display_toggle_tile_cursor();
  terminal_cursor_vsyncs = 0;
}

static bool display_terminal_flush_dirty(bool show_cursor) {
  uint16_t glyphs[DISPLAY_TERMINAL_COLUMNS];
  bool complete = true;

  if (terminal.cells == NULL ||
      !__atomic_load_n(&tile_font_valid, __ATOMIC_ACQUIRE) ||
      tile_width != DISPLAY_TERMINAL_TILE_WIDTH ||
      tile_height != DISPLAY_TERMINAL_TILE_HEIGHT)
    return false;

  display_terminal_hide_cursor();
  for (uint32_t row = 0; row < DISPLAY_TERMINAL_ROWS; row++) {
    for (uint32_t column = 0; column < DISPLAY_TERMINAL_COLUMNS;) {
      display_terminal_cell_t *first = display_terminal_cell(column, row);
      uint32_t start;
      uint32_t count;

      if (!first->dirty) {
        column++;
        continue;
      }
      start = column;
      count = 0;
      while (column < DISPLAY_TERMINAL_COLUMNS) {
        display_terminal_cell_t *cell = display_terminal_cell(column, row);

        if (!cell->dirty || cell->foreground != first->foreground ||
            cell->background != first->background)
          break;
        glyphs[count++] = cell->glyph;
        column++;
      }

      display_pv_command_t command = {
          .args = {start, row, count, 1u, first->foreground,
                   first->background, count, 0xffu},
      };
      if (!display_tile_blit_indices(&command, (const uint8_t *)glyphs,
                                     count * sizeof(glyphs[0]),
                                     sizeof(glyphs[0]), 0xffu)) {
        complete = false;
        continue;
      }
      for (uint32_t offset = 0; offset < count; offset++)
        display_terminal_cell(start + offset, row)->dirty = 0u;
    }
  }
  terminal.pixels_valid = complete;
  if (show_cursor)
    display_terminal_apply_cursor();
  return complete;
}

static void display_terminal_set_blank(display_terminal_cell_t *cell) {
  cell->glyph = ' ';
  cell->foreground = display_terminal_foreground();
  cell->background = display_terminal_background();
  cell->dirty = 1u;
}

static bool display_terminal_fill_pixels(uint32_t first_row,
                                         uint32_t row_count,
                                         uint16_t color) {
  display_pv_command_t fill = {
      .args = {0u, first_row * DISPLAY_TERMINAL_TILE_HEIGHT,
               DISPLAY_TERMINAL_COLUMNS * DISPLAY_TERMINAL_TILE_WIDTH,
               row_count * DISPLAY_TERMINAL_TILE_HEIGHT, color,
               DISPLAY_PV_ROP_COPY},
  };

  return row_count != 0 && display_fill_rect(&fill);
}

static void display_terminal_clear_rows(uint32_t first, uint32_t count) {
  if (count == 0 || first >= DISPLAY_TERMINAL_ROWS)
    return;
  if (count > DISPLAY_TERMINAL_ROWS - first)
    count = DISPLAY_TERMINAL_ROWS - first;

  for (uint32_t row = first; row < first + count; row++) {
    for (uint32_t column = 0; column < DISPLAY_TERMINAL_COLUMNS; column++)
      display_terminal_set_blank(display_terminal_cell(column, row));
  }
  if (__atomic_load_n(&tile_font_valid, __ATOMIC_ACQUIRE) &&
      display_terminal_fill_pixels(first, count,
                                   display_terminal_background())) {
    for (uint32_t row = first; row < first + count; row++) {
      for (uint32_t column = 0; column < DISPLAY_TERMINAL_COLUMNS; column++)
        display_terminal_cell(column, row)->dirty = 0u;
    }
  } else {
    terminal.pixels_valid = false;
  }
}

static void display_terminal_scroll_up(uint32_t lines) {
  uint32_t top = terminal.scroll_top;
  uint32_t bottom = terminal.scroll_bottom;
  uint32_t height = bottom - top + 1u;

  if (lines == 0)
    lines = 1;
  if (lines > height)
    lines = height;
  display_terminal_hide_cursor();

  if (lines < height) {
    if (top == 0u && bottom + 1u == DISPLAY_TERMINAL_ROWS) {
      terminal.row_origin += lines;
      if (terminal.row_origin >= DISPLAY_TERMINAL_ROWS)
        terminal.row_origin -= DISPLAY_TERMINAL_ROWS;
    } else {
      const size_t row_size = DISPLAY_TERMINAL_COLUMNS *
                              sizeof(display_terminal_cell_t);

      for (uint32_t row = top; row + lines <= bottom; row++)
        memcpy(display_terminal_cell(0, row),
               display_terminal_cell(0, row + lines), row_size);
    }
  }

  for (uint32_t row = bottom + 1u - lines; row <= bottom; row++) {
    for (uint32_t column = 0; column < DISPLAY_TERMINAL_COLUMNS; column++)
      display_terminal_set_blank(display_terminal_cell(column, row));
  }

  /* Do not move a 1 MiB scanout once per newline.  ANSI input arrives in
   * multi-kilobyte virtio records, so update only the compact cell model
   * while parsing and draw the final visible state once at the end of the
   * service slice. */
  display_terminal_mark_rows(top, bottom);
  terminal.pixels_valid = false;
}

static void display_terminal_scroll_down(uint32_t lines) {
  uint32_t top = terminal.scroll_top;
  uint32_t bottom = terminal.scroll_bottom;
  uint32_t height = bottom - top + 1u;

  if (lines == 0)
    lines = 1;
  if (lines > height)
    lines = height;
  display_terminal_hide_cursor();

  if (lines < height) {
    if (top == 0u && bottom + 1u == DISPLAY_TERMINAL_ROWS) {
      terminal.row_origin += DISPLAY_TERMINAL_ROWS - lines;
      if (terminal.row_origin >= DISPLAY_TERMINAL_ROWS)
        terminal.row_origin -= DISPLAY_TERMINAL_ROWS;
    } else {
      const size_t row_size = DISPLAY_TERMINAL_COLUMNS *
                              sizeof(display_terminal_cell_t);
      uint32_t row = bottom + 1u;

      while (row-- > top + lines)
        memcpy(display_terminal_cell(0, row),
               display_terminal_cell(0, row - lines), row_size);
    }
  }

  for (uint32_t row = top; row < top + lines; row++) {
    for (uint32_t column = 0; column < DISPLAY_TERMINAL_COLUMNS; column++)
      display_terminal_set_blank(display_terminal_cell(column, row));
  }

  display_terminal_mark_rows(top, bottom);
  terminal.pixels_valid = false;
}

static void display_terminal_linefeed(void) {
  terminal.wrap_pending = false;
  if (terminal.row == terminal.scroll_bottom)
    display_terminal_scroll_up(1u);
  else if (terminal.row + 1u < DISPLAY_TERMINAL_ROWS)
    terminal.row++;
}

static void display_terminal_put_glyph(uint8_t glyph) {
  display_terminal_cell_t *cell;
  uint16_t foreground;
  uint16_t background;

  if (terminal.wrap_pending) {
    terminal.column = 0;
    display_terminal_linefeed();
  }
  foreground = display_terminal_foreground();
  background = display_terminal_background();
  cell = display_terminal_cell(terminal.column, terminal.row);
  if (cell->glyph != glyph || cell->foreground != foreground ||
      cell->background != background) {
    cell->glyph = glyph;
    cell->foreground = foreground;
    cell->background = background;
    cell->dirty = 1u;
  }
  if (terminal.column + 1u == DISPLAY_TERMINAL_COLUMNS)
    terminal.wrap_pending = true;
  else
    terminal.column++;
}

static uint8_t display_terminal_unicode_glyph(uint32_t codepoint) {
  switch (codepoint) {
  case 0x00a0: return ' ';
  case 0x00b0: return 0xf8;
  case 0x2022: return 0x07;
  case 0x2500: return 0xc4;
  case 0x2502: return 0xb3;
  case 0x250c: return 0xda;
  case 0x2510: return 0xbf;
  case 0x2514: return 0xc0;
  case 0x2518: return 0xd9;
  case 0x251c: return 0xc3;
  case 0x2524: return 0xb4;
  case 0x252c: return 0xc2;
  case 0x2534: return 0xc1;
  case 0x253c: return 0xc5;
  case 0x2580: return 0xdf;
  case 0x2584: return 0xdc;
  case 0x2588: return 0xdb;
  case 0x2591: return 0xb0;
  case 0x2592: return 0xb1;
  case 0x2593: return 0xb2;
  default: return codepoint < 0x80u ? (uint8_t)codepoint : '?';
  }
}

static uint32_t display_terminal_parameter(uint32_t index,
                                           uint32_t default_value) {
  if (index >= terminal.parameter_count || terminal.parameters[index] == 0)
    return default_value;
  return terminal.parameters[index];
}

static void display_terminal_erase_cells(uint32_t row, uint32_t first,
                                         uint32_t last) {
  if (row >= DISPLAY_TERMINAL_ROWS || first >= DISPLAY_TERMINAL_COLUMNS)
    return;
  if (last >= DISPLAY_TERMINAL_COLUMNS)
    last = DISPLAY_TERMINAL_COLUMNS - 1u;
  for (uint32_t column = first; column <= last; column++)
    display_terminal_set_blank(display_terminal_cell(column, row));
}

static void display_terminal_sgr(void) {
  uint32_t count = terminal.parameter_count;

  if (!terminal.parameter_present)
    count = 1u;
  for (uint32_t index = 0; index < count; index++) {
    uint32_t parameter = terminal.parameters[index];

    if (parameter == 0) {
      terminal.foreground = terminal.default_foreground;
      terminal.background = terminal.default_background;
      terminal.bold = false;
      terminal.inverse = false;
    } else if (parameter == 1) {
      terminal.bold = true;
    } else if (parameter == 22) {
      terminal.bold = false;
    } else if (parameter == 7) {
      terminal.inverse = true;
    } else if (parameter == 27) {
      terminal.inverse = false;
    } else if (parameter >= 30 && parameter <= 37) {
      terminal.foreground = display_terminal_palette(
          parameter - 30u + (terminal.bold ? 8u : 0u));
    } else if (parameter == 39) {
      terminal.foreground = terminal.default_foreground;
    } else if (parameter >= 40 && parameter <= 47) {
      terminal.background = display_terminal_palette(parameter - 40u);
    } else if (parameter == 49) {
      terminal.background = terminal.default_background;
    } else if (parameter >= 90 && parameter <= 97) {
      terminal.foreground = display_terminal_palette(parameter - 90u + 8u);
    } else if (parameter >= 100 && parameter <= 107) {
      terminal.background = display_terminal_palette(parameter - 100u + 8u);
    } else if ((parameter == 38u || parameter == 48u) && index + 1u < count) {
      uint16_t color;
      bool valid = false;

      if (terminal.parameters[index + 1u] == 5u && index + 2u < count) {
        color = display_terminal_palette(terminal.parameters[index + 2u]);
        index += 2u;
        valid = true;
      } else if (terminal.parameters[index + 1u] == 2u &&
                 index + 4u < count) {
        color = display_terminal_rgb565(
            (uint8_t)terminal.parameters[index + 2u],
            (uint8_t)terminal.parameters[index + 3u],
            (uint8_t)terminal.parameters[index + 4u]);
        index += 4u;
        valid = true;
      }
      if (valid) {
        if (parameter == 38u)
          terminal.foreground = color;
        else
          terminal.background = color;
      }
    }
  }
}

static void display_terminal_csi(uint8_t final) {
  uint32_t first = display_terminal_parameter(0, 1u);
  uint32_t second = display_terminal_parameter(1, 1u);

  terminal.wrap_pending = false;
  switch (final) {
  case 'A':
    terminal.row = first > terminal.row ? 0u : terminal.row - first;
    break;
  case 'B':
    terminal.row = terminal.row + first >= DISPLAY_TERMINAL_ROWS
                       ? DISPLAY_TERMINAL_ROWS - 1u
                       : terminal.row + first;
    break;
  case 'C':
    terminal.column = terminal.column + first >= DISPLAY_TERMINAL_COLUMNS
                          ? DISPLAY_TERMINAL_COLUMNS - 1u
                          : terminal.column + first;
    break;
  case 'D':
    terminal.column = first > terminal.column ? 0u : terminal.column - first;
    break;
  case 'E':
    terminal.column = 0;
    terminal.row = terminal.row + first >= DISPLAY_TERMINAL_ROWS
                       ? DISPLAY_TERMINAL_ROWS - 1u
                       : terminal.row + first;
    break;
  case 'F':
    terminal.column = 0;
    terminal.row = first > terminal.row ? 0u : terminal.row - first;
    break;
  case 'G':
  case '`':
    terminal.column = first > DISPLAY_TERMINAL_COLUMNS
                          ? DISPLAY_TERMINAL_COLUMNS - 1u
                          : first - 1u;
    break;
  case 'H':
  case 'f':
    terminal.row = first > DISPLAY_TERMINAL_ROWS
                       ? DISPLAY_TERMINAL_ROWS - 1u
                       : first - 1u;
    terminal.column = second > DISPLAY_TERMINAL_COLUMNS
                          ? DISPLAY_TERMINAL_COLUMNS - 1u
                          : second - 1u;
    break;
  case 'd':
    terminal.row = first > DISPLAY_TERMINAL_ROWS
                       ? DISPLAY_TERMINAL_ROWS - 1u
                       : first - 1u;
    break;
  case 'J': {
    uint32_t mode = terminal.parameters[0];

    if (mode == 2u || mode == 3u) {
      display_terminal_hide_cursor();
      display_terminal_clear_rows(0, DISPLAY_TERMINAL_ROWS);
    } else if (mode == 1u) {
      for (uint32_t row = 0; row < terminal.row; row++)
        display_terminal_erase_cells(row, 0, DISPLAY_TERMINAL_COLUMNS - 1u);
      display_terminal_erase_cells(terminal.row, 0, terminal.column);
    } else {
      display_terminal_erase_cells(terminal.row, terminal.column,
                                   DISPLAY_TERMINAL_COLUMNS - 1u);
      for (uint32_t row = terminal.row + 1u;
           row < DISPLAY_TERMINAL_ROWS; row++)
        display_terminal_erase_cells(row, 0, DISPLAY_TERMINAL_COLUMNS - 1u);
    }
    break;
  }
  case 'K':
    if (terminal.parameters[0] == 1u)
      display_terminal_erase_cells(terminal.row, 0, terminal.column);
    else if (terminal.parameters[0] == 2u)
      display_terminal_erase_cells(terminal.row, 0,
                                   DISPLAY_TERMINAL_COLUMNS - 1u);
    else
      display_terminal_erase_cells(terminal.row, terminal.column,
                                   DISPLAY_TERMINAL_COLUMNS - 1u);
    break;
  case 'm': display_terminal_sgr(); break;
  case 's':
    terminal.saved_column = terminal.column;
    terminal.saved_row = terminal.row;
    break;
  case 'u':
    terminal.column = terminal.saved_column;
    terminal.row = terminal.saved_row;
    break;
  case '@': {
    uint32_t count = first;
    if (count > DISPLAY_TERMINAL_COLUMNS - terminal.column)
      count = DISPLAY_TERMINAL_COLUMNS - terminal.column;
    memmove(display_terminal_cell(terminal.column + count, terminal.row),
            display_terminal_cell(terminal.column, terminal.row),
            (DISPLAY_TERMINAL_COLUMNS - terminal.column - count) *
                sizeof(display_terminal_cell_t));
    display_terminal_erase_cells(terminal.row, terminal.column,
                                 terminal.column + count - 1u);
    for (uint32_t column = terminal.column;
         column < DISPLAY_TERMINAL_COLUMNS; column++)
      display_terminal_cell(column, terminal.row)->dirty = 1u;
    break;
  }
  case 'P': {
    uint32_t count = first;
    if (count > DISPLAY_TERMINAL_COLUMNS - terminal.column)
      count = DISPLAY_TERMINAL_COLUMNS - terminal.column;
    memmove(display_terminal_cell(terminal.column, terminal.row),
            display_terminal_cell(terminal.column + count, terminal.row),
            (DISPLAY_TERMINAL_COLUMNS - terminal.column - count) *
                sizeof(display_terminal_cell_t));
    display_terminal_erase_cells(
        terminal.row, DISPLAY_TERMINAL_COLUMNS - count,
        DISPLAY_TERMINAL_COLUMNS - 1u);
    for (uint32_t column = terminal.column;
         column < DISPLAY_TERMINAL_COLUMNS; column++)
      display_terminal_cell(column, terminal.row)->dirty = 1u;
    break;
  }
  case 'X':
    display_terminal_erase_cells(
        terminal.row, terminal.column,
        terminal.column + first - 1u < DISPLAY_TERMINAL_COLUMNS
            ? terminal.column + first - 1u
            : DISPLAY_TERMINAL_COLUMNS - 1u);
    break;
  case 'L':
    if (terminal.row >= terminal.scroll_top &&
        terminal.row <= terminal.scroll_bottom) {
      uint16_t saved_top = terminal.scroll_top;
      terminal.scroll_top = terminal.row;
      display_terminal_scroll_down(first);
      terminal.scroll_top = saved_top;
    }
    break;
  case 'M':
    if (terminal.row >= terminal.scroll_top &&
        terminal.row <= terminal.scroll_bottom) {
      uint16_t saved_top = terminal.scroll_top;
      terminal.scroll_top = terminal.row;
      display_terminal_scroll_up(first);
      terminal.scroll_top = saved_top;
    }
    break;
  case 'S': display_terminal_scroll_up(first); break;
  case 'T': display_terminal_scroll_down(first); break;
  case 'r':
    if (first < second && second <= DISPLAY_TERMINAL_ROWS) {
      terminal.scroll_top = first - 1u;
      terminal.scroll_bottom = second - 1u;
      terminal.column = 0;
      terminal.row = 0;
    } else if (!terminal.parameter_present) {
      terminal.scroll_top = 0;
      terminal.scroll_bottom = DISPLAY_TERMINAL_ROWS - 1u;
    }
    break;
  case 'h':
  case 'l':
    if (terminal.private_csi && terminal.parameters[0] == 25u)
      terminal.cursor_visible = final == 'h';
    else if (terminal.private_csi && terminal.parameters[0] == 1049u &&
             final == 'h') {
      terminal.saved_column = terminal.column;
      terminal.saved_row = terminal.row;
      terminal.column = terminal.row = 0;
      display_terminal_clear_rows(0, DISPLAY_TERMINAL_ROWS);
    } else if (terminal.private_csi && terminal.parameters[0] == 1049u) {
      terminal.column = terminal.saved_column;
      terminal.row = terminal.saved_row;
      display_terminal_clear_rows(0, DISPLAY_TERMINAL_ROWS);
    }
    break;
  default: break;
  }
}

static void display_terminal_reset_state(bool clear_screen) {
  terminal.column = 0;
  terminal.row = 0;
  terminal.saved_column = 0;
  terminal.saved_row = 0;
  terminal.scroll_top = 0;
  terminal.scroll_bottom = DISPLAY_TERMINAL_ROWS - 1u;
  terminal.row_origin = 0;
  terminal.default_foreground = display_terminal_palette(7u);
  terminal.default_background = display_terminal_palette(0u);
  terminal.foreground = terminal.default_foreground;
  terminal.background = terminal.default_background;
  terminal.bold = false;
  terminal.inverse = false;
  terminal.cursor_visible = true;
  terminal.wrap_pending = false;
  terminal.parse_state = DISPLAY_TERMINAL_GROUND;
  terminal.utf8_remaining = 0;
  if (clear_screen && terminal.cells != NULL)
    display_terminal_clear_rows(0, DISPLAY_TERMINAL_ROWS);
}

static void display_terminal_ground(uint8_t byte) {
  if (terminal.utf8_remaining != 0u) {
    if ((byte & 0xc0u) != 0x80u) {
      terminal.utf8_remaining = 0;
      display_terminal_put_glyph('?');
      display_terminal_ground(byte);
      return;
    }
    terminal.utf8_codepoint = (terminal.utf8_codepoint << 6) | (byte & 0x3fu);
    if (--terminal.utf8_remaining == 0u)
      display_terminal_put_glyph(
          display_terminal_unicode_glyph(terminal.utf8_codepoint));
    return;
  }
  if (byte >= 0xc2u && byte <= 0xdfu) {
    terminal.utf8_codepoint = byte & 0x1fu;
    terminal.utf8_remaining = 1u;
    return;
  }
  if (byte >= 0xe0u && byte <= 0xefu) {
    terminal.utf8_codepoint = byte & 0x0fu;
    terminal.utf8_remaining = 2u;
    return;
  }
  if (byte >= 0xf0u && byte <= 0xf4u) {
    terminal.utf8_codepoint = byte & 0x07u;
    terminal.utf8_remaining = 3u;
    return;
  }

  switch (byte) {
  case 0x07: break;
  case 0x08:
    terminal.wrap_pending = false;
    if (terminal.column != 0)
      terminal.column--;
    break;
  case 0x09:
    terminal.wrap_pending = false;
    terminal.column = (terminal.column + 8u) & ~7u;
    if (terminal.column >= DISPLAY_TERMINAL_COLUMNS)
      terminal.column = DISPLAY_TERMINAL_COLUMNS - 1u;
    break;
  case 0x0a:
    /* hvc0 disables guest OPOST/ONLCR so Linux can submit large records.
     * Perform newline-mode carriage return here on CPU1 instead. */
    terminal.column = 0;
    display_terminal_linefeed();
    break;
  case 0x0b:
  case 0x0c: display_terminal_linefeed(); break;
  case 0x0d:
    terminal.column = 0;
    terminal.wrap_pending = false;
    break;
  case 0x1b:
    terminal.parse_state = DISPLAY_TERMINAL_ESCAPE;
    break;
  default:
    if (byte >= 0x20u && byte != 0x7fu)
      display_terminal_put_glyph(byte < 0x80u ? byte : '?');
    break;
  }
}

static void display_terminal_parse_byte(uint8_t byte) {
  switch (terminal.parse_state) {
  case DISPLAY_TERMINAL_GROUND:
    display_terminal_ground(byte);
    break;
  case DISPLAY_TERMINAL_ESCAPE:
    terminal.parse_state = DISPLAY_TERMINAL_GROUND;
    if (byte == '[') {
      memset(terminal.parameters, 0, sizeof(terminal.parameters));
      terminal.parameter_count = 1u;
      terminal.parameter_present = false;
      terminal.private_csi = false;
      terminal.parse_state = DISPLAY_TERMINAL_CSI;
    } else if (byte == ']') {
      terminal.parse_state = DISPLAY_TERMINAL_OSC;
    } else if (byte == '7') {
      terminal.saved_column = terminal.column;
      terminal.saved_row = terminal.row;
    } else if (byte == '8') {
      terminal.column = terminal.saved_column;
      terminal.row = terminal.saved_row;
    } else if (byte == 'D') {
      display_terminal_linefeed();
    } else if (byte == 'E') {
      terminal.column = 0;
      display_terminal_linefeed();
    } else if (byte == 'M') {
      if (terminal.row == terminal.scroll_top)
        display_terminal_scroll_down(1u);
      else if (terminal.row != 0)
        terminal.row--;
    } else if (byte == 'c') {
      display_terminal_reset_state(true);
    }
    break;
  case DISPLAY_TERMINAL_CSI:
    if (byte == '?' && !terminal.parameter_present) {
      terminal.private_csi = true;
    } else if (byte >= '0' && byte <= '9') {
      uint16_t *parameter =
          &terminal.parameters[terminal.parameter_count - 1u];
      uint32_t value = (uint32_t)*parameter * 10u + (byte - '0');

      *parameter = value > UINT16_MAX ? UINT16_MAX : (uint16_t)value;
      terminal.parameter_present = true;
    } else if (byte == ';') {
      if (terminal.parameter_count < DISPLAY_TERMINAL_PARAM_COUNT)
        terminal.parameter_count++;
      terminal.parameter_present = true;
    } else if (byte >= 0x40u && byte <= 0x7eu) {
      display_terminal_csi(byte);
      terminal.parse_state = DISPLAY_TERMINAL_GROUND;
    } else if (byte == 0x1bu) {
      terminal.parse_state = DISPLAY_TERMINAL_ESCAPE;
    }
    break;
  case DISPLAY_TERMINAL_OSC:
    if (byte == 0x07u)
      terminal.parse_state = DISPLAY_TERMINAL_GROUND;
    else if (byte == 0x1bu)
      terminal.parse_state = DISPLAY_TERMINAL_OSC_ESCAPE;
    break;
  case DISPLAY_TERMINAL_OSC_ESCAPE:
    terminal.parse_state = byte == '\\' ? DISPLAY_TERMINAL_GROUND
                                          : DISPLAY_TERMINAL_OSC;
    break;
  }
}

static bool display_terminal_process_stream(void) {
  uint8_t buffer[DISPLAY_TERMINAL_READ_CHUNK];
  size_t processed = 0;
  bool claimed_terminal = false;

  if (terminal_stream == NULL || terminal.cells == NULL)
    return false;
  if (!terminal.active || !terminal_owns_framebuffer) {
    claimed_terminal = true;
    terminal.active = true;
    terminal_owns_framebuffer = true;
    terminal.pixels_valid = false;
    display_terminal_mark_rows(0, DISPLAY_TERMINAL_ROWS - 1u);
  }
  display_terminal_hide_cursor();
  while (processed < DISPLAY_TERMINAL_SERVICE_BYTES) {
    size_t limit = DISPLAY_TERMINAL_SERVICE_BYTES - processed;
    size_t length;

    if (limit > sizeof(buffer))
      limit = sizeof(buffer);
    length = xStreamBufferReceive(terminal_stream, buffer, limit, 0);
    if (length == 0)
      break;
    for (size_t index = 0; index < length; index++)
      display_terminal_parse_byte(buffer[index]);
    processed += length;
  }
  (void)display_terminal_flush_dirty(true);
  if (ppa_fill_touched_framebuffer && display_flush_accel_dirty())
    ppa_fill_touched_framebuffer = false;
  /* Event-only instrumentation preserves the existing ownership decisions.
   * A zero-byte takeover distinguishes the reproduced empty-wake hazard from
   * genuine terminal output; neither event alone diagnoses an MCU reset. */
  if (claimed_terminal)
    printf("Display diagnostic: terminal takeover at %u ms, input=%u bytes\n",
           (unsigned int)(esp_timer_get_time() / 1000),
           (unsigned int)processed);
  return xStreamBufferBytesAvailable(terminal_stream) != 0u;
}

static bool display_terminal_init(uint32_t refresh_hz) {
  terminal.cells = heap_caps_calloc(
      DISPLAY_TERMINAL_COLUMNS * DISPLAY_TERMINAL_ROWS,
      sizeof(display_terminal_cell_t), MALLOC_CAP_SPIRAM);
  if (terminal.cells == NULL)
    return false;
  terminal_stream = xStreamBufferCreateStatic(
      DISPLAY_TERMINAL_STREAM_SIZE, 1u, terminal_stream_storage,
      &terminal_stream_control);
  if (terminal_stream == NULL)
    return false;
  terminal_cursor_period_frames = refresh_hz > 1u ? refresh_hz / 2u : 1u;
  display_terminal_reset_state(true);
  return true;
}

static bool display_batch_next(const uint8_t *payload, uint32_t payload_length,
                               uint32_t *offset,
                               display_pv_batch_record_t *record,
                               const uint8_t **record_payload) {
  uint32_t padded_length;
  uint32_t record_length;

  if (*offset > payload_length ||
      sizeof(*record) > payload_length - *offset)
    return false;
  memcpy(record, payload + *offset, sizeof(*record));
  if (record->payload_length > UINT32_MAX - 3u)
    return false;
  padded_length = (record->payload_length + 3u) & ~3u;
  if (padded_length > UINT32_MAX - sizeof(*record))
    return false;
  record_length = sizeof(*record) + padded_length;
  if (record_length > payload_length - *offset)
    return false;
  *record_payload = record->payload_length
                        ? payload + *offset + sizeof(*record)
                        : NULL;
  *offset += record_length;
  return true;
}

static bool display_batch_structurally_valid(const uint8_t *payload,
                                             uint32_t payload_length,
                                             uint32_t record_count) {
  uint32_t offset = 0;

  if (payload == NULL || payload_length == 0 || record_count == 0 ||
      record_count > payload_length / sizeof(display_pv_batch_record_t))
    return false;
  for (uint32_t index = 0; index < record_count; index++) {
    display_pv_batch_record_t record;
    const uint8_t *record_payload;

    if (!display_batch_next(payload, payload_length, &offset, &record,
                            &record_payload))
      return false;
    switch (record.operation) {
    case DISPLAY_PV_OP_COPY:
    case DISPLAY_PV_OP_TILE_FILL:
    case DISPLAY_PV_OP_TILE_CURSOR:
      if (record.payload_length != 0)
        return false;
      break;
    case DISPLAY_PV_OP_TILE_BLIT:
      if (record.args[6] == 0 ||
          record.args[6] > UINT32_MAX / sizeof(uint32_t) ||
          record.payload_length != record.args[6] * sizeof(uint32_t) ||
          record_payload == NULL)
        return false;
      break;
    case DISPLAY_PV_OP_TEXT_RUN16:
      if (record.args[6] == 0 ||
          record.args[6] > UINT32_MAX / sizeof(uint16_t) ||
          record.payload_length != record.args[6] * sizeof(uint16_t) ||
          (record.args[7] != 0xffu && record.args[7] != 0x1ffu) ||
          record_payload == NULL)
        return false;
      break;
    default:
      return false;
    }
  }
  return offset == payload_length;
}

static bool display_batch_tile_rect_valid(const display_pv_batch_record_t *record) {
  uint32_t x = record->args[0];
  uint32_t y = record->args[1];
  uint32_t width = record->args[2];
  uint32_t height = record->args[3];
  uint32_t columns = tile_width ? DISPLAY_FB_WIDTH / tile_width : 0;
  uint32_t rows = tile_height ? DISPLAY_FB_HEIGHT / tile_height : 0;

  return columns != 0 && rows != 0 && width != 0 && height != 0 &&
         x < columns && y < rows && width <= columns - x &&
         height <= rows - y;
}

static bool display_batch_semantically_valid(const uint8_t *payload,
                                             uint32_t payload_length,
                                             uint32_t record_count) {
  uint32_t offset = 0;

  if (payload == NULL || payload_length == 0 || record_count == 0 ||
      record_count > payload_length / sizeof(display_pv_batch_record_t) ||
      !__atomic_load_n(&tile_font_valid, __ATOMIC_ACQUIRE))
    return false;
  for (uint32_t record_index = 0; record_index < record_count;
       record_index++) {
    display_pv_batch_record_t record;
    const uint8_t *record_payload;

    if (!display_batch_next(payload, payload_length, &offset, &record,
                            &record_payload))
      return false;
    switch (record.operation) {
    case DISPLAY_PV_OP_COPY:
      if (record.payload_length != 0 ||
          !display_rect_valid(record.args[0], record.args[1], record.args[4],
                              record.args[5]) ||
          !display_rect_valid(record.args[2], record.args[3], record.args[4],
                              record.args[5]) ||
          tile_width == 0 || tile_height == 0 ||
          record.args[0] % tile_width || record.args[2] % tile_width ||
          record.args[4] % tile_width || record.args[1] % tile_height ||
          record.args[3] % tile_height || record.args[5] % tile_height)
        return false;
      break;
    case DISPLAY_PV_OP_TILE_FILL:
      if (record.payload_length != 0 ||
          !display_batch_tile_rect_valid(&record) ||
          record.args[4] >= tile_count ||
          record.args[7] != DISPLAY_PV_ROP_COPY)
        return false;
      break;
    case DISPLAY_PV_OP_TILE_BLIT:
      if (!display_batch_tile_rect_valid(&record) ||
          record.args[2] > UINT32_MAX / record.args[3] ||
          record.args[6] != record.args[2] * record.args[3] ||
          record.args[6] > UINT32_MAX / sizeof(uint32_t) ||
          record.payload_length != record.args[6] * sizeof(uint32_t) ||
          record_payload == NULL)
        return false;
      break;
    case DISPLAY_PV_OP_TEXT_RUN16:
      if (!display_batch_tile_rect_valid(&record) ||
          record.args[2] > UINT32_MAX / record.args[3] ||
          record.args[6] != record.args[2] * record.args[3] ||
          record.args[6] > UINT32_MAX / sizeof(uint16_t) ||
          record.payload_length != record.args[6] * sizeof(uint16_t) ||
          (record.args[7] != 0xffu && record.args[7] != 0x1ffu) ||
          record_payload == NULL)
        return false;
      break;
    case DISPLAY_PV_OP_TILE_CURSOR: {
      uint32_t columns = tile_width ? DISPLAY_FB_WIDTH / tile_width : 0;
      uint32_t rows = tile_height ? DISPLAY_FB_HEIGHT / tile_height : 0;

      if (record.payload_length != 0 || record.args[2] > 1u ||
          record.args[3] > 5u ||
          record.args[0] >= columns || record.args[1] >= rows)
        return false;
      break;
    }
    default:
      return false;
    }
  }
  return offset == payload_length;
}

static bool display_tile_batch_sequential(const uint8_t *payload,
                                          uint32_t payload_length,
                                          uint32_t record_count) {
  uint32_t offset = 0;

  for (uint32_t record_index = 0; record_index < record_count;
       record_index++) {
    display_pv_batch_record_t record;
    display_pv_command_t command = { 0 };
    const uint8_t *record_payload;
    bool success;

    if (!display_batch_next(payload, payload_length, &offset, &record,
                            &record_payload))
      return false;
    memcpy(command.args, record.args, sizeof(command.args));
    switch (record.operation) {
    case DISPLAY_PV_OP_COPY:
      success = display_copy_rect(&command);
      break;
    case DISPLAY_PV_OP_TILE_FILL:
      success = display_tile_fill(&command);
      break;
    case DISPLAY_PV_OP_TILE_BLIT:
      success = display_tile_blit(&command, record_payload,
                                  record.payload_length);
      break;
    case DISPLAY_PV_OP_TEXT_RUN16:
      success = display_text_run16(&command, record_payload,
                                   record.payload_length);
      break;
    case DISPLAY_PV_OP_TILE_CURSOR:
      success = display_set_tile_cursor(&command);
      break;
    default:
      return false;
    }
    if (!success)
      return false;
  }
  DISPLAY_PERF_ADD(tile_batch_records, record_count);
  return true;
}

static bool display_tile_batch(const display_pv_command_t *command,
                               const uint8_t *payload,
                               uint32_t payload_length) {
  uint32_t record_count = command->args[0];

  if (command->args[1] != payload_length ||
      !display_batch_semantically_valid(payload, payload_length,
                                        record_count)) {
    DISPLAY_PERF_ADD(tile_batch_fallbacks, 1u);
    return false;
  }
  /* Keep one FIFO entry and one CPU0-to-CPU1 wake for the transaction, then
   * execute the established tile primitives in order. The attempted shadow
   * compositor repainted large terminal regions even when no cell was
   * overwritten, increasing PPA traffic and occasionally corrupting scanout.
   */
  return display_tile_batch_sequential(payload, payload_length,
                                       record_count);
}

static bool display_image1(const display_pv_command_t *command,
                           const uint8_t *payload,
                           uint32_t payload_length) {
  uint32_t x = command->args[0];
  uint32_t y = command->args[1];
  uint32_t width = command->args[2];
  uint32_t height = command->args[3];
  uint16_t foreground = (uint16_t)command->args[4];
  uint16_t background = (uint16_t)command->args[5];
  uint32_t depth = command->args[6];
  uint32_t data_length = command->args[7];
  size_t pitch;
  size_t expected;

  if (payload == NULL || depth != 1 ||
      !display_rect_valid(x, y, width, height))
    return false;
  pitch = (width + 7u) / 8u;
  expected = pitch * height;
  if (data_length != expected || data_length != payload_length)
    return false;

  display_prepare_glyph_lut(foreground, background);

  for (uint32_t row = 0; row < height; row++) {
    uint16_t *destination = (uint16_t *)(framebuffer +
        ((size_t)(y + row) * DISPLAY_FB_WIDTH + x) * sizeof(uint16_t));
    const uint8_t *source = payload + row * pitch;

    display_expand_mono_row(destination, source, width);
  }
  display_mark_accel_rect(x, y, width, height);
  return true;
}

static bool display_bulk_write(const display_pv_command_t *command,
                               const uint8_t *payload,
                               uint32_t payload_length) {
  uint32_t offset = command->args[0];

  if (payload == NULL || payload_length == 0u ||
      command->args[1] != payload_length || offset >= DISPLAY_FB_SIZE ||
      payload_length > DISPLAY_FB_SIZE - offset)
    return false;

  display_terminal_release_framebuffer();
  memcpy(framebuffer + offset, payload, payload_length);
  display_mark_accel_byte_range(offset, payload_length);
  DISPLAY_PERF_ADD(bulk_write_bytes, payload_length);
  return true;
}

static void display_pv_record_fifo_error(void) {
  (void)__atomic_add_fetch(&pv_fifo_errors, 1u, __ATOMIC_RELAXED);
}

static uint32_t display_pv_report_busy(void) {
  DISPLAY_PERF_ADD(fifo_busy, 1u);
  return DISPLAY_PV_STATUS_BUSY;
}

static void display_pv_release_entry(display_pv_queue_entry_t *entry,
                                     uint32_t tail) {
  uint8_t *payload = entry->payload;
  uint32_t payload_free_bit = entry->payload_free_bit;

  entry->payload = NULL;
  entry->payload_length = 0;
  entry->payload_free_bit = 0;
  if (payload_free_bit != 0) {
    uint32_t valid_mask =
        DISPLAY_PV_PAYLOAD_BLOCK_COUNT == 32u
            ? UINT32_MAX
            : (1u << DISPLAY_PV_PAYLOAD_BLOCK_COUNT) - 1u;

    if (payload_free_bit == 0 || (payload_free_bit & valid_mask) == 0 ||
        (payload_free_bit & (payload_free_bit - 1u)) != 0) {
      display_pv_record_fifo_error();
    } else {
      uint32_t previous = __atomic_fetch_or(
          &pv_payload_free_mask, payload_free_bit, __ATOMIC_RELEASE);

      if (previous & payload_free_bit)
        display_pv_record_fifo_error();
    }
  } else if (payload != NULL && payload != entry->inline_payload) {
    display_pv_record_fifo_error();
  }
  __atomic_store_n(&pv_consumer_tail, tail + 1u, __ATOMIC_RELEASE);
}

static bool display_process_pv_commands(void) {
  uint32_t processed = 0;
  bool commands_pending;

  /* The cursor is an XOR overlay. Remove it once for the whole command slice,
   * rather than twice around every glyph/copy, and draw only the final cursor
   * state after all underlying pixels have been updated. */
  if (tile_cursor_drawn)
    display_toggle_tile_cursor();

  while (processed < CONFIG_RV32_DISPLAY_COMMAND_SLICE) {
    uint32_t tail =
        __atomic_load_n(&pv_consumer_tail, __ATOMIC_RELAXED);
    uint32_t head =
        __atomic_load_n(&pv_producer_head, __ATOMIC_ACQUIRE);
    display_pv_queue_entry_t *entry;

    if (tail == head)
      break;
    entry = &pv_queue_entries[tail % DISPLAY_PV_FIFO_DEPTH];
    const display_pv_command_t *command = &entry->command;
    bool success = false;
#if CONFIG_RV32_HOST_PERF_STATS
    uint32_t command_started = esp_cpu_get_cycle_count();
#endif

    ppa_accel_active = false;
    switch (command->operation) {
    case DISPLAY_PV_OP_FILL:
      DISPLAY_PERF_ADD(fill_commands, 1u);
      success = display_fill_rect(command);
      break;
    case DISPLAY_PV_OP_COPY:
      DISPLAY_PERF_ADD(copy_commands, 1u);
      success = display_copy_rect(command);
      break;
    case DISPLAY_PV_OP_IMAGE1:
      DISPLAY_PERF_ADD(tile_commands, 1u);
      DISPLAY_PERF_ADD(image1_commands, 1u);
      success = display_image1(command, entry->payload,
                               entry->payload_length);
      break;
    case DISPLAY_PV_OP_BULK_WRITE:
      DISPLAY_PERF_ADD(bulk_write_commands, 1u);
      success = display_bulk_write(command, entry->payload,
                                   entry->payload_length);
      break;
    case DISPLAY_PV_OP_SET_TILE:
      DISPLAY_PERF_ADD(tile_commands, 1u);
      DISPLAY_PERF_ADD(tile_set_commands, 1u);
      success = display_set_tile(command, entry->payload,
                                 entry->payload_length);
      break;
    case DISPLAY_PV_OP_TILE_FILL:
      DISPLAY_PERF_ADD(tile_commands, 1u);
      DISPLAY_PERF_ADD(tile_fill_commands, 1u);
      success = display_tile_fill(command);
      break;
    case DISPLAY_PV_OP_TILE_BLIT:
      DISPLAY_PERF_ADD(tile_commands, 1u);
      DISPLAY_PERF_ADD(tile_blit_commands, 1u);
      success = display_tile_blit(command, entry->payload,
                                  entry->payload_length);
      break;
    case DISPLAY_PV_OP_TEXT_RUN16:
      DISPLAY_PERF_ADD(tile_commands, 1u);
      DISPLAY_PERF_ADD(tile_blit_commands, 1u);
      success = display_text_run16(command, entry->payload,
                                   entry->payload_length);
      break;
    case DISPLAY_PV_OP_TILE_CURSOR:
      DISPLAY_PERF_ADD(tile_commands, 1u);
      DISPLAY_PERF_ADD(tile_cursor_commands, 1u);
      success = display_set_tile_cursor(command);
      break;
    case DISPLAY_PV_OP_TILE_BATCH:
      DISPLAY_PERF_ADD(tile_commands, 1u);
      DISPLAY_PERF_ADD(tile_batch_commands, 1u);
      success = display_tile_batch(command, entry->payload,
                                   entry->payload_length);
      break;
    default:
      break;
    }
#if CONFIG_RV32_HOST_PERF_STATS
    uint32_t command_cycles = esp_cpu_get_cycle_count() - command_started;

    DISPLAY_PERF_ADD(command_cycles, command_cycles);
    display_perf_update_max(&display_perf.command_max_cycles,
                            command_cycles);
    switch (command->operation) {
    case DISPLAY_PV_OP_FILL:
      DISPLAY_PERF_ADD(fill_cycles, command_cycles);
      break;
    case DISPLAY_PV_OP_COPY:
      DISPLAY_PERF_ADD(copy_cycles, command_cycles);
      break;
    case DISPLAY_PV_OP_IMAGE1:
      DISPLAY_PERF_ADD(image1_cycles, command_cycles);
      break;
    case DISPLAY_PV_OP_BULK_WRITE:
      DISPLAY_PERF_ADD(bulk_write_cycles, command_cycles);
      break;
    case DISPLAY_PV_OP_SET_TILE:
      DISPLAY_PERF_ADD(tile_set_cycles, command_cycles);
      break;
    case DISPLAY_PV_OP_TILE_FILL:
      DISPLAY_PERF_ADD(tile_fill_cycles, command_cycles);
      break;
    case DISPLAY_PV_OP_TILE_BLIT:
    case DISPLAY_PV_OP_TEXT_RUN16:
      DISPLAY_PERF_ADD(tile_blit_cycles, command_cycles);
      break;
    case DISPLAY_PV_OP_TILE_CURSOR:
      DISPLAY_PERF_ADD(tile_cursor_cycles, command_cycles);
      break;
    case DISPLAY_PV_OP_TILE_BATCH:
      DISPLAY_PERF_ADD(tile_batch_cycles, command_cycles);
      break;
    default:
      break;
    }
#endif
    DISPLAY_PERF_ADD(commands, 1u);

    /* The sequence is the release marker for its status. Pollers acquire the
     * sequence before consuming status, avoiding a cross-core spinlock for
     * every completed command. */
    __atomic_store_n(&pv_completed_status,
                     success ? DISPLAY_PV_STATUS_OK
                             : DISPLAY_PV_STATUS_INVALID,
                     __ATOMIC_RELAXED);
    __atomic_store_n(&pv_completed_sequence, command->sequence,
                     __ATOMIC_RELEASE);
    display_pv_release_entry(entry, tail);
    processed++;
  }

  commands_pending = display_pv_commands_pending();
  /* Keep the XOR cursor hidden while another bounded slice is already
   * queued. Redrawing it here only to erase it at the start of the next
   * service pass adds pixel writes and cache traffic with no visible frame
   * between the two operations. */
  if (!commands_pending && tile_cursor_active && !tile_cursor_drawn)
    display_toggle_tile_cursor();

  /* A slice may contain several PPA backgrounds and CPU-expanded glyph runs.
   * Publish their merged dirty rows once, as one completed composite. */
  if (ppa_fill_touched_framebuffer && display_flush_accel_dirty())
    ppa_fill_touched_framebuffer = false;

  if (processed != 0)
    DISPLAY_PERF_ADD(fifo_slices, 1u);
  if (commands_pending) {
    DISPLAY_PERF_ADD(fifo_deferred, 1u);
    return true;
  }
  return false;
}

static void display_retry_ppa_frame(uint32_t width, uint32_t height) {
  portENTER_CRITICAL(&dirty_lock);
  if (!ppa_frame_pending && !ppa_stop_pending) {
    ppa_pending_width = width;
    ppa_pending_height = height;
    ppa_frame_pending = true;
  }
  portEXIT_CRITICAL(&dirty_lock);
}

static bool display_process_ppa_command(void) {
  bool frame_pending;
  bool stop_pending;
  uint32_t source_width;
  uint32_t source_height;
  uint32_t scale_sixteenths;
  uint32_t output_width;
  uint32_t output_height;
  static bool error_reported;
  esp_err_t err;

  portENTER_CRITICAL(&dirty_lock);
  frame_pending = ppa_frame_pending;
  stop_pending = ppa_stop_pending;
  source_width = ppa_pending_width;
  source_height = ppa_pending_height;
  ppa_frame_pending = false;
  ppa_stop_pending = false;
  portEXIT_CRITICAL(&dirty_lock);

  if (stop_pending) {
    ppa_accel_active = false;
    return true;
  }
  if (!frame_pending || !ppa_ready)
    return !frame_pending;

  if (!ppa_accel_active || source_width != ppa_active_width ||
      source_height != ppa_active_height) {
    memset(framebuffer, 0, DISPLAY_FB_SIZE);
    if (framebuffer != scanout_framebuffer) {
      display_mark_present_rect(0u, 0u, DISPLAY_FB_WIDTH,
                                DISPLAY_FB_HEIGHT);
      err = ESP_OK;
    } else {
      err = display_cache_clean(framebuffer, DISPLAY_FB_SIZE);
    }
    if (err != ESP_OK) {
      display_retry_ppa_frame(source_width, source_height);
      return false;
    }
    ppa_accel_active = true;
    ppa_active_width = source_width;
    ppa_active_height = source_height;
  }

  /* PPA scaling has 1/16 resolution.  Quantize downward so the result
   * always remains inside the scanout, then center it. */
  scale_sixteenths = DISPLAY_FB_WIDTH * 16u / source_width;
  uint32_t vertical_scale = DISPLAY_FB_HEIGHT * 16u / source_height;
  if (vertical_scale < scale_sixteenths)
    scale_sixteenths = vertical_scale;
  if (scale_sixteenths > 4095u)
    scale_sixteenths = 4095u;
  if (scale_sixteenths == 0) {
    display_retry_ppa_frame(source_width, source_height);
    return false;
  }
  output_width = source_width * scale_sixteenths / 16u;
  output_height = source_height * scale_sixteenths / 16u;

  const ppa_srm_oper_config_t operation = {
      .in =
          {
              .buffer = ppa_staging,
              .pic_w = source_width,
              .pic_h = source_height,
              .block_w = source_width,
              .block_h = source_height,
              .srm_cm = PPA_SRM_COLOR_MODE_RGB565,
          },
      .out =
          {
              .buffer = framebuffer,
              .buffer_size = DISPLAY_FB_SIZE,
              .pic_w = DISPLAY_FB_WIDTH,
              .pic_h = DISPLAY_FB_HEIGHT,
              .block_offset_x = (DISPLAY_FB_WIDTH - output_width) / 2u,
              .block_offset_y = (DISPLAY_FB_HEIGHT - output_height) / 2u,
              .srm_cm = PPA_SRM_COLOR_MODE_RGB565,
          },
      .rotation_angle = PPA_SRM_ROTATION_ANGLE_0,
      .scale_x = scale_sixteenths / 16.0f,
      .scale_y = scale_sixteenths / 16.0f,
      .mode = PPA_TRANS_MODE_BLOCKING,
  };

  #if CONFIG_RV32_HOST_PERF_STATS
  int64_t ppa_started = esp_timer_get_time();
  #endif
  err = ppa_do_scale_rotate_mirror(ppa_srm, &operation);
  #if CONFIG_RV32_HOST_PERF_STATS
  DISPLAY_PERF_ADD(ppa_us, esp_timer_get_time() - ppa_started);
  if (err == ESP_OK)
    DISPLAY_PERF_ADD(ppa_blits, 1u);
  #endif
  if (err != ESP_OK && !error_reported) {
    printf("WARNING: PPA display blit failed: %s\n", esp_err_to_name(err));
    error_reported = true;
  } else if (err == ESP_OK) {
    uint32_t output_x = (DISPLAY_FB_WIDTH - output_width) / 2u;
    uint32_t output_y = (DISPLAY_FB_HEIGHT - output_height) / 2u;

    error_reported = false;
    display_mark_present_rect(output_x, output_y, output_width,
                              output_height);
  }
  if (err != ESP_OK)
    display_retry_ppa_frame(source_width, source_height);
  return err == ESP_OK;
}

static esp_err_t display_present_render_surface(void) {
  static bool ppa_error_reported;
  esp_err_t err = ESP_ERR_NOT_SUPPORTED;
  bool used_ppa = false;
  uint32_t first_column;
  uint32_t last_column;
  uint32_t column_count;
  uint32_t first_row;
  uint32_t last_row;
  uint32_t row_count;
  size_t row_bytes;
  size_t present_offset;
  size_t present_end;
#if CONFIG_RV32_HOST_PERF_STATS
  int64_t started;
#endif

  if (framebuffer == NULL || scanout_framebuffer == NULL)
    return ESP_ERR_INVALID_STATE;
  if (framebuffer == scanout_framebuffer || !framebuffer_present_pending)
    return ESP_OK;

  first_column = framebuffer_present_first_column;
  last_column = framebuffer_present_last_column;
  first_row = framebuffer_present_first_row;
  last_row = framebuffer_present_last_row;
  if (first_column >= last_column || last_column > DISPLAY_FB_WIDTH ||
      first_row >= last_row || last_row > DISPLAY_FB_HEIGHT)
    return ESP_ERR_INVALID_STATE;
  column_count = last_column - first_column;
  row_count = last_row - first_row;
  row_bytes = (size_t)column_count * sizeof(uint16_t);
  present_offset = (size_t)first_row * DISPLAY_FB_STRIDE +
                   (size_t)first_column * sizeof(uint16_t);
  present_end = (size_t)(last_row - 1u) * DISPLAY_FB_STRIDE +
                (size_t)last_column * sizeof(uint16_t);

#if CONFIG_RV32_HOST_PERF_STATS
  started = esp_timer_get_time();
#endif
  if (ppa_ready) {
    const ppa_srm_oper_config_t operation = {
        .in =
            {
                .buffer = framebuffer,
                .pic_w = DISPLAY_FB_WIDTH,
                .pic_h = DISPLAY_FB_HEIGHT,
                .block_w = column_count,
                .block_h = row_count,
                .block_offset_x = first_column,
                .block_offset_y = first_row,
                .srm_cm = PPA_SRM_COLOR_MODE_RGB565,
            },
        .out =
            {
                .buffer = scanout_framebuffer,
                .buffer_size = DISPLAY_FB_SIZE,
                .pic_w = DISPLAY_FB_WIDTH,
                .pic_h = DISPLAY_FB_HEIGHT,
                .block_offset_x = first_column,
                .block_offset_y = first_row,
                .srm_cm = PPA_SRM_COLOR_MODE_RGB565,
            },
        .rotation_angle = PPA_SRM_ROTATION_ANGLE_0,
        .scale_x = 1.0f,
        .scale_y = 1.0f,
        .mode = PPA_TRANS_MODE_BLOCKING,
    };

    err = ppa_do_scale_rotate_mirror(ppa_srm, &operation);
    used_ppa = err == ESP_OK;
    if (used_ppa) {
      ppa_error_reported = false;
      DISPLAY_PERF_ADD(ppa_blits, 1u);
    } else if (!ppa_error_reported) {
      printf("WARNING: PPA render-surface present failed (%s); using CPU1 "
             "copy fallback\n",
             esp_err_to_name(err));
      ppa_error_reported = true;
    }
  }

  if (!used_ppa) {
    for (uint32_t row = first_row; row < last_row; row++) {
      size_t row_offset = (size_t)row * DISPLAY_FB_STRIDE +
                          (size_t)first_column * sizeof(uint16_t);

      memcpy(scanout_framebuffer + row_offset,
             framebuffer + row_offset, row_bytes);
    }
    /* One enclosing cache operation avoids a costly call per row. It may
     * publish unchanged stride gaps, but CPU copying and the reported
     * presentation-byte count remain limited to the damaged rectangle. */
    err = display_cache_clean(scanout_framebuffer + present_offset,
                              present_end - present_offset);
    DISPLAY_PERF_ADD(framebuffer_present_cpu_copies, 1u);
  }

#if CONFIG_RV32_HOST_PERF_STATS
  uint32_t elapsed = (uint32_t)(esp_timer_get_time() - started);

  DISPLAY_PERF_ADD(framebuffer_present_us, elapsed);
  if (used_ppa)
    DISPLAY_PERF_ADD(ppa_us, elapsed);
#endif
  if (err != ESP_OK)
    return err;

  framebuffer_present_pending = false;
  framebuffer_present_first_column = DISPLAY_FB_WIDTH;
  framebuffer_present_last_column = 0u;
  framebuffer_present_first_row = DISPLAY_FB_HEIGHT;
  framebuffer_present_last_row = 0u;
  DISPLAY_PERF_ADD(framebuffer_present_events, 1u);
  DISPLAY_PERF_ADD(framebuffer_present_bytes,
                   (size_t)row_count * row_bytes);
  return ESP_OK;
}

static void display_service_task(void *argument) {
  TaskHandle_t init_waiter = (TaskHandle_t)argument;
  bool present_error_reported = false;

  display_init_result = display_hardware_init();
  xTaskNotifyGive(init_waiter);

  if (display_init_result < 0) {
    display_task_handle = NULL;
    vTaskDelete(NULL);
    return;
  }

  for (;;) {
    uint32_t events;
    uint32_t sync_target;
    bool frame_ready;
    bool frame_changed;
    bool surface_locked = false;
    bool commands_pending = false;
    bool terminal_pending = false;

    xTaskNotifyWait(0, UINT32_MAX, &events, portMAX_DELAY);
#if CONFIG_RV32_HOST_PERF_STATS
    int64_t service_started = esp_timer_get_time();
#endif
#if defined(CONFIG_RV32_VIRTIO_GPU) && CONFIG_RV32_VIRTIO_GPU
    xSemaphoreTake(scanout_mutex, portMAX_DELAY);
#endif
    DISPLAY_PERF_ADD(service_wakes, 1u);
    if (__atomic_exchange_n(&raw_framebuffer_claim_pending, false,
                            __ATOMIC_ACQUIRE))
      display_terminal_release_framebuffer();
    /* A bounded slice prevents an always-full producer FIFO from hiding a
     * VSYNC bit behind an unbounded drain loop. */
    if ((events & LCD_NOTIFY_PV_COMMAND) ||
        display_pv_commands_pending())
      commands_pending = display_process_pv_commands();
    if ((events & LCD_NOTIFY_TERMINAL) ||
        (terminal_stream != NULL &&
         xStreamBufferBytesAvailable(terminal_stream) != 0u))
      terminal_pending = display_terminal_process_stream();
    if (events & LCD_NOTIFY_VSYNC) {
      portENTER_CRITICAL(&dirty_lock);
      frame_changed = pending_dirty || ppa_frame_pending || ppa_stop_pending;
      portEXIT_CRITICAL(&dirty_lock);
      frame_changed = frame_changed ||
                      accel_dirty_first < DISPLAY_FB_HEIGHT ||
                      ppa_fill_touched_framebuffer ||
                      framebuffer_present_pending;
      /* Take every live-scanout VSYNC. A producer can finish between the
       * preliminary dirty check and display_flush_dirty(); conditional locking
       * would allow that newly committed range to be copied concurrently with
       * the next emulator slice. The unchanged-frame critical section is only
       * a few flag checks, while a changed frame holds it for the PPA copy. */
      if (framebuffer != scanout_framebuffer) {
#if CONFIG_RV32_HOST_PERF_STATS
        int64_t snapshot_wait_started = esp_timer_get_time();
#endif
        xSemaphoreTake(framebuffer_mutex, portMAX_DELAY);
#if CONFIG_RV32_HOST_PERF_STATS
        uint32_t snapshot_wait_us =
            (uint32_t)(esp_timer_get_time() - snapshot_wait_started);

        DISPLAY_PERF_ADD(framebuffer_snapshot_wait_us, snapshot_wait_us);
        display_perf_update_max(&display_perf.framebuffer_snapshot_wait_max_us,
                                snapshot_wait_us);
#endif
        surface_locked = true;
        portENTER_CRITICAL(&dirty_lock);
        frame_changed = frame_changed || pending_dirty || ppa_frame_pending ||
                        ppa_stop_pending;
        portEXIT_CRITICAL(&dirty_lock);
        frame_changed = frame_changed ||
                        accel_dirty_first < DISPLAY_FB_HEIGHT ||
                        ppa_fill_touched_framebuffer ||
                        framebuffer_present_pending;
      }
#if CONFIG_RV32_HOST_PERF_STATS
      int64_t frame_started = esp_timer_get_time();
#endif
      DISPLAY_PERF_ADD(vsyncs, 1u);
#if CONFIG_RV32_DISPLAY_FLUSH_INTERVAL_MS > 0
      /* Accumulate nearby glyph stores into one PSRAM cache-clean range. */
      vTaskDelay(pdMS_TO_TICKS(CONFIG_RV32_DISPLAY_FLUSH_INTERVAL_MS));
#endif
      /* Snapshot the fence before consuming its pixels.  A request arriving
       * during this VSYNC is deliberately left for the next physical frame. */
      portENTER_CRITICAL(&dirty_lock);
      sync_target = frame_sync_requested;
      portEXIT_CRITICAL(&dirty_lock);
      frame_ready = display_flush_dirty();
      if (terminal_owns_framebuffer && terminal.active &&
          tile_cursor_active &&
          ++terminal_cursor_vsyncs >= terminal_cursor_period_frames) {
        display_toggle_tile_cursor();
        frame_changed = true;
        terminal_cursor_vsyncs = 0;
      }
      if (!display_flush_accel_dirty()) {
        frame_ready = false;
      } else {
        ppa_fill_touched_framebuffer = false;
      }
      if (!display_process_ppa_command())
        frame_ready = false;
      bool producer_active =
          framebuffer != scanout_framebuffer &&
          __atomic_load_n(&producer_dirty, __ATOMIC_ACQUIRE);
      bool present_deferred = false;
      esp_err_t present_err = ESP_OK;

      if (producer_active) {
        frame_ready = false;
        present_deferred = framebuffer_present_pending;
        if (present_deferred)
          DISPLAY_PERF_ADD(framebuffer_present_deferred, 1u);
      } else {
        present_err = display_present_render_surface();
      }
      if (!present_deferred && present_err == ESP_OK)
        present_err = display_backend_present(frame_changed);
      if (present_err != ESP_OK) {
        frame_ready = false;
        if (!present_error_reported) {
          printf("WARNING: Display frame transfer failed: %s\n",
                 esp_err_to_name(present_err));
          present_error_reported = true;
        }
      } else {
        present_error_reported = false;
      }
      if (frame_ready) {
        portENTER_CRITICAL(&dirty_lock);
        frame_sync_completed = sync_target;
        portEXIT_CRITICAL(&dirty_lock);
      }
#if CONFIG_RV32_HOST_PERF_STATS
      if (frame_changed)
        display_perf_record_frame(
            (uint32_t)(esp_timer_get_time() - frame_started));
#endif
      if (surface_locked)
        xSemaphoreGive(framebuffer_mutex);
    }
    if (commands_pending &&
        xTaskNotify(display_task_handle, LCD_NOTIFY_PV_COMMAND, eSetBits) !=
            pdPASS)
      display_pv_record_fifo_error();
    if (terminal_pending)
      (void)xTaskNotify(display_task_handle, LCD_NOTIFY_TERMINAL, eSetBits);
#if defined(CONFIG_RV32_VIRTIO_GPU) && CONFIG_RV32_VIRTIO_GPU
    xSemaphoreGive(scanout_mutex);
#endif
#if CONFIG_RV32_HOST_PERF_STATS
    DISPLAY_PERF_ADD(service_us, esp_timer_get_time() - service_started);
#endif
  }
}

int display_bridge_init(void) {
  TaskHandle_t caller = xTaskGetCurrentTaskHandle();
  BaseType_t display_core =
      (CONFIG_FREERTOS_NUMBER_OF_CORES > 1)
          ? ((xPortGetCoreID() + 1) % CONFIG_FREERTOS_NUMBER_OF_CORES)
          : xPortGetCoreID();
  BaseType_t created;

  framebuffer_mutex =
      xSemaphoreCreateMutexStatic(&framebuffer_mutex_storage);
  if (framebuffer_mutex == NULL) {
    printf("WARNING: Failed to create framebuffer snapshot mutex\n");
    return -1;
  }

#if defined(CONFIG_RV32_VIRTIO_GPU) && CONFIG_RV32_VIRTIO_GPU
  scanout_mutex = xSemaphoreCreateMutexStatic(&scanout_mutex_storage);
  if (scanout_mutex == NULL) {
    printf("WARNING: Failed to create display scanout mutex\n");
    return -1;
  }
#endif

  if (!display_bulk_storage_init()) {
    printf("WARNING: Failed to allocate display bulk storage\n");
    return -1;
  }

  if (!display_pv_queue_init()) {
    printf("WARNING: Failed to create LCD command FIFO\n");
    return -1;
  }

  created = xTaskCreatePinnedToCore(
      display_service_task, "lcd_service", LCD_TASK_STACK_SIZE, caller,
      LCD_TASK_PRIORITY, &display_task_handle, display_core);

  if (created != pdPASS) {
    printf("WARNING: Failed to create LCD service task\n");
    return -1;
  }

  ulTaskNotifyTake(pdTRUE, portMAX_DELAY);
  return display_init_result;
}

void display_bridge_set_guest_memory(void *host_base, uint32_t guest_base,
                                     size_t size) {
  guest_memory_host = host_base;
  guest_memory_base = guest_base;
  guest_memory_size = size;
}

void *display_bridge_framebuffer_host(void) {
  return framebuffer;
}

void display_bridge_mark_framebuffer_dirty(size_t start, size_t end,
                                           uint32_t store_calls,
                                           uint32_t store_bytes) {
  if (framebuffer == NULL || start >= end || end > DISPLAY_FB_SIZE)
    return;

  if (start < producer_dirty_start)
    producer_dirty_start = start;
  if (end > producer_dirty_end)
    producer_dirty_end = end;
  producer_last_write_us = esp_timer_get_time();
  __atomic_store_n(&producer_dirty, true, __ATOMIC_RELEASE);
  /* Stop the terminal renderer as soon as a raw producer claims the logical
   * surface.  Presentation remains deferred until the completed-frame
   * boundary below. */
  __atomic_store_n(&raw_framebuffer_claim_pending, true, __ATOMIC_RELEASE);
  DISPLAY_PERF_ADD(framebuffer_store_calls, store_calls);
  DISPLAY_PERF_ADD(framebuffer_store_bytes, store_bytes);
}

void display_bridge_commit(void) {
  if (!__atomic_load_n(&producer_dirty, __ATOMIC_ACQUIRE))
    return;

  portENTER_CRITICAL(&dirty_lock);
  if (producer_dirty_start < pending_dirty_start)
    pending_dirty_start = producer_dirty_start;
  if (producer_dirty_end > pending_dirty_end)
    pending_dirty_end = producer_dirty_end;
  pending_dirty = true;
  portEXIT_CRITICAL(&dirty_lock);

  DISPLAY_PERF_ADD(framebuffer_commit_events, 1u);
  DISPLAY_PERF_ADD(framebuffer_commit_bytes,
                   producer_dirty_end - producer_dirty_start);

  __atomic_store_n(&producer_dirty, false, __ATOMIC_RELEASE);
  producer_last_write_us = 0;
  producer_dirty_start = DISPLAY_FB_SIZE;
  producer_dirty_end = 0;
}

void display_bridge_guest_slice_begin(void) {
  if (framebuffer_mutex != NULL && framebuffer != NULL &&
      framebuffer != scanout_framebuffer) {
#if CONFIG_RV32_HOST_PERF_STATS
    int64_t wait_started = esp_timer_get_time();
#endif
    xSemaphoreTake(framebuffer_mutex, portMAX_DELAY);
#if CONFIG_RV32_HOST_PERF_STATS
    uint32_t wait_us = (uint32_t)(esp_timer_get_time() - wait_started);

    DISPLAY_PERF_ADD(framebuffer_guest_slices, 1u);
    DISPLAY_PERF_ADD(framebuffer_guest_wait_us, wait_us);
    display_perf_update_max(&display_perf.framebuffer_guest_wait_max_us,
                            wait_us);
#endif
  }
}

void display_bridge_guest_slice_end(void) {
  /* Explicit fence writes call display_bridge_commit() immediately.  For an
   * unfenced mmap producer, require sustained quiet rather than one clean
   * execution slice: Linux's timer IRQ alone creates a clean slice every
   * 10 ms and used to expose partially copied scanlines. */
  if (__atomic_load_n(&producer_dirty, __ATOMIC_ACQUIRE) &&
      producer_last_write_us != 0 &&
      esp_timer_get_time() - producer_last_write_us >=
          (int64_t)CONFIG_RV32_DISPLAY_LEGACY_QUIET_MS * 1000)
    display_bridge_commit();
  if (framebuffer_mutex != NULL && framebuffer != NULL &&
      framebuffer != scanout_framebuffer)
    xSemaphoreGive(framebuffer_mutex);
}

void *display_bridge_accel_buffer(size_t *capacity) {
  if (capacity != NULL)
    *capacity = ppa_ready ? DISPLAY_ACCEL_STAGE_SIZE : 0;
  return ppa_ready ? ppa_staging : NULL;
}

bool display_bridge_accel_blit(uint32_t width, uint32_t height) {
  if (!ppa_ready || width == 0 || width > DISPLAY_ACCEL_MAX_WIDTH ||
      height == 0 || height > DISPLAY_ACCEL_MAX_HEIGHT)
    return false;

  portENTER_CRITICAL(&dirty_lock);
  ppa_pending_width = width;
  ppa_pending_height = height;
  ppa_stop_pending = false;
  ppa_frame_pending = true;
  portEXIT_CRITICAL(&dirty_lock);
  return true;
}

void display_bridge_accel_stop(void) {
  portENTER_CRITICAL(&dirty_lock);
  ppa_stop_pending = true;
  ppa_frame_pending = false;
  portEXIT_CRITICAL(&dirty_lock);
}

bool display_bridge_xrgb8888_available(void) {
#if defined(CONFIG_RV32_VIRTIO_GPU) && CONFIG_RV32_VIRTIO_GPU
  return framebuffer != NULL && gpu_staging != NULL && scanout_mutex != NULL;
#else
  return false;
#endif
}

bool display_bridge_xrgb8888_update(display_bridge_resource_reader_t reader,
                                    void *context, uint64_t source_offset,
                                    uint32_t source_stride,
                                    uint32_t destination_x,
                                    uint32_t destination_y, uint32_t width,
                                    uint32_t height) {
#if defined(CONFIG_RV32_VIRTIO_GPU) && CONFIG_RV32_VIRTIO_GPU
  size_t input_row_bytes;
  uint32_t rows_per_chunk;
  bool success = true;

  if (reader == NULL || !display_bridge_xrgb8888_available() ||
      !display_rect_valid(destination_x, destination_y, width, height) ||
      width > SIZE_MAX / sizeof(uint32_t))
    return false;
  input_row_bytes = (size_t)width * sizeof(uint32_t);
  if (source_stride < input_row_bytes ||
      source_offset > UINT64_MAX - (uint64_t)(height - 1u) * source_stride)
    return false;
  rows_per_chunk = DISPLAY_GPU_STAGE_SIZE / input_row_bytes;
  if (rows_per_chunk == 0u)
    return false;

  xSemaphoreTake(scanout_mutex, portMAX_DELAY);
  display_terminal_release_framebuffer();
  portENTER_CRITICAL(&dirty_lock);
  ppa_frame_pending = false;
  ppa_stop_pending = false;
  portEXIT_CRITICAL(&dirty_lock);
  ppa_accel_active = false;

  for (uint32_t first_row = 0; first_row < height;) {
    uint32_t chunk_rows = height - first_row;
    bool ppa_converted = false;

    if (chunk_rows > rows_per_chunk)
      chunk_rows = rows_per_chunk;
    for (uint32_t row = 0; row < chunk_rows; row++) {
      if (!reader(context,
                  source_offset + (uint64_t)(first_row + row) * source_stride,
                  gpu_staging + (size_t)row * input_row_bytes,
                  input_row_bytes)) {
        success = false;
        goto out;
      }
    }

    if (ppa_ready) {
      const ppa_srm_oper_config_t operation = {
          .in =
              {
                  .buffer = gpu_staging,
                  .pic_w = width,
                  .pic_h = chunk_rows,
                  .block_w = width,
                  .block_h = chunk_rows,
                  .srm_cm = PPA_SRM_COLOR_MODE_ARGB8888,
              },
          .out =
              {
                  .buffer = framebuffer,
                  .buffer_size = DISPLAY_FB_SIZE,
                  .pic_w = DISPLAY_FB_WIDTH,
                  .pic_h = DISPLAY_FB_HEIGHT,
                  .block_offset_x = destination_x,
                  .block_offset_y = destination_y + first_row,
                  .srm_cm = PPA_SRM_COLOR_MODE_RGB565,
              },
          .rotation_angle = PPA_SRM_ROTATION_ANGLE_0,
          .scale_x = 1.0f,
          .scale_y = 1.0f,
          .mode = PPA_TRANS_MODE_BLOCKING,
      };

      ppa_converted =
          ppa_do_scale_rotate_mirror(ppa_srm, &operation) == ESP_OK;
      if (ppa_converted) {
        ppa_fill_touched_framebuffer = true;
        display_mark_present_rect(destination_x,
                                  destination_y + first_row,
                                  width, chunk_rows);
        DISPLAY_PERF_ADD(ppa_blits, 1u);
      }
    }

    if (!ppa_converted) {
      for (uint32_t row = 0; row < chunk_rows; row++) {
        const uint32_t *source =
            (const uint32_t *)(gpu_staging + (size_t)row * input_row_bytes);
        uint16_t *destination = (uint16_t *)(framebuffer +
            ((size_t)(destination_y + first_row + row) * DISPLAY_FB_WIDTH +
             destination_x) * sizeof(uint16_t));

        for (uint32_t column = 0; column < width; column++) {
          uint32_t pixel = source[column];
          destination[column] = (uint16_t)(((pixel >> 8) & 0xf800u) |
                                           ((pixel >> 5) & 0x07e0u) |
                                           ((pixel >> 3) & 0x001fu));
        }
      }
      display_mark_accel_rect(destination_x, destination_y + first_row,
                              width, chunk_rows);
    }
    first_row += chunk_rows;
  }

out:
  xSemaphoreGive(scanout_mutex);
  return success;
#else
  (void)reader;
  (void)context;
  (void)source_offset;
  (void)source_stride;
  (void)destination_x;
  (void)destination_y;
  (void)width;
  (void)height;
  return false;
#endif
}

void display_bridge_xrgb8888_flush(void) {
#if defined(CONFIG_RV32_VIRTIO_GPU) && CONFIG_RV32_VIRTIO_GPU
  if (display_task_handle != NULL)
    (void)xTaskNotify(display_task_handle, LCD_NOTIFY_VSYNC, eSetBits);
#endif
}

bool display_bridge_terminal_write(const void *buffer, size_t length) {
  if (length == 0)
    return true;
  if (buffer == NULL)
    return false;
  /* A missing display is not backpressure for a headless virtio console; its
   * UART mirror remains useful and must continue consuming descriptors. */
  if (framebuffer == NULL || terminal_stream == NULL ||
      display_task_handle == NULL)
    return true;
  if (length > DISPLAY_TERMINAL_STREAM_SIZE ||
      xStreamBufferSpacesAvailable(terminal_stream) < length)
    return false;
  if (xStreamBufferSend(terminal_stream, buffer, length, 0) != length)
    return false;
  (void)xTaskNotify(display_task_handle, LCD_NOTIFY_TERMINAL, eSetBits);
  return true;
}

uint32_t display_bridge_terminal_columns(void) {
  return DISPLAY_TERMINAL_COLUMNS;
}

uint32_t display_bridge_terminal_rows(void) {
  return DISPLAY_TERMINAL_ROWS;
}

bool display_bridge_contains(uint32_t address, size_t width) {
  if (framebuffer == NULL || width == 0 || width > sizeof(uint32_t) ||
      address < DISPLAY_FB_GUEST_BASE)
    return false;

  uint32_t offset = address - DISPLAY_FB_GUEST_BASE;
  return offset < DISPLAY_FB_APERTURE_SIZE &&
         width <= DISPLAY_FB_APERTURE_SIZE - offset;
}

uint32_t display_bridge_load(uint32_t address, size_t width) {
  size_t offset = address - DISPLAY_FB_GUEST_BASE;
  const uint8_t *source = NULL;
  uint32_t synthetic = 0;

  if (offset < DISPLAY_FB_SIZE && width <= DISPLAY_FB_SIZE - offset) {
    source = framebuffer + offset;
  } else if (address >= DISPLAY_ACCEL_STAGE_GUEST_BASE &&
             address - DISPLAY_ACCEL_STAGE_GUEST_BASE <
                 DISPLAY_ACCEL_STAGE_SIZE &&
             width <= DISPLAY_ACCEL_STAGE_SIZE -
                          (address - DISPLAY_ACCEL_STAGE_GUEST_BASE) &&
             ppa_staging != NULL) {
    source = ppa_staging + address - DISPLAY_ACCEL_STAGE_GUEST_BASE;
  } else if (address >= DISPLAY_ACCEL_STATUS_GUEST_BASE &&
             address - DISPLAY_ACCEL_STATUS_GUEST_BASE < sizeof(uint32_t) &&
             width <= sizeof(uint32_t) -
                          (address - DISPLAY_ACCEL_STATUS_GUEST_BASE)) {
    synthetic = ppa_ready ? DISPLAY_ACCEL_STATUS_MAGIC : 0;
    synthetic >>= 8u * (address - DISPLAY_ACCEL_STATUS_GUEST_BASE);
    if (width == 1)
      return synthetic & 0xffu;
    if (width == 2)
      return synthetic & 0xffffu;
    return synthetic;
  } else if (address >= DISPLAY_FB_COMMIT_GUEST_BASE &&
             address - DISPLAY_FB_COMMIT_GUEST_BASE < sizeof(uint32_t) &&
             width <=
                 sizeof(uint32_t) - (address - DISPLAY_FB_COMMIT_GUEST_BASE)) {
    portENTER_CRITICAL(&dirty_lock);
    synthetic = frame_sync_completed;
    portEXIT_CRITICAL(&dirty_lock);
    synthetic >>= 8u * (address - DISPLAY_FB_COMMIT_GUEST_BASE);
    if (width == 1)
      return synthetic & 0xffu;
    if (width == 2)
      return synthetic & 0xffffu;
    return synthetic;
  } else if (address >= DISPLAY_PV_REG_GUEST_BASE &&
             address - DISPLAY_PV_REG_GUEST_BASE < DISPLAY_PV_REG_SIZE &&
             width <= DISPLAY_PV_REG_SIZE -
                          (address - DISPLAY_PV_REG_GUEST_BASE)) {
    uint32_t register_offset = address - DISPLAY_PV_REG_GUEST_BASE;
    uint32_t word_offset = register_offset & ~3u;

    switch (word_offset) {
    case DISPLAY_PV_REG_MAGIC:
      synthetic = DISPLAY_PV_MAGIC;
      break;
    case DISPLAY_PV_REG_VERSION:
      synthetic = DISPLAY_PV_VERSION;
      break;
    case DISPLAY_PV_REG_FEATURES:
      synthetic = DISPLAY_PV_FEATURE_FILL | DISPLAY_PV_FEATURE_COPY |
                  DISPLAY_PV_FEATURE_VSYNC_FENCE |
                  DISPLAY_PV_FEATURE_SURFACE_INFO;
      if (guest_memory_host != NULL)
        synthetic |= DISPLAY_PV_FEATURE_SHARED_COMMAND |
                     DISPLAY_PV_FEATURE_SHARED_RESULT;
      if (pv_fifo_ready) {
        synthetic |= DISPLAY_PV_FEATURE_ASYNC_FIFO;
        if (ppa_staging != NULL) {
          synthetic |= DISPLAY_PV_FEATURE_IMAGE1 | DISPLAY_PV_FEATURE_TILE |
                       DISPLAY_PV_FEATURE_PAYLOAD_POOL |
                       DISPLAY_PV_FEATURE_TILE_BATCH |
                       DISPLAY_PV_FEATURE_TEXT_RUN16;
          if (guest_memory_host != NULL)
            synthetic |= DISPLAY_PV_FEATURE_BULK_WRITE;
        }
      }
      break;
    case DISPLAY_PV_REG_COMPLETED:
      synthetic = __atomic_load_n(&pv_completed_sequence, __ATOMIC_ACQUIRE);
      break;
    case DISPLAY_PV_REG_STATUS:
      synthetic = __atomic_load_n(&pv_completed_status, __ATOMIC_ACQUIRE);
      break;
    case DISPLAY_PV_REG_ACCEPTED:
      synthetic = __atomic_load_n(&pv_accepted_sequence, __ATOMIC_ACQUIRE);
      break;
    case DISPLAY_PV_REG_ACCEPT_STATUS:
      synthetic = __atomic_load_n(&pv_accepted_status, __ATOMIC_ACQUIRE);
      break;
    case DISPLAY_PV_REG_QUEUE_DEPTH:
      synthetic = DISPLAY_PV_FIFO_DEPTH;
      break;
    case DISPLAY_PV_REG_QUEUE_FREE:
      if (pv_fifo_ready)
        synthetic = display_pv_queue_free();
      break;
    case DISPLAY_PV_REG_PAYLOAD_LIMIT:
      synthetic = DISPLAY_PV_PAYLOAD_SIZE;
      break;
    case DISPLAY_PV_REG_PAYLOAD_BLOCKS:
      synthetic = DISPLAY_PV_PAYLOAD_BLOCK_COUNT;
      break;
    case DISPLAY_PV_REG_PAYLOAD_FREE:
      if (pv_fifo_ready)
        synthetic = display_pv_payload_free_count();
      break;
    case DISPLAY_PV_REG_SURFACE_WIDTH:
      synthetic = DISPLAY_FB_WIDTH;
      break;
    case DISPLAY_PV_REG_SURFACE_HEIGHT:
      synthetic = DISPLAY_FB_HEIGHT;
      break;
    case DISPLAY_PV_REG_SURFACE_STRIDE:
      synthetic = DISPLAY_FB_STRIDE;
      break;
    case DISPLAY_PV_REG_SURFACE_FORMAT:
      synthetic = DISPLAY_PV_FORMAT_RGB565;
      break;
    case DISPLAY_PV_REG_FIFO_ERRORS:
      synthetic = __atomic_load_n(&pv_fifo_errors, __ATOMIC_RELAXED);
      break;
    default:
      synthetic = 0;
      break;
    }
    synthetic >>= 8u * (register_offset & 3u);
    if (width == 1)
      return synthetic & 0xffu;
    if (width == 2)
      return synthetic & 0xffffu;
    return synthetic;
  }
  if (source == NULL)
    return 0;

  /* The emulator validates this access before dispatching it here. */
  switch (width) {
  case 1:
    return source[0];
  case 2: {
    uint16_t value;

    __builtin_memcpy(&value, source, sizeof(value));
    return value;
  }
  case 4: {
    uint32_t value;

    __builtin_memcpy(&value, source, sizeof(value));
    return value;
  }
  default:
    return 0;
  }
}

static uint32_t display_pv_enqueue(const display_pv_command_t *command,
                                   uint32_t shared_payload_address,
                                   uint32_t shared_payload_length,
                                   bool shared_payload) {
  display_pv_queue_entry_t *entry;
  uint8_t *payload = NULL;
  const uint8_t *payload_source = NULL;
  uint32_t payload_length = 0;
  uint32_t payload_free_bit = 0;
  uint32_t head;
  uint32_t tail;
  bool payload_pool_exhausted = false;

  if (display_task_handle == NULL || !pv_fifo_ready ||
      command->sequence == 0)
    return DISPLAY_PV_STATUS_FAILED;

  switch (command->operation) {
  case DISPLAY_PV_OP_FILL:
    if (!display_rect_valid(command->args[0], command->args[1],
                            command->args[2], command->args[3]) ||
        (command->args[5] != DISPLAY_PV_ROP_COPY &&
         command->args[5] != DISPLAY_PV_ROP_XOR))
      return DISPLAY_PV_STATUS_INVALID;
    break;
  case DISPLAY_PV_OP_COPY:
    if (!display_rect_valid(command->args[0], command->args[1],
                            command->args[4], command->args[5]) ||
        !display_rect_valid(command->args[2], command->args[3],
                            command->args[4], command->args[5]))
      return DISPLAY_PV_STATUS_INVALID;
    break;
  case DISPLAY_PV_OP_IMAGE1: {
    uint32_t width = command->args[2];
    uint32_t height = command->args[3];
    uint32_t pitch;

    payload_length = command->args[7];
    if (ppa_staging == NULL || command->args[6] != 1 ||
        !display_rect_valid(command->args[0], command->args[1], width,
                            height))
      return DISPLAY_PV_STATUS_INVALID;
    pitch = (width + 7u) / 8u;
    if (height != 0 && pitch > UINT32_MAX / height)
      return DISPLAY_PV_STATUS_INVALID;
    if (payload_length == 0 || payload_length != pitch * height)
      return DISPLAY_PV_STATUS_INVALID;
    break;
  }
  case DISPLAY_PV_OP_BULK_WRITE:
    payload_length = command->args[1];
    if (payload_length == 0u || command->args[0] >= DISPLAY_FB_SIZE ||
        payload_length > DISPLAY_FB_SIZE - command->args[0])
      return DISPLAY_PV_STATUS_INVALID;
    break;
  case DISPLAY_PV_OP_SET_TILE:
    payload_length = command->args[4];
    if (!display_tile_description_valid(command, payload_length))
      return DISPLAY_PV_STATUS_INVALID;
    break;
  case DISPLAY_PV_OP_TILE_FILL:
    if (command->args[2] == 0 || command->args[3] == 0 ||
        command->args[7] != DISPLAY_PV_ROP_COPY)
      return DISPLAY_PV_STATUS_INVALID;
    break;
  case DISPLAY_PV_OP_TILE_BLIT: {
    uint32_t width = command->args[2];
    uint32_t height = command->args[3];

    payload_length = command->args[7];
    if (width == 0 || height == 0 || width > UINT32_MAX / height ||
        command->args[6] != width * height || command->args[6] == 0 ||
        command->args[6] > UINT32_MAX / sizeof(uint32_t) ||
        payload_length != command->args[6] * sizeof(uint32_t))
      return DISPLAY_PV_STATUS_INVALID;
    break;
  }
  case DISPLAY_PV_OP_TEXT_RUN16: {
    uint32_t width = command->args[2];
    uint32_t height = command->args[3];
    uint32_t count = command->args[6];

    if (width == 0 || height == 0 || width > UINT32_MAX / height ||
        count != width * height || count == 0 ||
        count > UINT32_MAX / sizeof(uint16_t) ||
        (command->args[7] != 0xffu && command->args[7] != 0x1ffu))
      return DISPLAY_PV_STATUS_INVALID;
    payload_length = count * sizeof(uint16_t);
    break;
  }
  case DISPLAY_PV_OP_TILE_CURSOR:
    if (command->args[2] > 1u || command->args[3] > 5u)
      return DISPLAY_PV_STATUS_INVALID;
    break;
  case DISPLAY_PV_OP_TILE_BATCH:
    payload_length = command->args[1];
    if (command->args[0] == 0 || payload_length == 0)
      return DISPLAY_PV_STATUS_INVALID;
    break;
  default:
    return DISPLAY_PV_STATUS_INVALID;
  }

  if (payload_length > DISPLAY_PV_PAYLOAD_SIZE ||
      payload_length > DISPLAY_ACCEL_STAGE_SIZE ||
      (payload_length != 0 && ppa_staging == NULL))
    return DISPLAY_PV_STATUS_INVALID;

  if (shared_payload) {
    if (shared_payload_length != payload_length)
      return DISPLAY_PV_STATUS_INVALID;
    if (payload_length != 0) {
      payload_source = display_guest_pointer(shared_payload_address,
                                             payload_length);
      if (payload_source == NULL)
        return DISPLAY_PV_STATUS_INVALID;
    }
  } else if (payload_length != 0) {
    payload_source = ppa_staging;
  }
  if (command->operation == DISPLAY_PV_OP_TILE_BATCH &&
      !display_batch_structurally_valid(payload_source, payload_length,
                                        command->args[0]))
    return DISPLAY_PV_STATUS_INVALID;

  head = __atomic_load_n(&pv_producer_head, __ATOMIC_RELAXED);
  tail = __atomic_load_n(&pv_consumer_tail, __ATOMIC_ACQUIRE);
  if (head - tail >= DISPLAY_PV_FIFO_DEPTH)
    return display_pv_report_busy();
  entry = &pv_queue_entries[head % DISPLAY_PV_FIFO_DEPTH];
  entry->payload = NULL;
  entry->payload_length = 0;
  entry->payload_free_bit = 0;
  if (payload_length != 0) {
    if (payload_length <= DISPLAY_PV_INLINE_PAYLOAD_SIZE) {
      payload = entry->inline_payload;
      DISPLAY_PERF_ADD(producer_inline_payloads, 1u);
    } else {
      payload = display_pv_payload_take(&payload_free_bit);
      if (payload == NULL)
        return display_pv_report_busy();
      payload_pool_exhausted =
          __atomic_load_n(&pv_payload_free_mask, __ATOMIC_ACQUIRE) == 0;
      DISPLAY_PERF_ADD(producer_external_payloads, 1u);
    }
  }
  entry->command = *command;
  entry->payload_length = payload_length;
  entry->payload = payload;
  entry->payload_free_bit = payload_free_bit;
  if (payload != NULL) {
    memcpy(payload, payload_source, payload_length);
    DISPLAY_PERF_ADD(producer_payload_bytes, payload_length);
  }
  /* Publish the fully initialized slot. CPU1 acquires this index before it
   * reads the descriptor or its payload. */
  __atomic_store_n(&pv_producer_head, head + 1u, __ATOMIC_RELEASE);
  /* Reload the consumer index after publishing. If CPU1 drained the old tail
   * concurrently, this accurately detects the new empty-to-nonempty edge. */
  tail = __atomic_load_n(&pv_consumer_tail, __ATOMIC_ACQUIRE);
  uint32_t queued = head + 1u - tail;

  display_perf_update_fifo_high_water(queued);
  /* Wake CPU1 on the first command of a new burst. Waiting for a full slice
   * made isolated glyphs depend on the next VSYNC, while busy output appeared
   * faster merely because it reached the old threshold. Once awake, CPU1
   * drains bounded slices and reschedules itself until the FIFO is empty. */
  if (queued == 1u || payload_pool_exhausted) {
#if CONFIG_RV32_HOST_PERF_STATS
    int64_t notify_started = esp_timer_get_time();
#endif
    BaseType_t notified =
        xTaskNotify(display_task_handle, LCD_NOTIFY_PV_COMMAND, eSetBits);
#if CONFIG_RV32_HOST_PERF_STATS
    DISPLAY_PERF_ADD(producer_wakes, 1u);
    DISPLAY_PERF_ADD(producer_wake_us,
                     esp_timer_get_time() - notify_started);
#endif
    if (notified != pdPASS)
      display_pv_record_fifo_error();
  }
  return DISPLAY_PV_STATUS_OK;
}

static uint32_t display_pv_submit_current(uint32_t *accepted_sequence,
                                          uint32_t *shared_result_address) {
  display_pv_command_t command = pv_registers;
  uint32_t shared_address = pv_shared_command_address;
  uint32_t payload_address = 0;
  uint32_t payload_length = 0;
  bool shared = false;

  *shared_result_address = 0;

  /* Consume this pointer once. A later legacy command therefore cannot
   * accidentally reuse a descriptor left by a failed or older submission. */
  pv_shared_command_address = 0;
  if (shared_address != 0) {
    display_pv_shared_command_t shared_command;
    const uint8_t *source = display_guest_pointer(
        shared_address, sizeof(shared_command));

    if (source == NULL) {
      *accepted_sequence = 0;
      return DISPLAY_PV_STATUS_INVALID;
    }
    __atomic_thread_fence(__ATOMIC_ACQUIRE);
    memcpy(&shared_command, source, sizeof(shared_command));
    if (shared_command.magic != DISPLAY_PV_SHARED_COMMAND_MAGIC &&
        shared_command.magic != DISPLAY_PV_SHARED_RESULT_MAGIC) {
      *accepted_sequence = shared_command.sequence;
      return DISPLAY_PV_STATUS_INVALID;
    }
    if (shared_command.magic == DISPLAY_PV_SHARED_RESULT_MAGIC) {
      if (display_guest_pointer(shared_address,
                                sizeof(display_pv_shared_result_t)) == NULL) {
        *accepted_sequence = shared_command.sequence;
        return DISPLAY_PV_STATUS_INVALID;
      }
      *shared_result_address =
          shared_address + offsetof(display_pv_shared_result_t,
                                    accepted_sequence);
    }
    command.sequence = shared_command.sequence;
    command.operation = shared_command.operation;
    memcpy(command.args, shared_command.args, sizeof(command.args));
    payload_address = shared_command.payload_address;
    payload_length = shared_command.payload_length;
    shared = true;
  }

  *accepted_sequence = command.sequence;
  return display_pv_enqueue(&command, payload_address, payload_length, shared);
}

void display_bridge_store(uint32_t address, uint32_t value, size_t width) {
  /* The emulator validates this access before dispatching it here. */
  size_t offset = address - DISPLAY_FB_GUEST_BASE;
  uint8_t *destination;

  if (address >= DISPLAY_PV_REG_GUEST_BASE &&
      address - DISPLAY_PV_REG_GUEST_BASE < DISPLAY_PV_REG_SIZE &&
      width == sizeof(uint32_t)) {
    uint32_t register_offset = address - DISPLAY_PV_REG_GUEST_BASE;

    switch (register_offset) {
    case DISPLAY_PV_REG_SEQUENCE:
      pv_registers.sequence = value;
      break;
    case DISPLAY_PV_REG_OPERATION:
      pv_registers.operation = value;
      break;
    case DISPLAY_PV_REG_ARG0:
    case DISPLAY_PV_REG_ARG1:
    case DISPLAY_PV_REG_ARG2:
    case DISPLAY_PV_REG_ARG3:
    case DISPLAY_PV_REG_ARG4:
    case DISPLAY_PV_REG_ARG5:
    case DISPLAY_PV_REG_ARG6:
    case DISPLAY_PV_REG_ARG7:
      pv_registers.args[(register_offset - DISPLAY_PV_REG_ARG0) /
                        sizeof(uint32_t)] = value;
      break;
    case DISPLAY_PV_REG_SHARED_COMMAND:
      pv_shared_command_address = value;
      break;
    case DISPLAY_PV_REG_DOORBELL: {
      uint32_t accepted_sequence;
      uint32_t shared_result_address;
      uint32_t status;
#if CONFIG_RV32_HOST_PERF_STATS
      int64_t submit_started;
#endif

      if (value != DISPLAY_PV_SUBMIT)
        break;
#if CONFIG_RV32_HOST_PERF_STATS
      submit_started = esp_timer_get_time();
#endif
      status = display_pv_submit_current(&accepted_sequence,
                                         &shared_result_address);
#if CONFIG_RV32_HOST_PERF_STATS
      DISPLAY_PERF_ADD(producer_us,
                       esp_timer_get_time() - submit_started);
      DISPLAY_PERF_ADD(producer_submissions, 1u);
#endif

      __atomic_store_n(&pv_accepted_status, status, __ATOMIC_RELAXED);
      __atomic_store_n(&pv_accepted_sequence, accepted_sequence,
                       __ATOMIC_RELEASE);
      if (shared_result_address != 0) {
        uint32_t *result = (uint32_t *)display_guest_pointer(
            shared_result_address, 2u * sizeof(uint32_t));

        if (result != NULL) {
          __atomic_store_n(&result[1], status, __ATOMIC_RELAXED);
          __atomic_store_n(&result[0], accepted_sequence, __ATOMIC_RELEASE);
          DISPLAY_PERF_ADD(producer_shared_results, 1u);
        } else {
          display_pv_record_fifo_error();
        }
      }
      break;
    }
    default:
      break;
    }
    return;
  }

  if (address == DISPLAY_ACCEL_SIZE_GUEST_BASE && width == sizeof(uint32_t)) {
    portENTER_CRITICAL(&dirty_lock);
    ppa_source_size = value;
    portEXIT_CRITICAL(&dirty_lock);
    return;
  }

  if (address == DISPLAY_ACCEL_COMMAND_GUEST_BASE &&
      width == sizeof(uint32_t)) {
    if (value == DISPLAY_ACCEL_COMMAND_STOP) {
      display_bridge_accel_stop();
    } else if (value == DISPLAY_ACCEL_COMMAND_BLIT && ppa_ready) {
      uint32_t source_size;

      portENTER_CRITICAL(&dirty_lock);
      source_size = ppa_source_size;
      portEXIT_CRITICAL(&dirty_lock);

      (void)display_bridge_accel_blit(source_size & 0xffffu, source_size >> 16);
    }
    return;
  }

  /* A full-frame writer rings this doorbell after its last pixel store. */
  if (address == DISPLAY_FB_COMMIT_GUEST_BASE && width == sizeof(uint32_t)) {
    if (value == DISPLAY_FB_COMMIT_SYNC) {
      display_bridge_commit();
      portENTER_CRITICAL(&dirty_lock);
      frame_sync_requested++;
      portEXIT_CRITICAL(&dirty_lock);
    }
    return;
  }

  if (offset < DISPLAY_FB_SIZE && width <= DISPLAY_FB_SIZE - offset) {
    destination = framebuffer + offset;
  } else if (address >= DISPLAY_ACCEL_STAGE_GUEST_BASE &&
             address - DISPLAY_ACCEL_STAGE_GUEST_BASE <
                 DISPLAY_ACCEL_STAGE_SIZE &&
             width <= DISPLAY_ACCEL_STAGE_SIZE -
                          (address - DISPLAY_ACCEL_STAGE_GUEST_BASE) &&
             ppa_staging != NULL) {
    destination = ppa_staging + address - DISPLAY_ACCEL_STAGE_GUEST_BASE;
  } else {
    return;
  }

  switch (width) {
  case 1:
    destination[0] = (uint8_t)value;
    break;
  case 2:
    __builtin_memcpy(destination, &value, sizeof(uint16_t));
    break;
  case 4:
    __builtin_memcpy(destination, &value, sizeof(uint32_t));
    break;
  default:
    return;
  }
  if (offset >= DISPLAY_FB_SIZE)
    return;

  display_bridge_mark_framebuffer_dirty(offset, offset + width, 1u, width);
}

void display_bridge_perf_read_and_reset(
    struct display_bridge_perf_stats *stats) {
  if (stats == NULL)
    return;

  memset(stats, 0, sizeof(*stats));
#if CONFIG_RV32_HOST_PERF_STATS
#define DISPLAY_PERF_TAKE(field)                                           \
  stats->field = __atomic_exchange_n(&display_perf.field, 0u,              \
                                      __ATOMIC_RELAXED)
  DISPLAY_PERF_TAKE(service_wakes);
  DISPLAY_PERF_TAKE(service_us);
  DISPLAY_PERF_TAKE(vsyncs);
  DISPLAY_PERF_TAKE(frame_samples);
  DISPLAY_PERF_TAKE(frame_total_us);
  DISPLAY_PERF_TAKE(frame_max_us);
  DISPLAY_PERF_TAKE(commands);
  DISPLAY_PERF_TAKE(command_cycles);
  DISPLAY_PERF_TAKE(command_max_cycles);
  DISPLAY_PERF_TAKE(fill_commands);
  DISPLAY_PERF_TAKE(fill_cycles);
  DISPLAY_PERF_TAKE(copy_commands);
  DISPLAY_PERF_TAKE(copy_cycles);
  DISPLAY_PERF_TAKE(image1_commands);
  DISPLAY_PERF_TAKE(image1_cycles);
  DISPLAY_PERF_TAKE(bulk_write_commands);
  DISPLAY_PERF_TAKE(bulk_write_cycles);
  DISPLAY_PERF_TAKE(bulk_write_bytes);
  DISPLAY_PERF_TAKE(tile_commands);
  DISPLAY_PERF_TAKE(tile_set_commands);
  DISPLAY_PERF_TAKE(tile_set_cycles);
  DISPLAY_PERF_TAKE(tile_fill_commands);
  DISPLAY_PERF_TAKE(tile_fill_cycles);
  DISPLAY_PERF_TAKE(tile_blit_commands);
  DISPLAY_PERF_TAKE(tile_blit_cycles);
  DISPLAY_PERF_TAKE(tile_cursor_commands);
  DISPLAY_PERF_TAKE(tile_cursor_cycles);
  DISPLAY_PERF_TAKE(tile_batch_commands);
  DISPLAY_PERF_TAKE(tile_batch_cycles);
  DISPLAY_PERF_TAKE(tile_batch_records);
  DISPLAY_PERF_TAKE(tile_batch_fallbacks);
  DISPLAY_PERF_TAKE(cursor_toggles);
  DISPLAY_PERF_TAKE(cursor_toggle_cycles);
  DISPLAY_PERF_TAKE(cursor_toggle_max_cycles);
  DISPLAY_PERF_TAKE(ppa_fills);
  DISPLAY_PERF_TAKE(ppa_blits);
  DISPLAY_PERF_TAKE(ppa_us);
  DISPLAY_PERF_TAKE(ppa_fill_pixels);
  DISPLAY_PERF_TAKE(cpu_fill_pixels);
  DISPLAY_PERF_TAKE(copy_pixels);
  DISPLAY_PERF_TAKE(tile_pixels);
  DISPLAY_PERF_TAKE(cache_syncs);
  DISPLAY_PERF_TAKE(cache_bytes);
  DISPLAY_PERF_TAKE(cache_us);
  DISPLAY_PERF_TAKE(framebuffer_store_calls);
  DISPLAY_PERF_TAKE(framebuffer_store_bytes);
  DISPLAY_PERF_TAKE(framebuffer_commit_events);
  DISPLAY_PERF_TAKE(framebuffer_commit_bytes);
  DISPLAY_PERF_TAKE(framebuffer_publish_events);
  DISPLAY_PERF_TAKE(framebuffer_publish_bytes);
  DISPLAY_PERF_TAKE(framebuffer_present_events);
  DISPLAY_PERF_TAKE(framebuffer_present_bytes);
  DISPLAY_PERF_TAKE(framebuffer_present_us);
  DISPLAY_PERF_TAKE(framebuffer_present_cpu_copies);
  DISPLAY_PERF_TAKE(framebuffer_present_deferred);
  DISPLAY_PERF_TAKE(framebuffer_guest_slices);
  DISPLAY_PERF_TAKE(framebuffer_guest_wait_us);
  DISPLAY_PERF_TAKE(framebuffer_guest_wait_max_us);
  DISPLAY_PERF_TAKE(framebuffer_snapshot_wait_us);
  DISPLAY_PERF_TAKE(framebuffer_snapshot_wait_max_us);
  DISPLAY_PERF_TAKE(fifo_busy);
  DISPLAY_PERF_TAKE(fifo_high_water);
  DISPLAY_PERF_TAKE(fifo_slices);
  DISPLAY_PERF_TAKE(fifo_deferred);
  DISPLAY_PERF_TAKE(producer_submissions);
  DISPLAY_PERF_TAKE(producer_us);
  DISPLAY_PERF_TAKE(producer_payload_bytes);
  DISPLAY_PERF_TAKE(producer_inline_payloads);
  DISPLAY_PERF_TAKE(producer_external_payloads);
  DISPLAY_PERF_TAKE(producer_shared_results);
  DISPLAY_PERF_TAKE(producer_wakes);
  DISPLAY_PERF_TAKE(producer_wake_us);
#undef DISPLAY_PERF_TAKE
#endif
}
