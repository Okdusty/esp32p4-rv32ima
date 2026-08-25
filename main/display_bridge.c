/*
 * Backend-neutral paravirtual RGB565 framebuffer bridge.
 *
 * Linux sees the DPI driver's scanout allocation as a simple-framebuffer at
 * DISPLAY_FB_GUEST_BASE.  Guest reads and writes are redirected to the real
 * ESP-IDF framebuffer by the emulator's MMIO callbacks.  A separate generic
 * RGB565 staging aperture can ask the ESP32-P4 PPA to scale and center a
 * smaller packed surface without changing tty0's native scanout geometry.
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
#include "esp_err.h"
#include "esp_heap_caps.h"
#include "esp_timer.h"
#include "freertos/FreeRTOS.h"
#include "freertos/queue.h"
#include "freertos/task.h"

/*
 * Dirty framebuffer writes accumulate until the panel's VSYNC interrupt. The
 * CPU1 service merges wide regions and cleans sparse glyph rows independently.
 */
#define LCD_TASK_STACK_SIZE 6144u
/*
 * ESP-Hosted creates its SDIO workers at priority 23.  Keep the short,
 * event-driven cache-clean task above them so a broken or disconnected C6
 * cannot freeze an otherwise healthy framebuffer.
 */
#define LCD_TASK_PRIORITY (configMAX_PRIORITIES - 1)
#define LCD_NOTIFY_VSYNC (1u << 0)
#define LCD_NOTIFY_PV_COMMAND (1u << 1)

#define DISPLAY_PV_FIFO_DEPTH CONFIG_RV32_DISPLAY_COMMAND_FIFO_DEPTH
#define DISPLAY_PV_PAYLOAD_SIZE CONFIG_RV32_DISPLAY_PAYLOAD_BLOCK_SIZE
#define DISPLAY_PV_PAYLOAD_BLOCK_COUNT CONFIG_RV32_DISPLAY_PAYLOAD_BLOCK_COUNT
#define DISPLAY_TILE_MAX_COUNT 256u
#define DISPLAY_TILE_FONT_SIZE DISPLAY_PV_PAYLOAD_SIZE
/* Gaps below this size are cheaper to write back than another cache API call. */
#define DISPLAY_CACHE_MSYNC_MERGE_GAP 256u

_Static_assert(DISPLAY_PV_PAYLOAD_SIZE <= DISPLAY_ACCEL_STAGE_SIZE,
               "FIFO payload must fit in the guest staging aperture");
_Static_assert(DISPLAY_PV_PAYLOAD_SIZE *
                       (DISPLAY_PV_PAYLOAD_BLOCK_COUNT + 1u) <=
                   160u * 1024u,
               "FIFO payload pool and tile cache exceed the RAM budget");

typedef struct {
  uint32_t sequence;
  uint32_t operation;
  uint32_t args[8];
} display_pv_command_t;

typedef struct {
  display_pv_command_t command;
  uint32_t payload_length;
  uint8_t *payload;
} display_pv_queue_entry_t;

typedef struct {
  uint16_t pixels[4];
} display_glyph_nibble_t;

static uint8_t *framebuffer;
/* CPU0 owns the producer range; CPU1 only touches the pending range. */
static bool producer_dirty;
static size_t producer_dirty_start = DISPLAY_FB_SIZE;
static size_t producer_dirty_end;
static bool pending_dirty;
static size_t pending_dirty_start = DISPLAY_FB_SIZE;
static size_t pending_dirty_end;
static uint8_t *ppa_staging;
static ppa_client_handle_t ppa_srm;
static ppa_client_handle_t ppa_fill;
static bool ppa_ready;
static bool ppa_fill_ready;
/* A PPA fill reaches scanout memory immediately. If the same command adds
 * CPU-drawn pixels, publish those pixels before the panel can show only the
 * PPA-written background. */
static bool ppa_fill_touched_scanout;
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
static uint8_t pv_payload_blocks[DISPLAY_PV_PAYLOAD_BLOCK_COUNT]
                                [DISPLAY_PV_PAYLOAD_SIZE]
    __attribute__((aligned(64)));
static QueueHandle_t pv_ready_queue;
static QueueHandle_t pv_free_queue;
static QueueHandle_t pv_payload_free_queue;
static StaticQueue_t pv_ready_queue_control;
static StaticQueue_t pv_free_queue_control;
static StaticQueue_t pv_payload_free_queue_control;
static uint8_t pv_ready_queue_storage[DISPLAY_PV_FIFO_DEPTH *
                                      sizeof(display_pv_queue_entry_t *)];
static uint8_t pv_free_queue_storage[DISPLAY_PV_FIFO_DEPTH *
                                     sizeof(display_pv_queue_entry_t *)];
static uint8_t pv_payload_free_queue_storage[
    DISPLAY_PV_PAYLOAD_BLOCK_COUNT * sizeof(uint8_t *)];
static uint32_t pv_accepted_sequence;
static uint32_t pv_accepted_status = DISPLAY_PV_STATUS_FAILED;
static uint32_t pv_completed_sequence;
static uint32_t pv_completed_status;
static uint32_t pv_fifo_errors;
static uint8_t *guest_memory_host;
static uint32_t guest_memory_base;
static size_t guest_memory_size;
static uint32_t pv_shared_command_address;
static uint8_t tile_font[DISPLAY_TILE_FONT_SIZE];
static uint16_t tile_width;
static uint16_t tile_height;
static uint16_t tile_count;
static uint16_t tile_bytes;
static bool tile_font_valid;
static uint8_t tile_blank[DISPLAY_TILE_MAX_COUNT];
static bool tile_cursor_active;
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
#else
#define DISPLAY_PERF_ADD(field, value) do { } while (0)
#define display_perf_update_fifo_high_water(depth) do { } while (0)
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

static bool IRAM_ATTR display_vsync_callback(void *context) {
  BaseType_t higher_priority_task_woken = pdFALSE;
  TaskHandle_t task = display_task_handle;

  (void)context;
  if (task != NULL)
    xTaskNotifyFromISR(task, LCD_NOTIFY_VSYNC, eSetBits,
                      &higher_priority_task_woken);
  return higher_priority_task_woken == pdTRUE;
}

static bool display_pv_queue_init(void) {
  pv_ready_queue = xQueueCreateStatic(
      DISPLAY_PV_FIFO_DEPTH,
      sizeof(display_pv_queue_entry_t *), pv_ready_queue_storage,
      &pv_ready_queue_control);
  pv_free_queue = xQueueCreateStatic(
      DISPLAY_PV_FIFO_DEPTH,
      sizeof(display_pv_queue_entry_t *), pv_free_queue_storage,
      &pv_free_queue_control);
  pv_payload_free_queue = xQueueCreateStatic(
      DISPLAY_PV_PAYLOAD_BLOCK_COUNT, sizeof(uint8_t *),
      pv_payload_free_queue_storage, &pv_payload_free_queue_control);
  if (pv_ready_queue == NULL || pv_free_queue == NULL ||
      pv_payload_free_queue == NULL)
    return false;

  for (uint32_t row = 0; row < DISPLAY_FB_HEIGHT; row++)
    accel_dirty_start[row] = DISPLAY_FB_WIDTH;
  for (uint32_t index = 0; index < DISPLAY_PV_FIFO_DEPTH; index++) {
    display_pv_queue_entry_t *entry = &pv_queue_entries[index];

    entry->payload = NULL;
    entry->payload_length = 0;
    if (xQueueSend(pv_free_queue, &entry, 0) != pdTRUE)
      return false;
  }
  for (uint32_t index = 0; index < DISPLAY_PV_PAYLOAD_BLOCK_COUNT; index++) {
    uint8_t *payload = pv_payload_blocks[index];

    if (xQueueSend(pv_payload_free_queue, &payload, 0) != pdTRUE)
      return false;
  }
  return true;
}

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

static int display_hardware_init(void) {
  display_backend_surface_t surface = { 0 };
  esp_err_t err;

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
  framebuffer = surface.framebuffer;

  memset(framebuffer, 0, DISPLAY_FB_SIZE);
  err = display_cache_clean(framebuffer, DISPLAY_FB_SIZE);
  if (err != ESP_OK)
    goto fail;

  err = display_ppa_init();
  if (err != ESP_OK)
    printf("WARNING: PPA display acceleration unavailable: %s\n",
           esp_err_to_name(err));

  err = display_backend_start();
  if (err != ESP_OK)
    goto fail;

  printf("%s ready: %ux%u packed RGB565 at %u Hz, "
         "framebuffer %p mapped at guest 0x%08x (%u bytes), "
         "VSYNC cache-clean delay %d ms, async FIFO %u commands + "
         "%u x %u-byte payloads, "
         "service on CPU%d\n",
         surface.name != NULL ? surface.name : "Display backend",
         DISPLAY_FB_WIDTH, DISPLAY_FB_HEIGHT,
         (unsigned int)surface.refresh_hz, framebuffer,
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

  esp_err_t err = display_cache_clean(framebuffer + start, end - start);

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
  return true;
}

static bool display_flush_accel_dirty(void) {
  size_t run_start = 0;
  size_t run_end = 0;
  bool have_run = false;
  esp_err_t err = ESP_OK;

  if (accel_dirty_first >= DISPLAY_FB_HEIGHT)
    return true;

  for (uint32_t row = accel_dirty_first; row <= accel_dirty_last; row++) {
    uint32_t first = accel_dirty_start[row];
    uint32_t last = accel_dirty_end[row];
    size_t start;
    size_t end;

    if (first >= last)
      continue;
    start = ((size_t)row * DISPLAY_FB_WIDTH + first) * sizeof(uint16_t);
    end = ((size_t)row * DISPLAY_FB_WIDTH + last) * sizeof(uint16_t);
    if (have_run && start <= run_end + DISPLAY_CACHE_MSYNC_MERGE_GAP) {
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
    have_run = true;
  }
  if (have_run)
    err = display_cache_clean(framebuffer + run_start, run_end - run_start);
  if (err != ESP_OK)
    return false;

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

static bool display_rect_valid(uint32_t x, uint32_t y, uint32_t width,
                               uint32_t height) {
  return width != 0 && height != 0 && x < DISPLAY_FB_WIDTH &&
         y < DISPLAY_FB_HEIGHT && width <= DISPLAY_FB_WIDTH - x &&
         height <= DISPLAY_FB_HEIGHT - y;
}

static bool display_ppa_fill_rect(uint32_t x, uint32_t y, uint32_t width,
                                  uint32_t height, uint16_t color) {
  esp_err_t err;
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
      .fill_color_val = color,
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
  if (err == ESP_OK)
    ppa_fill_touched_scanout = true;
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

static bool display_tile_blit(const display_pv_command_t *command,
                              const uint8_t *payload,
                              uint32_t payload_length) {
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
  bool background_prefilled = false;

  if (!__atomic_load_n(&tile_font_valid, __ATOMIC_ACQUIRE) ||
      payload == NULL || width == 0 || height == 0 ||
      width > UINT32_MAX / height || count != width * height ||
      count > UINT32_MAX / sizeof(uint32_t) ||
      payload_length != count * sizeof(uint32_t) ||
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
    uint32_t index;

    memcpy(&index, payload + tile * sizeof(index), sizeof(index));
    if (index >= tile_count)
      return false;
    if (display_tile_is_blank(index))
      blank_count++;
  }
  /* Large text updates are usually mostly spaces. Fill their background once
   * with PPA, then expand only visible glyphs on CPU1. This preserves the
   * standard tileblit contract while avoiding thousands of identical stores. */
  if (blank_count != 0 && blank_count * 2u >= count)
    background_prefilled = display_ppa_fill_rect(
        pixel_x, pixel_y, pixel_width, pixel_height, background);
  for (uint32_t tile = 0; tile < count; tile++) {
    uint32_t index;

    memcpy(&index, payload + tile * sizeof(index), sizeof(index));
    if (background_prefilled && display_tile_is_blank(index))
      continue;
    if (!display_draw_tile(pixel_x + (tile % width) * tile_width,
                           pixel_y + (tile / width) * tile_height, index,
                           foreground, background))
      return false;
    if (background_prefilled)
      display_mark_accel_rect(
          pixel_x + (tile % width) * tile_width,
          pixel_y + (tile / width) * tile_height,
          tile_width, tile_height);
  }
  if (!background_prefilled)
    display_mark_accel_rect(pixel_x, pixel_y, pixel_width, pixel_height);
  return true;
}

static void display_toggle_tile_cursor(void) {
  uint32_t start_row;
  uint32_t pixel_x;
  uint32_t pixel_y;
  uint32_t cursor_height;

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
  for (uint32_t row = 0; row < cursor_height; row++) {
    uint16_t *destination = (uint16_t *)(framebuffer +
        ((size_t)(pixel_y + row) * DISPLAY_FB_WIDTH + pixel_x) *
            sizeof(uint16_t));

    for (uint32_t column = 0; column < tile_width; column++)
      destination[column] ^= tile_cursor_color;
  }
  display_mark_accel_rect(pixel_x, pixel_y, tile_width, cursor_height);
}

static bool display_tile_cursor(const display_pv_command_t *command) {
  uint32_t x = command->args[0];
  uint32_t y = command->args[1];
  uint32_t mode = command->args[2];
  uint32_t shape = command->args[3];
  uint16_t foreground = (uint16_t)command->args[4];
  uint16_t background = (uint16_t)command->args[5];

  if (!__atomic_load_n(&tile_font_valid, __ATOMIC_ACQUIRE) || mode > 1u ||
      shape > 5u || x > UINT32_MAX / tile_width ||
      y > UINT32_MAX / tile_height ||
      x * tile_width >= DISPLAY_FB_WIDTH ||
      y * tile_height >= DISPLAY_FB_HEIGHT)
    return false;
  if (tile_cursor_active)
    display_toggle_tile_cursor();
  tile_cursor_active = mode != 0 && shape != 0;
  tile_cursor_x = x;
  tile_cursor_y = y;
  tile_cursor_shape = shape;
  tile_cursor_color = foreground ^ background;
  if (tile_cursor_color == 0)
    tile_cursor_color = UINT16_MAX;
  if (tile_cursor_active)
    display_toggle_tile_cursor();
  return true;
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

static bool display_pv_payload_owned(const uint8_t *payload) {
  uintptr_t start = (uintptr_t)&pv_payload_blocks[0][0];
  uintptr_t end = start + sizeof(pv_payload_blocks);
  uintptr_t address = (uintptr_t)payload;

  return address >= start && address < end &&
         (address - start) % DISPLAY_PV_PAYLOAD_SIZE == 0;
}

static void display_pv_record_fifo_error(void) {
  (void)__atomic_add_fetch(&pv_fifo_errors, 1u, __ATOMIC_RELAXED);
}

static uint32_t display_pv_report_busy(void) {
  DISPLAY_PERF_ADD(fifo_busy, 1u);
  return DISPLAY_PV_STATUS_BUSY;
}

static void display_pv_release_entry(display_pv_queue_entry_t *entry) {
  uint8_t *payload = entry->payload;

  entry->payload = NULL;
  entry->payload_length = 0;
  if (payload != NULL) {
    if (!display_pv_payload_owned(payload) ||
        xQueueSend(pv_payload_free_queue, &payload, 0) != pdTRUE)
      display_pv_record_fifo_error();
  }
  if (xQueueSend(pv_free_queue, &entry, 0) != pdTRUE)
    display_pv_record_fifo_error();
}

static void display_process_pv_commands(void) {
  display_pv_queue_entry_t *entry;

  while (xQueueReceive(pv_ready_queue, &entry, 0) == pdTRUE) {
    const display_pv_command_t *command = &entry->command;
    bool restore_cursor = false;
    bool success = false;

    ppa_accel_active = false;
    if (tile_cursor_active && command->operation != DISPLAY_PV_OP_TILE_CURSOR) {
      /* The cursor is an XOR overlay. Remove it before changing the pixels
       * below it, then restore it without losing the guest's blink state. */
      display_toggle_tile_cursor();
      restore_cursor = true;
    }
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
      success = display_image1(command, entry->payload,
                               entry->payload_length);
      break;
    case DISPLAY_PV_OP_SET_TILE:
      DISPLAY_PERF_ADD(tile_commands, 1u);
      success = display_set_tile(command, entry->payload,
                                 entry->payload_length);
      break;
    case DISPLAY_PV_OP_TILE_FILL:
      DISPLAY_PERF_ADD(tile_commands, 1u);
      success = display_tile_fill(command);
      break;
    case DISPLAY_PV_OP_TILE_BLIT:
      DISPLAY_PERF_ADD(tile_commands, 1u);
      success = display_tile_blit(command, entry->payload,
                                  entry->payload_length);
      break;
    case DISPLAY_PV_OP_TILE_CURSOR:
      DISPLAY_PERF_ADD(tile_commands, 1u);
      success = display_tile_cursor(command);
      break;
    default:
      break;
    }
    if (restore_cursor && tile_cursor_active)
      display_toggle_tile_cursor();

    /* Large text batches may use PPA for the background and CPU1 for glyphs.
     * PPA writes are DMA-visible immediately, so clean the glyph rows now as
     * one finished composite instead of exposing a blank intermediate frame. */
    if (ppa_fill_touched_scanout) {
      if (display_flush_accel_dirty())
        ppa_fill_touched_scanout = false;
    }
    DISPLAY_PERF_ADD(commands, 1u);

    portENTER_CRITICAL(&dirty_lock);
    pv_completed_status =
        success ? DISPLAY_PV_STATUS_OK : DISPLAY_PV_STATUS_INVALID;
    pv_completed_sequence = command->sequence;
    portEXIT_CRITICAL(&dirty_lock);
    display_pv_release_entry(entry);
  }
}

static void display_process_ppa_command(void) {
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
    return;
  }
  if (!frame_pending || !ppa_ready)
    return;

  if (!ppa_accel_active || source_width != ppa_active_width ||
      source_height != ppa_active_height) {
    memset(framebuffer, 0, DISPLAY_FB_SIZE);
    err = display_cache_clean(framebuffer, DISPLAY_FB_SIZE);
    if (err != ESP_OK)
      return;
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
  if (scale_sixteenths == 0)
    return;
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
    error_reported = false;
  }
}

static void display_service_task(void *argument) {
  TaskHandle_t init_waiter = (TaskHandle_t)argument;

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

    xTaskNotifyWait(0, UINT32_MAX, &events, portMAX_DELAY);
#if CONFIG_RV32_HOST_PERF_STATS
    int64_t service_started = esp_timer_get_time();
#endif
    DISPLAY_PERF_ADD(service_wakes, 1u);
    /* Execute every queued native operation on CPU1.  Linux has already
     * returned from its fbdev callback, and all resulting cache writeback is
     * still coalesced below at the physical frame boundary. */
    if ((events & LCD_NOTIFY_PV_COMMAND) ||
        uxQueueMessagesWaiting(pv_ready_queue) != 0)
      display_process_pv_commands();
    if (events & LCD_NOTIFY_VSYNC) {
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
      if (!display_flush_accel_dirty()) {
        frame_ready = false;
      } else {
        ppa_fill_touched_scanout = false;
      }
      display_process_ppa_command();
      if (frame_ready) {
        portENTER_CRITICAL(&dirty_lock);
        frame_sync_completed = sync_target;
        portEXIT_CRITICAL(&dirty_lock);
      }
    }
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

void display_bridge_commit(void) {
  if (!producer_dirty)
    return;

  portENTER_CRITICAL(&dirty_lock);
  if (producer_dirty_start < pending_dirty_start)
    pending_dirty_start = producer_dirty_start;
  if (producer_dirty_end > pending_dirty_end)
    pending_dirty_end = producer_dirty_end;
  pending_dirty = true;
  portEXIT_CRITICAL(&dirty_lock);

  producer_dirty = false;
  producer_dirty_start = DISPLAY_FB_SIZE;
  producer_dirty_end = 0;
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
        synthetic |= DISPLAY_PV_FEATURE_SHARED_COMMAND;
      if (pv_ready_queue != NULL && pv_free_queue != NULL) {
        synthetic |= DISPLAY_PV_FEATURE_ASYNC_FIFO;
        if (ppa_staging != NULL && pv_payload_free_queue != NULL)
          synthetic |= DISPLAY_PV_FEATURE_IMAGE1 | DISPLAY_PV_FEATURE_TILE |
                       DISPLAY_PV_FEATURE_PAYLOAD_POOL;
      }
      break;
    case DISPLAY_PV_REG_COMPLETED:
      portENTER_CRITICAL(&dirty_lock);
      synthetic = pv_completed_sequence;
      portEXIT_CRITICAL(&dirty_lock);
      break;
    case DISPLAY_PV_REG_STATUS:
      portENTER_CRITICAL(&dirty_lock);
      synthetic = pv_completed_status;
      portEXIT_CRITICAL(&dirty_lock);
      break;
    case DISPLAY_PV_REG_ACCEPTED:
      portENTER_CRITICAL(&dirty_lock);
      synthetic = pv_accepted_sequence;
      portEXIT_CRITICAL(&dirty_lock);
      break;
    case DISPLAY_PV_REG_ACCEPT_STATUS:
      portENTER_CRITICAL(&dirty_lock);
      synthetic = pv_accepted_status;
      portEXIT_CRITICAL(&dirty_lock);
      break;
    case DISPLAY_PV_REG_QUEUE_DEPTH:
      synthetic = DISPLAY_PV_FIFO_DEPTH;
      break;
    case DISPLAY_PV_REG_QUEUE_FREE:
      if (pv_free_queue != NULL)
        synthetic = uxQueueMessagesWaiting(pv_free_queue);
      break;
    case DISPLAY_PV_REG_PAYLOAD_LIMIT:
      synthetic = DISPLAY_PV_PAYLOAD_SIZE;
      break;
    case DISPLAY_PV_REG_PAYLOAD_BLOCKS:
      synthetic = DISPLAY_PV_PAYLOAD_BLOCK_COUNT;
      break;
    case DISPLAY_PV_REG_PAYLOAD_FREE:
      if (pv_payload_free_queue != NULL)
        synthetic = uxQueueMessagesWaiting(pv_payload_free_queue);
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

  if (display_task_handle == NULL || pv_ready_queue == NULL ||
      pv_free_queue == NULL || command->sequence == 0)
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
  case DISPLAY_PV_OP_TILE_CURSOR:
    if (command->args[2] > 1u || command->args[3] > 5u)
      return DISPLAY_PV_STATUS_INVALID;
    break;
  default:
    return DISPLAY_PV_STATUS_INVALID;
  }

  if (payload_length > DISPLAY_PV_PAYLOAD_SIZE ||
      payload_length > DISPLAY_ACCEL_STAGE_SIZE ||
      (payload_length != 0 &&
       (ppa_staging == NULL || pv_payload_free_queue == NULL)))
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

  if (xQueueReceive(pv_free_queue, &entry, 0) != pdTRUE)
    return display_pv_report_busy();
  entry->payload = NULL;
  entry->payload_length = 0;
  if (payload_length != 0 &&
      xQueueReceive(pv_payload_free_queue, &payload, 0) != pdTRUE) {
    if (xQueueSend(pv_free_queue, &entry, 0) != pdTRUE)
      display_pv_record_fifo_error();
    return display_pv_report_busy();
  }
  entry->command = *command;
  entry->payload_length = payload_length;
  entry->payload = payload;
  if (payload != NULL)
    memcpy(payload, payload_source, payload_length);
  if (xQueueSend(pv_ready_queue, &entry, 0) != pdTRUE) {
    display_pv_record_fifo_error();
    display_pv_release_entry(entry);
    return display_pv_report_busy();
  }
  display_perf_update_fifo_high_water(
      uxQueueMessagesWaiting(pv_ready_queue));
  if (xTaskNotify(display_task_handle, LCD_NOTIFY_PV_COMMAND, eSetBits) !=
      pdPASS)
    display_pv_record_fifo_error();
  return DISPLAY_PV_STATUS_OK;
}

static uint32_t display_pv_submit_current(uint32_t *accepted_sequence) {
  display_pv_command_t command = pv_registers;
  uint32_t shared_address = pv_shared_command_address;
  uint32_t payload_address = 0;
  uint32_t payload_length = 0;
  bool shared = false;

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
    if (shared_command.magic != DISPLAY_PV_SHARED_COMMAND_MAGIC) {
      *accepted_sequence = shared_command.sequence;
      return DISPLAY_PV_STATUS_INVALID;
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
      uint32_t status;

      if (value != DISPLAY_PV_SUBMIT)
        break;
      status = display_pv_submit_current(&accepted_sequence);

      portENTER_CRITICAL(&dirty_lock);
      pv_accepted_status = status;
      pv_accepted_sequence = accepted_sequence;
      portEXIT_CRITICAL(&dirty_lock);
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

  if (offset < producer_dirty_start)
    producer_dirty_start = offset;
  if (offset + width > producer_dirty_end)
    producer_dirty_end = offset + width;
  producer_dirty = true;
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
  DISPLAY_PERF_TAKE(commands);
  DISPLAY_PERF_TAKE(fill_commands);
  DISPLAY_PERF_TAKE(copy_commands);
  DISPLAY_PERF_TAKE(tile_commands);
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
  DISPLAY_PERF_TAKE(fifo_busy);
  DISPLAY_PERF_TAKE(fifo_high_water);
#undef DISPLAY_PERF_TAKE
#endif
}
