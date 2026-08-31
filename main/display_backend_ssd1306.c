/* SSD1306 I2C implementation of the generic RGB565 display backend. */

#include "display_backend.h"

#include "display_bridge.h"
#include "driver/i2c_master.h"
#include "esp_heap_caps.h"
#include "esp_lcd_io_i2c.h"
#include "esp_lcd_panel_io.h"
#include "esp_lcd_panel_ops.h"
#include "esp_lcd_panel_ssd1306.h"
#include "esp_log.h"
#include "esp_private/esp_cache_private.h"
#include "esp_timer.h"

#include <string.h>

#define SSD1306_WIDTH 128u
#define SSD1306_PAGE_HEIGHT 8u
#define SSD1306_BUFFER_SIZE (SSD1306_WIDTH * DISPLAY_FB_HEIGHT / 8u)
#define SSD1306_TRANSFER_TIMEOUT_MS 1000

static const char *TAG = "rv32-ssd1306";

_Static_assert(DISPLAY_FB_WIDTH == SSD1306_WIDTH,
               "SSD1306 backend requires a 128-pixel surface");
_Static_assert(DISPLAY_FB_HEIGHT == 32u || DISPLAY_FB_HEIGHT == 64u,
               "SSD1306 backend requires a 128x32 or 128x64 surface");
_Static_assert(DISPLAY_FB_STRIDE == SSD1306_WIDTH * sizeof(uint16_t),
               "SSD1306 backend requires packed RGB565 input");

static i2c_master_bus_handle_t i2c_bus;
static esp_lcd_panel_io_handle_t panel_io;
static esp_lcd_panel_handle_t panel;
static esp_timer_handle_t refresh_timer;
static display_backend_vsync_callback_t backend_vsync_callback;
static void *backend_vsync_context;
static uint8_t *rgb565_framebuffer;
static uint8_t oled_pixels[SSD1306_BUFFER_SIZE];
static uint8_t previous_pixels[SSD1306_BUFFER_SIZE];
static bool previous_pixels_valid;
static bool frame_transfer_pending = true;

static void display_backend_refresh_timer(void *context) {
  (void)context;
  if (backend_vsync_callback != NULL)
    (void)backend_vsync_callback(backend_vsync_context, false);
}

static bool display_backend_pixel_is_on(uint16_t pixel) {
  /* Preserve colored terminal text on a monochrome panel. Requiring roughly
   * 20% intensity in any channel rejects black and very dark antialiasing
   * without making pure red or blue disappear through a luma threshold. */
  return ((pixel >> 11) & 0x1fu) >= 6u ||
         ((pixel >> 5) & 0x3fu) >= 12u || (pixel & 0x1fu) >= 6u;
}

static void display_backend_convert_frame(void) {
  const uint16_t *source = (const uint16_t *)rgb565_framebuffer;

  memset(oled_pixels, 0, sizeof(oled_pixels));
  for (uint32_t y = 0; y < DISPLAY_FB_HEIGHT; y++) {
    uint8_t bit = (uint8_t)(1u << (y % SSD1306_PAGE_HEIGHT));
    uint8_t *page = oled_pixels + (y / SSD1306_PAGE_HEIGHT) * SSD1306_WIDTH;

    for (uint32_t x = 0; x < SSD1306_WIDTH; x++) {
      if (display_backend_pixel_is_on(source[y * SSD1306_WIDTH + x]))
        page[x] |= bit;
    }
  }
}

static esp_err_t display_backend_recover(void) {
  esp_err_t err;

  /* A transient NACK can leave either the controller or the ESP-IDF I2C
   * master state out of sync. Reset the bus, then replay the same panel
   * configuration sequence used during boot. */
  err = i2c_master_bus_reset(i2c_bus);
  if (err != ESP_OK)
    return err;
  err = esp_lcd_panel_reset(panel);
  if (err != ESP_OK)
    return err;
  err = esp_lcd_panel_init(panel);
  if (err != ESP_OK)
    return err;
  err = esp_lcd_panel_disp_on_off(panel, true);
  if (err != ESP_OK)
    return err;

  previous_pixels_valid = false;
  frame_transfer_pending = true;
  ESP_LOGW(TAG, "I2C transfer recovered; SSD1306 configuration replayed");
  return ESP_OK;
}

esp_err_t display_backend_init(display_backend_vsync_callback_t vsync_callback,
                               void *vsync_context,
                               display_backend_surface_t *surface) {
  esp_err_t err;
  size_t framebuffer_alignment;

  if (vsync_callback == NULL || surface == NULL)
    return ESP_ERR_INVALID_ARG;

  const i2c_master_bus_config_t bus_config = {
      .i2c_port = CONFIG_RV32_SSD1306_I2C_PORT,
      .sda_io_num = CONFIG_RV32_SSD1306_SDA_GPIO,
      .scl_io_num = CONFIG_RV32_SSD1306_SCL_GPIO,
      .clk_source = I2C_CLK_SRC_DEFAULT,
      .glitch_ignore_cnt = 7,
      .flags.enable_internal_pullup = CONFIG_RV32_SSD1306_INTERNAL_PULLUPS,
  };
  err = i2c_new_master_bus(&bus_config, &i2c_bus);
  if (err != ESP_OK)
    return err;

  const esp_lcd_panel_io_i2c_config_t io_config = {
      .dev_addr = CONFIG_RV32_SSD1306_I2C_ADDRESS,
      .scl_speed_hz = CONFIG_RV32_SSD1306_I2C_CLOCK_HZ,
      .control_phase_bytes = 1,
      .dc_bit_offset = 6,
      .lcd_cmd_bits = 8,
      .lcd_param_bits = 8,
      .transaction_timeout_ms = SSD1306_TRANSFER_TIMEOUT_MS,
  };
  err = esp_lcd_new_panel_io_i2c(i2c_bus, &io_config, &panel_io);
  if (err != ESP_OK)
    return err;

  const esp_lcd_panel_ssd1306_config_t vendor_config = {
      .height = DISPLAY_FB_HEIGHT,
  };
  const esp_lcd_panel_dev_config_t panel_config = {
      .reset_gpio_num = CONFIG_RV32_SSD1306_RESET_GPIO,
      .bits_per_pixel = 1,
      .vendor_config = (void *)&vendor_config,
  };
  err = esp_lcd_new_panel_ssd1306(panel_io, &panel_config, &panel);
  if (err != ESP_OK)
    return err;
  err = esp_lcd_panel_reset(panel);
  if (err != ESP_OK)
    return err;
  err = esp_lcd_panel_init(panel);
  if (err != ESP_OK)
    return err;

  err = esp_cache_get_alignment(MALLOC_CAP_SPIRAM, &framebuffer_alignment);
  if (err != ESP_OK)
    return err;
  if (framebuffer_alignment == 0u)
    return ESP_ERR_INVALID_SIZE;
  rgb565_framebuffer = heap_caps_aligned_calloc(
      framebuffer_alignment, 1, DISPLAY_FB_SIZE,
      MALLOC_CAP_SPIRAM | MALLOC_CAP_8BIT);
  if (rgb565_framebuffer == NULL)
    return ESP_ERR_NO_MEM;

  backend_vsync_callback = vsync_callback;
  backend_vsync_context = vsync_context;
  const esp_timer_create_args_t timer_config = {
      .callback = display_backend_refresh_timer,
      .dispatch_method = ESP_TIMER_TASK,
      .name = "ssd1306_refresh",
      .skip_unhandled_events = true,
  };
  err = esp_timer_create(&timer_config, &refresh_timer);
  if (err != ESP_OK)
    return err;

  *surface = (display_backend_surface_t){
      .framebuffer = rgb565_framebuffer,
      .framebuffer_size = DISPLAY_FB_SIZE,
      .width = DISPLAY_FB_WIDTH,
      .height = DISPLAY_FB_HEIGHT,
      .stride = DISPLAY_FB_STRIDE,
      .format = DISPLAY_BACKEND_FORMAT_RGB565,
      .refresh_hz = CONFIG_RV32_SSD1306_REFRESH_HZ,
      .name = "SSD1306 I2C OLED",
  };
  return ESP_OK;
}

esp_err_t display_backend_start(void) {
  esp_err_t err;

  if (panel == NULL || refresh_timer == NULL)
    return ESP_ERR_INVALID_STATE;
  err = esp_lcd_panel_disp_on_off(panel, true);
  if (err != ESP_OK)
    return err;
  err = display_backend_present(true);
  if (err != ESP_OK)
    return err;
  return esp_timer_start_periodic(
      refresh_timer, 1000000u / CONFIG_RV32_SSD1306_REFRESH_HZ);
}

esp_err_t display_backend_present(bool frame_changed) {
  esp_err_t err;

  if (panel == NULL || rgb565_framebuffer == NULL)
    return ESP_ERR_INVALID_STATE;
  if (!frame_changed && !frame_transfer_pending)
    return ESP_OK;
  frame_transfer_pending = true;
  display_backend_convert_frame();
  if (previous_pixels_valid &&
      memcmp(oled_pixels, previous_pixels, sizeof(oled_pixels)) == 0) {
    frame_transfer_pending = false;
    return ESP_OK;
  }

  err = esp_lcd_panel_draw_bitmap(panel, 0, 0, SSD1306_WIDTH,
                                  DISPLAY_FB_HEIGHT, oled_pixels);
  if (err != ESP_OK) {
    previous_pixels_valid = false;
    esp_err_t recovery_err = display_backend_recover();

    if (recovery_err != ESP_OK)
      return recovery_err;
    err = esp_lcd_panel_draw_bitmap(panel, 0, 0, SSD1306_WIDTH,
                                    DISPLAY_FB_HEIGHT, oled_pixels);
  }
  if (err == ESP_OK) {
    memcpy(previous_pixels, oled_pixels, sizeof(previous_pixels));
    previous_pixels_valid = true;
    frame_transfer_pending = false;
  }
  return err;
}
