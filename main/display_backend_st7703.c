/* Waveshare 720x720 ST7703 implementation of the generic display backend. */

#include "display_backend.h"

#include "display_bridge.h"
#include "driver/gpio.h"
#include "driver/ledc.h"
#include "esp_attr.h"
#include "esp_lcd_mipi_dsi.h"
#include "esp_lcd_panel_ops.h"
#include "esp_lcd_st7703.h"
#include "esp_ldo_regulator.h"

#define LCD_RESET_GPIO GPIO_NUM_27
#define LCD_BACKLIGHT_GPIO GPIO_NUM_26
#define LCD_BACKLIGHT_ENABLE GPIO_NUM_33
#define LCD_BACKLIGHT_TIMER LEDC_TIMER_1
#define LCD_BACKLIGHT_CHAN LEDC_CHANNEL_1
#define LCD_DSI_PHY_LDO_CHAN 3
#define LCD_PANEL_LDO_CHAN 4
#define LCD_HORIZONTAL_TOTAL (720u + 50u + 20u + 50u)
#define LCD_VERTICAL_TOTAL (720u + 20u + 4u + 20u)
#define LCD_DPI_CLOCK_MHZ                                                      \
  ((float)CONFIG_RV32_ST7703_REFRESH_HZ * (float)LCD_HORIZONTAL_TOTAL *        \
   (float)LCD_VERTICAL_TOTAL / 1000000.0f)

_Static_assert(DISPLAY_FB_WIDTH == 720u && DISPLAY_FB_HEIGHT == 720u,
               "ST7703 backend requires a 720x720 display surface");
_Static_assert(DISPLAY_FB_STRIDE == 1440u,
               "ST7703 backend requires packed RGB565 scanout");

static esp_ldo_channel_handle_t dsi_phy_ldo;
static esp_ldo_channel_handle_t panel_ldo;
static esp_lcd_dsi_bus_handle_t dsi_bus;
static esp_lcd_panel_io_handle_t panel_io;
static esp_lcd_panel_handle_t panel;
static display_backend_vsync_callback_t backend_vsync_callback;
static void *backend_vsync_context;

static bool IRAM_ATTR display_backend_vsync(
    esp_lcd_panel_handle_t callback_panel,
    esp_lcd_dpi_panel_event_data_t *event_data, void *user_context) {
  (void)callback_panel;
  (void)event_data;
  (void)user_context;

  if (backend_vsync_callback == NULL)
    return false;
  return backend_vsync_callback(backend_vsync_context, true);
}

static esp_err_t display_backend_enable_power(void) {
  const esp_ldo_channel_config_t dsi_ldo_config = {
      .chan_id = LCD_DSI_PHY_LDO_CHAN,
      .voltage_mv = 2500,
  };
  const esp_ldo_channel_config_t panel_ldo_config = {
      .chan_id = LCD_PANEL_LDO_CHAN,
      .voltage_mv = 3300,
  };
  esp_err_t err = esp_ldo_acquire_channel(&dsi_ldo_config, &dsi_phy_ldo);

  if (err != ESP_OK)
    return err;
  return esp_ldo_acquire_channel(&panel_ldo_config, &panel_ldo);
}

static esp_err_t display_backend_enable_backlight(void) {
  const gpio_config_t enable_config = {
      .pin_bit_mask = 1ULL << LCD_BACKLIGHT_ENABLE,
      .mode = GPIO_MODE_OUTPUT,
      .pull_up_en = GPIO_PULLUP_DISABLE,
      .pull_down_en = GPIO_PULLDOWN_DISABLE,
      .intr_type = GPIO_INTR_DISABLE,
  };
  const ledc_timer_config_t timer_config = {
      .speed_mode = LEDC_LOW_SPEED_MODE,
      .duty_resolution = LEDC_TIMER_10_BIT,
      .timer_num = LCD_BACKLIGHT_TIMER,
      .freq_hz = 5000,
      .clk_cfg = LEDC_AUTO_CLK,
  };
  const ledc_channel_config_t channel_config = {
      .gpio_num = LCD_BACKLIGHT_GPIO,
      .speed_mode = LEDC_LOW_SPEED_MODE,
      .channel = LCD_BACKLIGHT_CHAN,
      .intr_type = LEDC_INTR_DISABLE,
      .timer_sel = LCD_BACKLIGHT_TIMER,
      .duty = 1023,
      .hpoint = 0,
      .flags.output_invert = 1,
  };
  esp_err_t err = gpio_config(&enable_config);

  if (err != ESP_OK)
    return err;
  err = gpio_set_level(LCD_BACKLIGHT_ENABLE, 1);
  if (err != ESP_OK)
    return err;
  err = ledc_timer_config(&timer_config);
  if (err != ESP_OK)
    return err;
  return ledc_channel_config(&channel_config);
}

esp_err_t display_backend_init(display_backend_vsync_callback_t vsync_callback,
                               void *vsync_context,
                               display_backend_surface_t *surface) {
  esp_err_t err;

  if (vsync_callback == NULL || surface == NULL)
    return ESP_ERR_INVALID_ARG;
  err = display_backend_enable_power();
  if (err != ESP_OK)
    return err;
  err = display_backend_enable_backlight();
  if (err != ESP_OK)
    return err;

  const esp_lcd_dsi_bus_config_t bus_config = {
      .bus_id = 0,
      .num_data_lanes = 2,
      .phy_clk_src = 0,
      .lane_bit_rate_mbps = 480,
  };
  err = esp_lcd_new_dsi_bus(&bus_config, &dsi_bus);
  if (err != ESP_OK)
    return err;

  const esp_lcd_dbi_io_config_t io_config = {
      .virtual_channel = 0,
      .lcd_cmd_bits = 8,
      .lcd_param_bits = 8,
  };
  err = esp_lcd_new_panel_io_dbi(dsi_bus, &io_config, &panel_io);
  if (err != ESP_OK)
    return err;

  esp_lcd_dpi_panel_config_t dpi_config =
      ST7703_720_720_PANEL_60HZ_DPI_CONFIG(LCD_COLOR_PIXEL_FORMAT_RGB565);
  dpi_config.dpi_clock_freq_mhz = LCD_DPI_CLOCK_MHZ;
  dpi_config.num_fbs = 1;

  st7703_vendor_config_t vendor_config = {
      .flags =
          {
              .use_mipi_interface = 1,
          },
      .mipi_config =
          {
              .dsi_bus = dsi_bus,
              .dpi_config = &dpi_config,
          },
  };
  const esp_lcd_panel_dev_config_t panel_config = {
      .reset_gpio_num = LCD_RESET_GPIO,
      .rgb_ele_order = LCD_RGB_ELEMENT_ORDER_RGB,
      .bits_per_pixel = 16,
      .vendor_config = &vendor_config,
  };
  err = esp_lcd_new_panel_st7703(panel_io, &panel_config, &panel);
  if (err != ESP_OK)
    return err;
  err = esp_lcd_panel_reset(panel);
  if (err != ESP_OK)
    return err;
  err = esp_lcd_panel_init(panel);
  if (err != ESP_OK)
    return err;

  backend_vsync_callback = vsync_callback;
  backend_vsync_context = vsync_context;
  const esp_lcd_dpi_panel_event_callbacks_t callbacks = {
      .on_vsync = display_backend_vsync,
  };
  err = esp_lcd_dpi_panel_register_event_callbacks(panel, &callbacks, NULL);
  if (err != ESP_OK)
    return err;

  uint8_t *framebuffer;
  err = esp_lcd_dpi_panel_get_frame_buffer(panel, 1, (void **)&framebuffer);
  if (err != ESP_OK || framebuffer == NULL)
    return err == ESP_OK ? ESP_ERR_INVALID_STATE : err;

  *surface = (display_backend_surface_t){
      .framebuffer = framebuffer,
      .framebuffer_size = DISPLAY_FB_SIZE,
      .width = DISPLAY_FB_WIDTH,
      .height = DISPLAY_FB_HEIGHT,
      .stride = DISPLAY_FB_STRIDE,
      .format = DISPLAY_BACKEND_FORMAT_RGB565,
      .refresh_hz = CONFIG_RV32_ST7703_REFRESH_HZ,
      .name = "Waveshare ST7703 MIPI-DSI",
  };
  return ESP_OK;
}

esp_err_t display_backend_start(void) {
  if (panel == NULL)
    return ESP_ERR_INVALID_STATE;
  return esp_lcd_panel_disp_on_off(panel, true);
}

esp_err_t display_backend_present(bool frame_changed) {
  (void)frame_changed;
  return ESP_OK;
}
