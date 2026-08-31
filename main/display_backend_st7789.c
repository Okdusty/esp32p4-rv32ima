/* Generic 240x240 ST7789 SPI implementation of the RGB565 display backend. */

#include "display_backend.h"

#include "display_bridge.h"
#include "driver/gpio.h"
#include "driver/spi_master.h"
#include "esp_check.h"
#include "esp_heap_caps.h"
#include "esp_lcd_io_spi.h"
#include "esp_lcd_panel_commands.h"
#include "esp_lcd_panel_io.h"
#include "esp_lcd_panel_ops.h"
#include "esp_lcd_panel_st7789.h"
#include "esp_log.h"
#include "esp_private/esp_cache_private.h"
#include "esp_rom_sys.h"
#include "esp_timer.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"

#include <inttypes.h>
#include <string.h>

#define ST7789_WIDTH 240u
#define ST7789_HEIGHT 240u
#define ST7789_TRANSFER_QUEUE_DEPTH 2u
#define ST7789_GPIO_SETTLE_US 2u
#define ST7789_TEST_PATTERN_MS 1500u

#if defined(CONFIG_RV32_ST7789_SPI3) && CONFIG_RV32_ST7789_SPI3
#define ST7789_SPI_HOST SPI3_HOST
#define ST7789_SPI_HOST_NAME "SPI3"
#else
#define ST7789_SPI_HOST SPI2_HOST
#define ST7789_SPI_HOST_NAME "SPI2"
#endif

#if defined(CONFIG_RV32_ST7789_BGR) && CONFIG_RV32_ST7789_BGR
#define ST7789_ELEMENT_ORDER LCD_RGB_ELEMENT_ORDER_BGR
#else
#define ST7789_ELEMENT_ORDER LCD_RGB_ELEMENT_ORDER_RGB
#endif

#if defined(CONFIG_RV32_ST7789_BACKLIGHT_ACTIVE_HIGH) &&                   \
    CONFIG_RV32_ST7789_BACKLIGHT_ACTIVE_HIGH
#define ST7789_BACKLIGHT_ON 1
#define ST7789_BACKLIGHT_OFF 0
#else
#define ST7789_BACKLIGHT_ON 0
#define ST7789_BACKLIGHT_OFF 1
#endif

_Static_assert(DISPLAY_FB_WIDTH == ST7789_WIDTH &&
                   DISPLAY_FB_HEIGHT == ST7789_HEIGHT,
               "ST7789 backend requires a 240x240 display surface");
_Static_assert(DISPLAY_FB_STRIDE == ST7789_WIDTH * sizeof(uint16_t),
               "ST7789 backend requires packed RGB565 input");

static const char *TAG = "rv32-st7789";
static esp_lcd_panel_io_handle_t panel_io;
static esp_lcd_panel_handle_t panel;
static esp_timer_handle_t refresh_timer;
static display_backend_vsync_callback_t backend_vsync_callback;
static void *backend_vsync_context;
static uint8_t *rgb565_framebuffer;
static volatile uint32_t color_transfers_completed;
static uint32_t effective_refresh_hz;

typedef struct {
  int gpio;
  const char *name;
} st7789_pin_t;

static const st7789_pin_t st7789_pins[] = {
    {CONFIG_RV32_ST7789_SCLK_GPIO, "SCLK"},
    {CONFIG_RV32_ST7789_MOSI_GPIO, "MOSI"},
    {CONFIG_RV32_ST7789_DC_GPIO, "DC"},
    {CONFIG_RV32_ST7789_CS_GPIO, "CS"},
    {CONFIG_RV32_ST7789_RESET_GPIO, "RESET"},
    {CONFIG_RV32_ST7789_BACKLIGHT_GPIO, "BACKLIGHT"},
};

static bool display_backend_color_transfer_done(
    esp_lcd_panel_io_handle_t io, esp_lcd_panel_io_event_data_t *event,
    void *context) {
  (void)io;
  (void)event;
  (void)context;
  __atomic_add_fetch(&color_transfers_completed, 1u, __ATOMIC_RELEASE);
  return false;
}

static esp_err_t display_backend_validate_pins(void) {
  for (size_t index = 0; index < sizeof(st7789_pins) / sizeof(st7789_pins[0]);
       index++) {
    const st7789_pin_t *pin = &st7789_pins[index];

    if (pin->gpio < 0)
      continue;
    if (!GPIO_IS_VALID_OUTPUT_GPIO(pin->gpio)) {
      ESP_LOGE(TAG, "%s GPIO%d is not output-capable", pin->name, pin->gpio);
      return ESP_ERR_INVALID_ARG;
    }
    for (size_t previous = 0; previous < index; previous++) {
      if (st7789_pins[previous].gpio == pin->gpio) {
        ESP_LOGE(TAG, "%s and %s both use GPIO%d", st7789_pins[previous].name,
                 pin->name, pin->gpio);
        return ESP_ERR_INVALID_ARG;
      }
    }
  }
  return ESP_OK;
}

static esp_err_t display_backend_test_gpio(const st7789_pin_t *pin) {
  const gpio_config_t config = {
      .pin_bit_mask = 1ULL << pin->gpio,
      .mode = GPIO_MODE_INPUT_OUTPUT,
      .pull_up_en = GPIO_PULLUP_DISABLE,
      .pull_down_en = GPIO_PULLDOWN_DISABLE,
      .intr_type = GPIO_INTR_DISABLE,
  };
  esp_err_t err = gpio_config(&config);

  if (err != ESP_OK)
    return err;
  for (int level = 0; level <= 1; level++) {
    err = gpio_set_level(pin->gpio, level);
    if (err != ESP_OK)
      return err;
    esp_rom_delay_us(ST7789_GPIO_SETTLE_US);
    if (gpio_get_level(pin->gpio) != level) {
      ESP_LOGE(TAG, "%s GPIO%d is stuck at level %d", pin->name, pin->gpio,
               gpio_get_level(pin->gpio));
      return ESP_ERR_INVALID_RESPONSE;
    }
  }
  return ESP_OK;
}

static esp_err_t display_backend_qualify_gpio_pads(void) {
  esp_err_t err = display_backend_validate_pins();

  if (err != ESP_OK)
    return err;

  /* Hold the controller in reset while command and data pads are toggled.
   * This verifies the P4 pads and GPIO matrix, not continuity beyond the P4:
   * the write-only panel wiring has no return signal to test. */
#if CONFIG_RV32_ST7789_RESET_GPIO >= 0
  const gpio_config_t reset_config = {
      .pin_bit_mask = 1ULL << CONFIG_RV32_ST7789_RESET_GPIO,
      .mode = GPIO_MODE_INPUT_OUTPUT,
      .pull_up_en = GPIO_PULLUP_DISABLE,
      .pull_down_en = GPIO_PULLDOWN_DISABLE,
      .intr_type = GPIO_INTR_DISABLE,
  };
  err = gpio_config(&reset_config);
  if (err != ESP_OK)
    return err;
  err = gpio_set_level(CONFIG_RV32_ST7789_RESET_GPIO, 0);
  if (err != ESP_OK)
    return err;
#endif

  for (size_t index = 0; index < sizeof(st7789_pins) / sizeof(st7789_pins[0]);
       index++) {
    const st7789_pin_t *pin = &st7789_pins[index];

    if (pin->gpio < 0 || pin->gpio == CONFIG_RV32_ST7789_RESET_GPIO ||
        pin->gpio == CONFIG_RV32_ST7789_BACKLIGHT_GPIO)
      continue;
    err = display_backend_test_gpio(pin);
    if (err != ESP_OK)
      return err;
  }

#if CONFIG_RV32_ST7789_RESET_GPIO >= 0
  /* Release the temporary reservation before testing RESET through the same
   * single-pin helper. Reconfiguring an already reserved pin is harmless but
   * makes ESP-IDF emit a misleading GPIO-conflict warning. */
  (void)gpio_reset_pin(CONFIG_RV32_ST7789_RESET_GPIO);
  err = display_backend_test_gpio(
      &(st7789_pin_t){CONFIG_RV32_ST7789_RESET_GPIO, "RESET"});
  if (err != ESP_OK)
    return err;
  (void)gpio_set_level(CONFIG_RV32_ST7789_RESET_GPIO, 0);
#endif

  for (size_t index = 0; index < sizeof(st7789_pins) / sizeof(st7789_pins[0]);
       index++) {
    int gpio = st7789_pins[index].gpio;

    if (gpio >= 0 && gpio != CONFIG_RV32_ST7789_BACKLIGHT_GPIO)
      (void)gpio_reset_pin(gpio);
  }
  ESP_LOGI(TAG, "GPIO pad readback passed; GPIO-matrix SPI route is usable");
  return ESP_OK;
}

static esp_err_t display_backend_wait_for_transfer(uint32_t previous_count,
                                                   int64_t started_us) {
  const int64_t expected_us =
      ((int64_t)DISPLAY_FB_SIZE * 8 * 1000000 +
       CONFIG_RV32_ST7789_SPI_CLOCK_HZ - 1) /
      CONFIG_RV32_ST7789_SPI_CLOCK_HZ;
  const int64_t timeout_us = expected_us * 3 + 500000;
  while (__atomic_load_n(&color_transfers_completed, __ATOMIC_ACQUIRE) ==
         previous_count) {
    if (esp_timer_get_time() - started_us > timeout_us) {
      ESP_LOGE(TAG, "SPI color DMA timed out after %" PRId64 " us", timeout_us);
      return ESP_ERR_TIMEOUT;
    }
    vTaskDelay(1);
  }
  ESP_LOGI(TAG, "SPI color DMA completed in %" PRId64 " us",
           esp_timer_get_time() - started_us);
  return ESP_OK;
}

static esp_err_t display_backend_present_and_wait(void) {
  uint32_t previous_count =
      __atomic_load_n(&color_transfers_completed, __ATOMIC_ACQUIRE);
  int64_t started_us = esp_timer_get_time();
  esp_err_t err = display_backend_present(true);

  if (err != ESP_OK)
    return err;
  return display_backend_wait_for_transfer(previous_count, started_us);
}

#if defined(CONFIG_RV32_ST7789_STARTUP_TEST_PATTERN) &&                  \
    CONFIG_RV32_ST7789_STARTUP_TEST_PATTERN
static void display_backend_make_test_pattern(void) {
  static const uint16_t colors[] = {0xf800u, 0x07e0u, 0x001fu, 0xffffu};
  uint16_t *pixels = (uint16_t *)rgb565_framebuffer;

  for (uint32_t y = 0; y < ST7789_HEIGHT; y++) {
    for (uint32_t x = 0; x < ST7789_WIDTH; x++)
      pixels[y * ST7789_WIDTH + x] = colors[x / (ST7789_WIDTH / 4u)];
  }
}
#endif

static void display_backend_refresh_timer(void *context) {
  (void)context;
  if (backend_vsync_callback != NULL)
    (void)backend_vsync_callback(backend_vsync_context, false);
}

static esp_err_t display_backend_configure_backlight(void) {
#if CONFIG_RV32_ST7789_BACKLIGHT_GPIO >= 0
  const gpio_config_t config = {
      .pin_bit_mask = 1ULL << CONFIG_RV32_ST7789_BACKLIGHT_GPIO,
      .mode = GPIO_MODE_OUTPUT,
      .pull_up_en = GPIO_PULLUP_DISABLE,
      .pull_down_en = GPIO_PULLDOWN_DISABLE,
      .intr_type = GPIO_INTR_DISABLE,
  };
  esp_err_t err = gpio_config(&config);

  if (err != ESP_OK)
    return err;
  return gpio_set_level(CONFIG_RV32_ST7789_BACKLIGHT_GPIO,
                        ST7789_BACKLIGHT_OFF);
#else
  return ESP_OK;
#endif
}

static esp_err_t display_backend_set_backlight(bool enabled) {
#if CONFIG_RV32_ST7789_BACKLIGHT_GPIO >= 0
  return gpio_set_level(CONFIG_RV32_ST7789_BACKLIGHT_GPIO,
                        enabled ? ST7789_BACKLIGHT_ON : ST7789_BACKLIGHT_OFF);
#else
  (void)enabled;
  return ESP_OK;
#endif
}

esp_err_t display_backend_init(display_backend_vsync_callback_t vsync_callback,
                               void *vsync_context,
                               display_backend_surface_t *surface) {
  esp_err_t err;
  size_t framebuffer_alignment;

  if (vsync_callback == NULL || surface == NULL)
    return ESP_ERR_INVALID_ARG;

  ESP_RETURN_ON_ERROR(display_backend_qualify_gpio_pads(), TAG,
                      "ST7789 GPIO qualification failed");
  err = display_backend_configure_backlight();
  ESP_RETURN_ON_ERROR(err, TAG, "backlight GPIO setup failed");

  ESP_LOGI(TAG,
           "%s host, %d Hz mode %d: SCLK=GPIO%d MOSI=GPIO%d DC=GPIO%d "
           "RESET=%d BACKLIGHT=%d",
           ST7789_SPI_HOST_NAME, CONFIG_RV32_ST7789_SPI_CLOCK_HZ,
           CONFIG_RV32_ST7789_SPI_MODE, CONFIG_RV32_ST7789_SCLK_GPIO,
           CONFIG_RV32_ST7789_MOSI_GPIO, CONFIG_RV32_ST7789_DC_GPIO,
           CONFIG_RV32_ST7789_RESET_GPIO,
           CONFIG_RV32_ST7789_BACKLIGHT_GPIO);
#if CONFIG_RV32_ST7789_CS_GPIO >= 0
  ESP_LOGI(TAG, "chip-select driven on GPIO%d", CONFIG_RV32_ST7789_CS_GPIO);
#else
  ESP_LOGI(TAG, "chip-select disabled; panel must be hardwired selected");
#endif

  const spi_bus_config_t bus_config = {
      .mosi_io_num = CONFIG_RV32_ST7789_MOSI_GPIO,
      .miso_io_num = -1,
      .sclk_io_num = CONFIG_RV32_ST7789_SCLK_GPIO,
      .quadwp_io_num = -1,
      .quadhd_io_num = -1,
      .data4_io_num = -1,
      .data5_io_num = -1,
      .data6_io_num = -1,
      .data7_io_num = -1,
      .max_transfer_sz = DISPLAY_FB_SIZE,
  };
  err = spi_bus_initialize(ST7789_SPI_HOST, &bus_config, SPI_DMA_CH_AUTO);
  ESP_RETURN_ON_ERROR(err, TAG, "SPI bus initialization failed");

  const esp_lcd_panel_io_spi_config_t io_config = {
      .cs_gpio_num = CONFIG_RV32_ST7789_CS_GPIO,
      .dc_gpio_num = CONFIG_RV32_ST7789_DC_GPIO,
      .spi_mode = CONFIG_RV32_ST7789_SPI_MODE,
      .pclk_hz = CONFIG_RV32_ST7789_SPI_CLOCK_HZ,
      .trans_queue_depth = ST7789_TRANSFER_QUEUE_DEPTH,
      .on_color_trans_done = display_backend_color_transfer_done,
      .user_ctx = NULL,
      .lcd_cmd_bits = 8,
      .lcd_param_bits = 8,
      .flags.psram_dma_direct = 1,
  };
  err = esp_lcd_new_panel_io_spi(
      (esp_lcd_spi_bus_handle_t)ST7789_SPI_HOST, &io_config, &panel_io);
  ESP_RETURN_ON_ERROR(err, TAG, "ST7789 SPI panel IO creation failed");

  const esp_lcd_panel_dev_config_t panel_config = {
      .rgb_ele_order = ST7789_ELEMENT_ORDER,
      /* The guest and ESP32-P4 store uint16_t RGB565 pixels least-significant
       * byte first. ST7789 RAMCTRL can consume that order directly, avoiding
       * a byte-swap pass before every frame. */
      .data_endian = LCD_RGB_DATA_ENDIAN_LITTLE,
      .bits_per_pixel = 16,
      .reset_gpio_num = CONFIG_RV32_ST7789_RESET_GPIO,
  };
  err = esp_lcd_new_panel_st7789(panel_io, &panel_config, &panel);
  ESP_RETURN_ON_ERROR(err, TAG, "ST7789 panel creation failed");
  err = esp_lcd_panel_reset(panel);
  ESP_RETURN_ON_ERROR(err, TAG, "ST7789 reset failed");
#if CONFIG_RV32_ST7789_RESET_GPIO >= 0
  /* ESP-IDF supplies the required active-low pulse. Some modules need the
   * datasheet's conservative reset recovery time before accepting SLPOUT. */
  vTaskDelay(pdMS_TO_TICKS(120));
  ESP_LOGI(TAG, "RESET GPIO%d pulsed active-low; 120 ms recovery complete",
           CONFIG_RV32_ST7789_RESET_GPIO);
#else
  ESP_LOGI(TAG, "hardware RESET absent; software reset command completed");
#endif
  err = esp_lcd_panel_init(panel);
  ESP_RETURN_ON_ERROR(err, TAG, "ST7789 initialization command sequence failed");
  /* NORON is optional on many modules after SLPOUT, but explicitly selecting
   * normal mode makes the standard sequence work with stricter ST7789 boards. */
  err = esp_lcd_panel_io_tx_param(panel_io, LCD_CMD_NORON, NULL, 0);
  ESP_RETURN_ON_ERROR(err, TAG, "ST7789 normal-mode command failed");
  vTaskDelay(pdMS_TO_TICKS(10));
  err = esp_lcd_panel_set_gap(panel, CONFIG_RV32_ST7789_X_GAP,
                              CONFIG_RV32_ST7789_Y_GAP);
  ESP_RETURN_ON_ERROR(err, TAG, "ST7789 address-window offset failed");
  err = esp_lcd_panel_swap_xy(
      panel,
#if defined(CONFIG_RV32_ST7789_SWAP_XY) && CONFIG_RV32_ST7789_SWAP_XY
      true
#else
      false
#endif
  );
  ESP_RETURN_ON_ERROR(err, TAG, "ST7789 axis-swap setup failed");
  err = esp_lcd_panel_mirror(
      panel,
#if defined(CONFIG_RV32_ST7789_MIRROR_X) && CONFIG_RV32_ST7789_MIRROR_X
      true,
#else
      false,
#endif
#if defined(CONFIG_RV32_ST7789_MIRROR_Y) && CONFIG_RV32_ST7789_MIRROR_Y
      true
#else
      false
#endif
  );
  ESP_RETURN_ON_ERROR(err, TAG, "ST7789 mirror setup failed");
  err = esp_lcd_panel_invert_color(
      panel,
#if defined(CONFIG_RV32_ST7789_INVERT_COLORS) &&                         \
    CONFIG_RV32_ST7789_INVERT_COLORS
      true
#else
      false
#endif
  );
  ESP_RETURN_ON_ERROR(err, TAG, "ST7789 inversion setup failed");

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

  /* A changed ST7789 frame always transfers the complete RGB565 surface.
   * Do not schedule frames faster than the configured wire can carry them. */
  effective_refresh_hz = CONFIG_RV32_ST7789_SPI_CLOCK_HZ /
                         (DISPLAY_FB_SIZE * 8u);
  if (effective_refresh_hz == 0u)
    effective_refresh_hz = 1u;
  if (effective_refresh_hz > CONFIG_RV32_ST7789_REFRESH_HZ)
    effective_refresh_hz = CONFIG_RV32_ST7789_REFRESH_HZ;
  if (effective_refresh_hz != CONFIG_RV32_ST7789_REFRESH_HZ) {
    ESP_LOGW(TAG, "refresh limited from %d to %" PRIu32
                  " Hz by the %d Hz SPI wire",
             CONFIG_RV32_ST7789_REFRESH_HZ, effective_refresh_hz,
             CONFIG_RV32_ST7789_SPI_CLOCK_HZ);
  }

  backend_vsync_callback = vsync_callback;
  backend_vsync_context = vsync_context;
  const esp_timer_create_args_t timer_config = {
      .callback = display_backend_refresh_timer,
      .dispatch_method = ESP_TIMER_TASK,
      .name = "st7789_refresh",
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
      .refresh_hz = effective_refresh_hz,
      .name = "ST7789 SPI",
  };
  return ESP_OK;
}

esp_err_t display_backend_start(void) {
  esp_err_t err;

  if (panel == NULL || refresh_timer == NULL || rgb565_framebuffer == NULL)
    return ESP_ERR_INVALID_STATE;
  err = esp_lcd_panel_disp_on_off(panel, true);
  if (err != ESP_OK)
    return err;
  err = display_backend_set_backlight(true);
  if (err != ESP_OK)
    return err;

#if defined(CONFIG_RV32_ST7789_STARTUP_TEST_PATTERN) &&                  \
    CONFIG_RV32_ST7789_STARTUP_TEST_PATTERN
  display_backend_make_test_pattern();
  ESP_LOGI(TAG,
           "qualification image: red | green | blue | white RGB565 bars");
  err = display_backend_present_and_wait();
  if (err != ESP_OK)
    return err;
  vTaskDelay(pdMS_TO_TICKS(ST7789_TEST_PATTERN_MS));
  memset(rgb565_framebuffer, 0, DISPLAY_FB_SIZE);
#endif

  /* Do not let Linux reuse the scanout buffer until the initial transfer has
   * left PSRAM. This also proves that the ESP LCD SPI ISR completes. */
  err = display_backend_present_and_wait();
  if (err != ESP_OK)
    return err;
  ESP_LOGI(TAG,
           "panel command path and SPI DMA completed; starting Linux display");
  return esp_timer_start_periodic(refresh_timer, 1000000u / effective_refresh_hz);
}

esp_err_t display_backend_present(bool frame_changed) {
  if (panel == NULL || rgb565_framebuffer == NULL)
    return ESP_ERR_INVALID_STATE;
  if (!frame_changed)
    return ESP_OK;

  /* esp_lcd queues the color payload through SPI DMA. The next CASET/RASET
   * command naturally waits for any preceding frame, keeping only one live
   * use of the shared framebuffer without a polling loop on CPU1. */
  return esp_lcd_panel_draw_bitmap(panel, 0, 0, ST7789_WIDTH, ST7789_HEIGHT,
                                   rgb565_framebuffer);
}
