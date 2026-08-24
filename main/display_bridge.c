/*
 * Native ST7703 display and paravirtual RGB565 framebuffer bridge.
 *
 * Linux sees the DPI driver's scanout allocation as a simple-framebuffer at
 * DISPLAY_FB_GUEST_BASE.  Guest reads and writes are redirected to the real
 * ESP-IDF framebuffer by the emulator's MMIO callbacks.
 *
 * SPDX-License-Identifier: BSD-3-Clause
 */

#include "display_bridge.h"

#include <stdio.h>
#include <string.h>

#include "driver/gpio.h"
#include "driver/ledc.h"
#include "esp_cache.h"
#include "esp_err.h"
#include "esp_lcd_mipi_dsi.h"
#include "esp_lcd_panel_ops.h"
#include "esp_lcd_st7703.h"
#include "esp_ldo_regulator.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"

#define LCD_RESET_GPIO       GPIO_NUM_27
#define LCD_BACKLIGHT_GPIO   GPIO_NUM_26
#define LCD_BACKLIGHT_ENABLE GPIO_NUM_33
#define LCD_BACKLIGHT_TIMER  LEDC_TIMER_1
#define LCD_BACKLIGHT_CHAN   LEDC_CHANNEL_1
#define LCD_DSI_PHY_LDO_CHAN 3
#define LCD_PANEL_LDO_CHAN   4
/*
 * The Waveshare timing is 38 MHz / (720 + 50 + 20 + 50) /
 * (720 + 20 + 4 + 20) = 59.22 Hz.  Poll at the emulator's 5 ms dirty-range
 * handoff cadence so writes become DMA-visible with low latency, including
 * writes made while the panel is scanning the current frame.  The dirty check
 * makes idle polls free of cache-writeback work.
 */
#define LCD_FLUSH_PERIOD_MS  5u
#define LCD_TASK_STACK_SIZE  6144u
/*
 * ESP-Hosted creates its SDIO workers at priority 23.  Keep the short,
 * periodic cache-clean task above them so a broken or disconnected C6 cannot
 * freeze an otherwise healthy framebuffer.  The task sleeps between flushes.
 */
#define LCD_TASK_PRIORITY    (configMAX_PRIORITIES - 1)

static esp_ldo_channel_handle_t dsi_phy_ldo;
static esp_ldo_channel_handle_t panel_ldo;
static esp_lcd_dsi_bus_handle_t dsi_bus;
static esp_lcd_panel_io_handle_t panel_io;
static esp_lcd_panel_handle_t panel;
static uint8_t *framebuffer;
/* CPU0 owns the producer range; CPU1 only touches the pending range. */
static bool producer_dirty;
static size_t producer_dirty_start = DISPLAY_FB_SIZE;
static size_t producer_dirty_end;
static bool pending_dirty;
static size_t pending_dirty_start = DISPLAY_FB_SIZE;
static size_t pending_dirty_end;
static portMUX_TYPE dirty_lock = portMUX_INITIALIZER_UNLOCKED;
static int display_init_result = -1;

static esp_err_t display_enable_power(void)
{
	esp_ldo_channel_config_t dsi_ldo_config = {
		.chan_id = LCD_DSI_PHY_LDO_CHAN,
		.voltage_mv = 2500,
	};
	esp_ldo_channel_config_t panel_ldo_config = {
		.chan_id = LCD_PANEL_LDO_CHAN,
		.voltage_mv = 3300,
	};
	esp_err_t err = esp_ldo_acquire_channel(&dsi_ldo_config, &dsi_phy_ldo);

	if (err != ESP_OK)
		return err;

	return esp_ldo_acquire_channel(&panel_ldo_config, &panel_ldo);
}

static esp_err_t display_enable_backlight(void)
{
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

static int display_hardware_init(void)
{
	esp_err_t err;

	printf("\n=== Initializing Waveshare ST7703 display ===\n");

	err = display_enable_power();
	if (err != ESP_OK)
		goto fail;

	err = display_enable_backlight();
	if (err != ESP_OK)
		goto fail;

	const esp_lcd_dsi_bus_config_t bus_config = {
		.bus_id = 0,
		.num_data_lanes = 2,
		.phy_clk_src = 0,
		.lane_bit_rate_mbps = 480,
	};
	err = esp_lcd_new_dsi_bus(&bus_config, &dsi_bus);
	if (err != ESP_OK)
		goto fail;

	const esp_lcd_dbi_io_config_t io_config = {
		.virtual_channel = 0,
		.lcd_cmd_bits = 8,
		.lcd_param_bits = 8,
	};
	err = esp_lcd_new_panel_io_dbi(dsi_bus, &io_config, &panel_io);
	if (err != ESP_OK)
		goto fail;

	esp_lcd_dpi_panel_config_t dpi_config =
		ST7703_720_720_PANEL_60HZ_DPI_CONFIG(
			LCD_COLOR_PIXEL_FORMAT_RGB565);
	dpi_config.num_fbs = 1;

	st7703_vendor_config_t vendor_config = {
		.flags = {
			.use_mipi_interface = 1,
		},
		.mipi_config = {
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
		goto fail;

	err = esp_lcd_panel_reset(panel);
	if (err != ESP_OK)
		goto fail;

	err = esp_lcd_panel_init(panel);
	if (err != ESP_OK)
		goto fail;

	err = esp_lcd_dpi_panel_get_frame_buffer(
		panel, 1, (void **)&framebuffer);
	if (err != ESP_OK || framebuffer == NULL)
		goto fail;

	memset(framebuffer, 0, DISPLAY_FB_SIZE);
	err = esp_cache_msync(framebuffer, DISPLAY_FB_SIZE,
		ESP_CACHE_MSYNC_FLAG_DIR_C2M |
		ESP_CACHE_MSYNC_FLAG_UNALIGNED);
	if (err != ESP_OK)
		goto fail;

	err = esp_lcd_panel_disp_on_off(panel, true);
	if (err != ESP_OK)
		goto fail;

	printf("ST7703 ready: 720x720 RGB565 at 59 Hz, framebuffer %p mapped at "
	       "guest 0x%08x (%u bytes), service on CPU%d\n",
	       framebuffer, DISPLAY_FB_GUEST_BASE,
	       (unsigned int)DISPLAY_FB_SIZE, (int)xPortGetCoreID());
	return 0;

fail:
	printf("WARNING: ST7703 initialization failed: %s; continuing with "
	       "serial console only\n", esp_err_to_name(err));
	framebuffer = NULL;
	return -1;
}

static void display_flush_dirty(void)
{
	size_t start;
	size_t end;

	portENTER_CRITICAL(&dirty_lock);
	if (!pending_dirty) {
		portEXIT_CRITICAL(&dirty_lock);
		return;
	}

	start = pending_dirty_start;
	end = pending_dirty_end;
	pending_dirty = false;
	pending_dirty_start = DISPLAY_FB_SIZE;
	pending_dirty_end = 0;
	portEXIT_CRITICAL(&dirty_lock);

	if (start >= end || end > DISPLAY_FB_SIZE)
		return;

	esp_err_t err = esp_cache_msync(framebuffer + start, end - start,
		ESP_CACHE_MSYNC_FLAG_DIR_C2M |
		ESP_CACHE_MSYNC_FLAG_UNALIGNED);

	if (err != ESP_OK) {
		/* Preserve the range so a transient synchronization failure retries. */
		portENTER_CRITICAL(&dirty_lock);
		if (start < pending_dirty_start)
			pending_dirty_start = start;
		if (end > pending_dirty_end)
			pending_dirty_end = end;
		pending_dirty = true;
		portEXIT_CRITICAL(&dirty_lock);
	}
}

static void display_service_task(void *argument)
{
	TaskHandle_t init_waiter = (TaskHandle_t)argument;
	TickType_t last_wake;

	display_init_result = display_hardware_init();
	xTaskNotifyGive(init_waiter);

	if (display_init_result < 0) {
		vTaskDelete(NULL);
		return;
	}

	last_wake = xTaskGetTickCount();
	for (;;) {
		vTaskDelayUntil(&last_wake,
			pdMS_TO_TICKS(LCD_FLUSH_PERIOD_MS));
		display_flush_dirty();
	}
}

int display_bridge_init(void)
{
	TaskHandle_t caller = xTaskGetCurrentTaskHandle();
	BaseType_t display_core =
		(CONFIG_FREERTOS_NUMBER_OF_CORES > 1)
			? ((xPortGetCoreID() + 1) % CONFIG_FREERTOS_NUMBER_OF_CORES)
			: xPortGetCoreID();
	BaseType_t created = xTaskCreatePinnedToCore(
		display_service_task, "lcd_service", LCD_TASK_STACK_SIZE,
		caller, LCD_TASK_PRIORITY, NULL, display_core);

	if (created != pdPASS) {
		printf("WARNING: Failed to create LCD service task\n");
		return -1;
	}

	ulTaskNotifyTake(pdTRUE, portMAX_DELAY);
	return display_init_result;
}

void display_bridge_commit(void)
{
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

bool display_bridge_contains(uint32_t address, size_t width)
{
	if (framebuffer == NULL || width == 0 || width > sizeof(uint32_t) ||
		address < DISPLAY_FB_GUEST_BASE)
		return false;

	uint32_t offset = address - DISPLAY_FB_GUEST_BASE;
	return offset < DISPLAY_FB_SIZE && width <= DISPLAY_FB_SIZE - offset;
}

uint32_t display_bridge_load(uint32_t address, size_t width)
{
	const uint8_t *source =
		framebuffer + address - DISPLAY_FB_GUEST_BASE;

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

void display_bridge_store(uint32_t address, uint32_t value, size_t width)
{
	/* The emulator validates this access before dispatching it here. */
	size_t offset = address - DISPLAY_FB_GUEST_BASE;

	switch (width) {
	case 1:
		framebuffer[offset] = (uint8_t)value;
		break;
	case 2:
		__builtin_memcpy(framebuffer + offset, &value, sizeof(uint16_t));
		break;
	case 4:
		__builtin_memcpy(framebuffer + offset, &value, sizeof(uint32_t));
		break;
	default:
		return;
	}
	if (offset < producer_dirty_start)
		producer_dirty_start = offset;
	if (offset + width > producer_dirty_end)
		producer_dirty_end = offset + width;
	producer_dirty = true;
}
