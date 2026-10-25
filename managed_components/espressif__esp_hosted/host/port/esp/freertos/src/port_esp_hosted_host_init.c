/*
 * SPDX-FileCopyrightText: 2024-2025 Espressif Systems (Shanghai) CO LTD
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#include "esp_log.h"
#include "esp_hosted.h"

#include "port_esp_hosted_host_log.h"

#include "esp_private/startup_internal.h"

DEFINE_LOG_TAG(host_init);

//ESP_SYSTEM_INIT_FN(esp_hosted_host_init, BIT(0), 120)
/* Not a constructor: constructors run on CPU0, and the SDIO transport
 * allocates the SDMMC interrupt on the CPU that starts it.  The application
 * calls esp_hosted_init() on CPU1 instead. */
void esp_hosted_host_init(void);
void esp_hosted_host_init(void)
{
	ESP_LOGI(TAG, "ESP Hosted : Host chip_ip[%d]", CONFIG_IDF_FIRMWARE_CHIP_ID);
	ESP_ERROR_CHECK(esp_hosted_init());
}

static void __attribute__((destructor)) esp_hosted_host_deinit(void)
{
	ESP_LOGI(TAG, "ESP Hosted deinit");
	esp_hosted_deinit();
}
