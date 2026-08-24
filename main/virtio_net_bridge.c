/*
 * ESP32-C6 SDIO to RV32 Linux virtio-net bridge.
 *
 * The C6 owns 802.11 association.  Linux owns Ethernet, ARP, DHCP, IP and all
 * higher layers.  Packets cross the emulator boundary through standard
 * virtio-mmio split rings in identity-mapped PSRAM.  All SDIO/RPC and packet
 * queue work runs from a FreeRTOS task pinned to the P4's second core; the
 * emulator core only services small MMIO register accesses and its PLIC bit.
 *
 * SPDX-License-Identifier: BSD-3-Clause
 */

#include <stdbool.h>
#include <inttypes.h>
#include <stdint.h>
#include <stdio.h>
#include <string.h>

#include "freertos/FreeRTOS.h"
#include "freertos/queue.h"
#include "freertos/task.h"

#include "esp_err.h"
#include "esp_event.h"
#include "esp_hosted.h"
#include "esp_log.h"
#include "esp_private/wifi.h"
#include "esp_wifi.h"

#include "psram.h"
#include "virtio_net_bridge.h"

#define TAG "rv32-net"

#define VIRTIO_MMIO_MAGIC_VALUE       0x000u
#define VIRTIO_MMIO_VERSION           0x004u
#define VIRTIO_MMIO_DEVICE_ID         0x008u
#define VIRTIO_MMIO_VENDOR_ID         0x00cu
#define VIRTIO_MMIO_DEVICE_FEATURES   0x010u
#define VIRTIO_MMIO_DEVICE_FEATURES_SEL 0x014u
#define VIRTIO_MMIO_DRIVER_FEATURES   0x020u
#define VIRTIO_MMIO_DRIVER_FEATURES_SEL 0x024u
#define VIRTIO_MMIO_QUEUE_SEL         0x030u
#define VIRTIO_MMIO_QUEUE_NUM_MAX     0x034u
#define VIRTIO_MMIO_QUEUE_NUM         0x038u
#define VIRTIO_MMIO_QUEUE_READY       0x044u
#define VIRTIO_MMIO_QUEUE_NOTIFY      0x050u
#define VIRTIO_MMIO_INTERRUPT_STATUS  0x060u
#define VIRTIO_MMIO_INTERRUPT_ACK     0x064u
#define VIRTIO_MMIO_STATUS            0x070u
#define VIRTIO_MMIO_QUEUE_DESC_LOW    0x080u
#define VIRTIO_MMIO_QUEUE_DESC_HIGH   0x084u
#define VIRTIO_MMIO_QUEUE_AVAIL_LOW   0x090u
#define VIRTIO_MMIO_QUEUE_AVAIL_HIGH  0x094u
#define VIRTIO_MMIO_QUEUE_USED_LOW    0x0a0u
#define VIRTIO_MMIO_QUEUE_USED_HIGH   0x0a4u
#define VIRTIO_MMIO_CONFIG_GENERATION 0x0fcu
#define VIRTIO_MMIO_CONFIG            0x100u

#define VIRTIO_MMIO_INT_VRING  (1u << 0)
#define VIRTIO_MMIO_INT_CONFIG (1u << 1)

#define VIRTIO_F_VERSION_1      32u
#define VIRTIO_NET_F_MAC         5u
#define VIRTIO_NET_F_STATUS     16u
#define VIRTIO_NET_S_LINK_UP     1u

#define VIRTIO_STATUS_DRIVER_OK  (1u << 2)

#define VIRTQ_DESC_F_NEXT        1u
#define VIRTQ_DESC_F_WRITE       2u
#define VIRTQ_DESC_F_INDIRECT    4u
#define VIRTQ_AVAIL_F_NO_INTERRUPT 1u

#define VIRTIO_NET_RX_QUEUE      0u
#define VIRTIO_NET_TX_QUEUE      1u
#define VIRTIO_NET_QUEUE_COUNT   2u
#define VIRTIO_NET_QUEUE_MAX     64u
#define VIRTIO_NET_HEADER_SIZE   12u
#define ETHERNET_FRAME_MAX       1600u
#define RX_SOFTWARE_QUEUE_DEPTH  8u
#define C6_RETRY_DELAY_MS        5000u
#define WIFI_RETRY_DELAY_MS      2000u
#define BRIDGE_TASK_PRIORITY     10u

struct virtq_desc {
	uint64_t address;
	uint32_t length;
	uint16_t flags;
	uint16_t next;
} __attribute__((packed));

struct virtq_used_elem {
	uint32_t id;
	uint32_t length;
} __attribute__((packed));

struct virtio_queue_state {
	uint32_t size;
	uint32_t ready;
	uint64_t desc;
	uint64_t avail;
	uint64_t used;
	uint16_t last_avail;
};

struct rx_packet {
	uint16_t length;
	uint8_t data[ETHERNET_FRAME_MAX];
};

static struct virtio_queue_state queues[VIRTIO_NET_QUEUE_COUNT];
static uint32_t device_features_sel;
static uint32_t driver_features_sel;
static uint32_t driver_features[2];
static uint32_t queue_sel;
static uint32_t device_status;
static uint32_t interrupt_status;
static uint32_t config_generation;
static bool station_connected;
static bool wifi_connect_pending;
static TickType_t wifi_connect_due;
static int32_t hosted_event_pending = -1;
static int32_t wifi_event_pending = -1;
static uint32_t wifi_disconnect_reason;
static uint8_t station_mac[6] = { 0x02, 0x50, 0x34, 0x00, 0x00, 0x01 };
static uint32_t rx_frame_count;
static uint32_t rx_drop_count;
static uint32_t tx_frame_count;
static uint32_t tx_error_count;
static bool packet_path_reported;

static struct rx_packet rx_packet_pool[RX_SOFTWARE_QUEUE_DEPTH];
static QueueHandle_t rx_ready_queue;
static QueueHandle_t rx_free_queue;
static StaticQueue_t rx_ready_queue_control;
static StaticQueue_t rx_free_queue_control;
static uint8_t rx_ready_queue_storage[RX_SOFTWARE_QUEUE_DEPTH *
				sizeof(struct rx_packet *)];
static uint8_t rx_free_queue_storage[RX_SOFTWARE_QUEUE_DEPTH *
			       sizeof(struct rx_packet *)];
static TaskHandle_t bridge_task_handle;

static esp_err_t wifi_receive(void *buffer, uint16_t length,
			      void *buffer_to_free);

static bool guest_range_valid(uint64_t address, size_t length)
{
	if ((address >> 32) != 0 || length == 0)
		return false;
	if (address < GUEST_DMA_BASE || address > GUEST_PHYS_END)
		return false;
	return length <= (size_t)(GUEST_PHYS_END - (uint32_t)address);
}

static bool guest_read(uint64_t address, void *destination, size_t length)
{
	if (!guest_range_valid(address, length))
		return false;
	memcpy(destination, (const void *)(uintptr_t)(uint32_t)address, length);
	return true;
}

static bool guest_write(uint64_t address, const void *source, size_t length)
{
	if (!guest_range_valid(address, length))
		return false;
	memcpy((void *)(uintptr_t)(uint32_t)address, source, length);
	return true;
}

static bool queue_snapshot(uint32_t index, struct virtio_queue_state *queue)
{
	if (index >= VIRTIO_NET_QUEUE_COUNT)
		return false;

	if (!__atomic_load_n(&queues[index].ready, __ATOMIC_ACQUIRE))
		return false;
	*queue = queues[index];
	return queue->size &&
		queue->size <= VIRTIO_NET_QUEUE_MAX &&
		guest_range_valid(queue->desc,
			queue->size * sizeof(struct virtq_desc)) &&
		guest_range_valid(queue->avail,
			6u + queue->size * sizeof(uint16_t)) &&
		guest_range_valid(queue->used,
			6u + queue->size * sizeof(struct virtq_used_elem));
}

static bool queue_read_desc(const struct virtio_queue_state *queue,
			    uint16_t index, struct virtq_desc *descriptor)
{
	if (index >= queue->size)
		return false;
	return guest_read(queue->desc + index * sizeof(*descriptor), descriptor,
			  sizeof(*descriptor));
}

static bool queue_pop_available(uint32_t index,
				struct virtio_queue_state *queue,
				uint16_t *head)
{
	uint16_t available_index;
	uint16_t ring_entry;

	if (!queue_snapshot(index, queue))
		return false;
	if (!guest_read(queue->avail + 2u, &available_index,
			sizeof(available_index)))
		return false;
	__atomic_thread_fence(__ATOMIC_ACQUIRE);
	if (queues[index].last_avail == available_index)
		return false;

	uint16_t slot = queues[index].last_avail % queue->size;
	if (!guest_read(queue->avail + 4u + slot * sizeof(uint16_t),
			&ring_entry, sizeof(ring_entry)))
		return false;
	queues[index].last_avail++;
	queue->last_avail = queues[index].last_avail;
	*head = ring_entry;
	return true;
}

static void queue_complete(uint32_t index,
			   const struct virtio_queue_state *queue,
			   uint16_t head, uint32_t length)
{
	uint16_t used_index;
	uint16_t available_flags = 0;
	struct virtq_used_elem element = {
		.id = head,
		.length = length,
	};

	if (!guest_read(queue->used + 2u, &used_index, sizeof(used_index)))
		return;
	uint16_t slot = used_index % queue->size;
	if (!guest_write(queue->used + 4u +
			 slot * sizeof(struct virtq_used_elem),
			 &element, sizeof(element)))
		return;
	__atomic_thread_fence(__ATOMIC_RELEASE);
	used_index++;
	if (!guest_write(queue->used + 2u, &used_index, sizeof(used_index)))
		return;
	__atomic_thread_fence(__ATOMIC_RELEASE);

	guest_read(queue->avail, &available_flags, sizeof(available_flags));
	if (!(available_flags & VIRTQ_AVAIL_F_NO_INTERRUPT))
		__atomic_fetch_or(&interrupt_status, VIRTIO_MMIO_INT_VRING,
				  __ATOMIC_RELEASE);
}

static bool descriptor_copy_out(const struct virtio_queue_state *queue,
				uint16_t head, uint8_t *packet,
				size_t *packet_length)
{
	struct virtq_desc descriptor;
	size_t total = 0;
	size_t skipped = 0;
	uint16_t index = head;

	for (uint32_t walked = 0; walked < queue->size; walked++) {
		if (!queue_read_desc(queue, index, &descriptor) ||
		    (descriptor.flags & (VIRTQ_DESC_F_WRITE |
					 VIRTQ_DESC_F_INDIRECT)) ||
		    !guest_range_valid(descriptor.address, descriptor.length))
			return false;

		size_t offset = 0;
		if (skipped < VIRTIO_NET_HEADER_SIZE) {
			size_t need = VIRTIO_NET_HEADER_SIZE - skipped;
			offset = descriptor.length < need ? descriptor.length : need;
			skipped += offset;
		}
		if (descriptor.length > offset) {
			size_t copy_length = descriptor.length - offset;
			if (copy_length > ETHERNET_FRAME_MAX - total)
				return false;
			if (!guest_read(descriptor.address + offset, packet + total,
					copy_length))
				return false;
			total += copy_length;
		}

		if (!(descriptor.flags & VIRTQ_DESC_F_NEXT)) {
			if (skipped != VIRTIO_NET_HEADER_SIZE || total < 14u)
				return false;
			*packet_length = total;
			return true;
		}
		index = descriptor.next;
	}
	return false;
}

static bool descriptor_copy_in(const struct virtio_queue_state *queue,
			       uint16_t head, const uint8_t *packet,
			       size_t packet_length, uint32_t *used_length)
{
	static const uint8_t zero_header[VIRTIO_NET_HEADER_SIZE];
	struct virtq_desc descriptor;
	size_t source_offset = 0;
	size_t total = VIRTIO_NET_HEADER_SIZE + packet_length;
	uint16_t index = head;

	for (uint32_t walked = 0; walked < queue->size; walked++) {
		if (!queue_read_desc(queue, index, &descriptor) ||
		    !(descriptor.flags & VIRTQ_DESC_F_WRITE) ||
		    (descriptor.flags & VIRTQ_DESC_F_INDIRECT) ||
		    !guest_range_valid(descriptor.address, descriptor.length))
			return false;

		size_t destination_offset = 0;
		while (destination_offset < descriptor.length &&
		       source_offset < total) {
			const uint8_t *source;
			size_t available;
			if (source_offset < VIRTIO_NET_HEADER_SIZE) {
				source = zero_header + source_offset;
				available = VIRTIO_NET_HEADER_SIZE - source_offset;
			} else {
				size_t packet_offset =
					source_offset - VIRTIO_NET_HEADER_SIZE;
				source = packet + packet_offset;
				available = packet_length - packet_offset;
			}
			size_t room = descriptor.length - destination_offset;
			size_t copy_length = available < room ? available : room;
			if (!guest_write(descriptor.address + destination_offset,
					source, copy_length))
				return false;
			destination_offset += copy_length;
			source_offset += copy_length;
		}

		if (source_offset == total) {
			*used_length = total;
			return true;
		}
		if (!(descriptor.flags & VIRTQ_DESC_F_NEXT))
			return false;
		index = descriptor.next;
	}
	return false;
}

static bool process_one_tx(void)
{
	struct virtio_queue_state queue;
	uint16_t head;
	uint8_t packet[ETHERNET_FRAME_MAX];
	size_t packet_length = 0;

	if (!queue_pop_available(VIRTIO_NET_TX_QUEUE, &queue, &head))
		return false;
	if (descriptor_copy_out(&queue, head, packet, &packet_length) &&
	    __atomic_load_n(&station_connected, __ATOMIC_ACQUIRE)) {
		if (esp_wifi_internal_tx(WIFI_IF_STA, packet, packet_length) ==
		    ESP_OK)
			__atomic_add_fetch(&tx_frame_count, 1u, __ATOMIC_RELAXED);
		else
			__atomic_add_fetch(&tx_error_count, 1u, __ATOMIC_RELAXED);
	}
	queue_complete(VIRTIO_NET_TX_QUEUE, &queue, head, 0);
	return true;
}

static bool process_one_rx(const struct rx_packet *packet)
{
	struct virtio_queue_state queue;
	uint16_t head;
	uint32_t used_length = 0;

	if (!queue_pop_available(VIRTIO_NET_RX_QUEUE, &queue, &head))
		return false;
	if (!descriptor_copy_in(&queue, head, packet->data, packet->length,
				&used_length))
		used_length = 0;
	queue_complete(VIRTIO_NET_RX_QUEUE, &queue, head, used_length);
	return true;
}

static void set_link_state(bool connected)
{
	if (__atomic_exchange_n(&station_connected, connected,
				__ATOMIC_ACQ_REL) == connected)
		return;
	__atomic_add_fetch(&config_generation, 1u, __ATOMIC_RELAXED);
	__atomic_fetch_or(&interrupt_status, VIRTIO_MMIO_INT_CONFIG,
			  __ATOMIC_RELEASE);
	if (bridge_task_handle)
		xTaskNotifyGive(bridge_task_handle);
}

static void schedule_wifi_connect(uint32_t delay_ms)
{
	TickType_t due = xTaskGetTickCount() + pdMS_TO_TICKS(delay_ms);

	__atomic_store_n(&wifi_connect_due, due, __ATOMIC_RELAXED);
	__atomic_store_n(&wifi_connect_pending, true, __ATOMIC_RELEASE);
	if (bridge_task_handle)
		xTaskNotifyGive(bridge_task_handle);
}

static void hosted_event_handler(void *argument, esp_event_base_t base,
				 int32_t event_id, void *event_data)
{
	(void)argument;
	(void)event_data;
	if (base != ESP_HOSTED_EVENT)
		return;
	if (event_id != ESP_HOSTED_EVENT_TRANSPORT_UP &&
	    event_id != ESP_HOSTED_EVENT_TRANSPORT_FAILURE &&
	    event_id != ESP_HOSTED_EVENT_TRANSPORT_DOWN)
		return;

	/*
	 * ESP-Hosted dispatches this from IDF's shared sys_evt task.  Keep the
	 * callback leaf-like: logging here enters newlib's formatter through a
	 * recursive lock and previously overflowed the event task stack.  The
	 * CPU1 bridge task owns all diagnostics and link-state changes.
	 */
	__atomic_store_n(&hosted_event_pending, event_id, __ATOMIC_RELEASE);
	if (bridge_task_handle)
		xTaskNotifyGive(bridge_task_handle);
}

static void wifi_event_handler(void *argument, esp_event_base_t base,
			       int32_t event_id, void *event_data)
{
	(void)argument;
	(void)event_data;
	if (base != WIFI_EVENT)
		return;
	if (event_id != WIFI_EVENT_STA_START &&
	    event_id != WIFI_EVENT_STA_CONNECTED &&
	    event_id != WIFI_EVENT_STA_DISCONNECTED)
		return;

	if (event_id == WIFI_EVENT_STA_DISCONNECTED) {
		const wifi_event_sta_disconnected_t *disconnected = event_data;

		__atomic_store_n(&wifi_disconnect_reason,
				 disconnected ? disconnected->reason : 0u,
				 __ATOMIC_RELAXED);
	}
	__atomic_store_n(&wifi_event_pending, event_id, __ATOMIC_RELEASE);
	if (bridge_task_handle)
		xTaskNotifyGive(bridge_task_handle);
}

static void process_hosted_event(void)
{
	int32_t event_id = __atomic_exchange_n(&hosted_event_pending, -1,
					       __ATOMIC_ACQ_REL);

	if (event_id == ESP_HOSTED_EVENT_TRANSPORT_UP) {
		ESP_LOGI(TAG, "C6 SDIO transport is up");
	} else if (event_id == ESP_HOSTED_EVENT_TRANSPORT_FAILURE ||
		   event_id == ESP_HOSTED_EVENT_TRANSPORT_DOWN) {
		ESP_LOGE(TAG, "C6 SDIO transport went down (event %" PRId32 ")",
			 event_id);
		set_link_state(false);
	}
}

static void process_wifi_event(void)
{
	int32_t event_id = __atomic_exchange_n(&wifi_event_pending, -1,
					       __ATOMIC_ACQ_REL);

	if (event_id == WIFI_EVENT_STA_START) {
		if (CONFIG_RV32_WIFI_SSID[0] != '\0')
			schedule_wifi_connect(0);
	} else if (event_id == WIFI_EVENT_STA_CONNECTED) {
		wifi_ap_record_t access_point = { 0 };

		/* Wi-Fi startup may replace the remote raw receive callback. */
		if (esp_wifi_internal_reg_rxcb(WIFI_IF_STA, wifi_receive) != ESP_OK)
			ESP_LOGE(TAG, "could not attach connected raw RX path");
		if (esp_wifi_sta_get_ap_info(&access_point) == ESP_OK)
			ESP_LOGI(TAG,
				 "associated BSSID %02x:%02x:%02x:%02x:%02x:%02x channel %u",
				 access_point.bssid[0], access_point.bssid[1],
				 access_point.bssid[2], access_point.bssid[3],
				 access_point.bssid[4], access_point.bssid[5],
				 access_point.primary);
		__atomic_store_n(&wifi_connect_pending, false, __ATOMIC_RELEASE);
		ESP_LOGI(TAG, "C6 associated; Linux eth0 carrier on");
		set_link_state(true);
	} else if (event_id == WIFI_EVENT_STA_DISCONNECTED) {
		uint32_t reason = __atomic_load_n(&wifi_disconnect_reason,
						  __ATOMIC_RELAXED);

		esp_wifi_internal_reg_rxcb(WIFI_IF_STA, NULL);
		ESP_LOGW(TAG, "C6 disconnected from AP (reason %" PRIu32
			 "); retrying", reason);
		set_link_state(false);
		if (CONFIG_RV32_WIFI_SSID[0] != '\0')
			schedule_wifi_connect(WIFI_RETRY_DELAY_MS);
	}
}

static esp_err_t wifi_receive(void *buffer, uint16_t length,
			      void *buffer_to_free)
{
	struct rx_packet *packet = NULL;
	esp_err_t result = ESP_OK;

	if (length < 14u || length > ETHERNET_FRAME_MAX ||
	    rx_ready_queue == NULL || rx_free_queue == NULL) {
		result = ESP_ERR_INVALID_SIZE;
	} else if (xQueueReceive(rx_free_queue, &packet, 0) != pdTRUE) {
		__atomic_add_fetch(&rx_drop_count, 1u, __ATOMIC_RELAXED);
		result = ESP_ERR_NO_MEM;
	} else {
		packet->length = length;
		memcpy(packet->data, buffer, length);
		if (xQueueSend(rx_ready_queue, &packet, 0) != pdTRUE) {
			xQueueSend(rx_free_queue, &packet, 0);
			__atomic_add_fetch(&rx_drop_count, 1u, __ATOMIC_RELAXED);
			result = ESP_ERR_NO_MEM;
		} else if (bridge_task_handle) {
			__atomic_add_fetch(&rx_frame_count, 1u, __ATOMIC_RELAXED);
			xTaskNotifyGive(bridge_task_handle);
		}
	}
	esp_wifi_internal_free_rx_buffer(buffer_to_free);
	return result;
}

static bool initialize_c6(void)
{
	wifi_init_config_t init_config = WIFI_INIT_CONFIG_DEFAULT();
	wifi_config_t wifi_config = { 0 };
	esp_err_t error;

	ESP_LOGI(TAG, "starting C6 over SDIO4 GPIO14-19, reset GPIO54");
	error = esp_hosted_init();
	if (error != ESP_OK) {
		ESP_LOGE(TAG, "ESP-Hosted initialization failed: %s",
			 esp_err_to_name(error));
		return false;
	}

	/*
	 * The C6 may still be powering up when the P4 reaches this task.  Keep all
	 * retries on CPU1 and never restart or block the emulator/display core.
	 */
	for (;;) {
		error = esp_hosted_connect_to_slave();
		if (error == ESP_OK)
			break;
		ESP_LOGW(TAG, "C6 transport unavailable (%s); retrying in %u ms",
			 esp_err_to_name(error), C6_RETRY_DELAY_MS);
		vTaskDelay(pdMS_TO_TICKS(C6_RETRY_DELAY_MS));
	}

	/*
	 * Factory C6 firmware predating ESP-Hosted 2.6 does not implement the
	 * firmware-version RPC.  The transport INIT event already reports its
	 * compatibility, so querying it again only causes a guaranteed timeout
	 * before remote Wi-Fi initialization.
	 */

	error = esp_wifi_init(&init_config);
	if (error != ESP_OK) {
		ESP_LOGE(TAG, "remote esp_wifi_init failed: %s",
			 esp_err_to_name(error));
		return false;
	}
	if (esp_wifi_get_mac(WIFI_IF_STA, station_mac) != ESP_OK)
		ESP_LOGW(TAG, "using fallback locally administered MAC");
	else
		__atomic_add_fetch(&config_generation, 1u, __ATOMIC_RELAXED);

	if (esp_wifi_internal_reg_rxcb(WIFI_IF_STA, wifi_receive) != ESP_OK) {
		ESP_LOGE(TAG, "could not attach raw Ethernet receive path");
		return false;
	}
	if (esp_wifi_set_mode(WIFI_MODE_STA) != ESP_OK)
		return false;

	if (CONFIG_RV32_WIFI_SSID[0] != '\0') {
		strncpy((char *)wifi_config.sta.ssid, CONFIG_RV32_WIFI_SSID,
			sizeof(wifi_config.sta.ssid) - 1u);
		strncpy((char *)wifi_config.sta.password, CONFIG_RV32_WIFI_PASSWORD,
			sizeof(wifi_config.sta.password) - 1u);
		wifi_config.sta.threshold.authmode = WIFI_AUTH_WPA2_PSK;
		wifi_config.sta.sae_pwe_h2e = WPA3_SAE_PWE_BOTH;
		if (esp_wifi_set_config(WIFI_IF_STA, &wifi_config) != ESP_OK)
			return false;
	} else {
		ESP_LOGW(TAG, "SSID is empty; C6 is ready but association is disabled");
	}

	error = esp_wifi_start();
	if (error != ESP_OK) {
		ESP_LOGE(TAG, "remote esp_wifi_start failed: %s",
			 esp_err_to_name(error));
		return false;
	}
	ESP_LOGI(TAG, "virtio-net MAC %02x:%02x:%02x:%02x:%02x:%02x",
		 station_mac[0], station_mac[1], station_mac[2], station_mac[3],
		 station_mac[4], station_mac[5]);
	return true;
}

static void bridge_task(void *argument)
{
	(void)argument;
	bool c6_ready = initialize_c6();
	struct rx_packet *pending_rx = NULL;

	if (!c6_ready)
		ESP_LOGE(TAG, "wireless link unavailable; Linux eth0 remains down");

	for (;;) {
		ulTaskNotifyTake(pdTRUE, pdMS_TO_TICKS(20));
		process_hosted_event();
		process_wifi_event();

		if (c6_ready &&
		    __atomic_load_n(&wifi_connect_pending, __ATOMIC_ACQUIRE)) {
			TickType_t now = xTaskGetTickCount();
			TickType_t due = __atomic_load_n(&wifi_connect_due,
						       __ATOMIC_RELAXED);

			if ((int32_t)(now - due) >= 0 &&
			    __atomic_exchange_n(&wifi_connect_pending, false,
						__ATOMIC_ACQ_REL)) {
				esp_err_t error = esp_wifi_connect();

				if (error != ESP_OK) {
					ESP_LOGW(TAG,
						 "C6 association request failed: %s",
						 esp_err_to_name(error));
					schedule_wifi_connect(WIFI_RETRY_DELAY_MS);
				}
			}
		}

		if (!(__atomic_load_n(&device_status, __ATOMIC_ACQUIRE) &
		      VIRTIO_STATUS_DRIVER_OK))
			continue;

		while (process_one_tx())
			;

		for (;;) {
			if (pending_rx == NULL) {
				if (xQueueReceive(rx_ready_queue, &pending_rx, 0) !=
				    pdTRUE)
					break;
			}
			if (!process_one_rx(pending_rx))
				break;
			xQueueSend(rx_free_queue, &pending_rx, 0);
			pending_rx = NULL;
		}

		if (!packet_path_reported &&
		    (__atomic_load_n(&rx_frame_count, __ATOMIC_RELAXED) != 0u ||
		     __atomic_load_n(&tx_frame_count, __ATOMIC_RELAXED) != 0u ||
		     __atomic_load_n(&rx_drop_count, __ATOMIC_RELAXED) != 0u ||
		     __atomic_load_n(&tx_error_count, __ATOMIC_RELAXED) != 0u)) {
			packet_path_reported = true;
			ESP_LOGI(TAG,
				 "packet path active: rx=%" PRIu32 " tx=%" PRIu32
				 " rx_drop=%" PRIu32 " tx_error=%" PRIu32,
				 __atomic_load_n(&rx_frame_count, __ATOMIC_RELAXED),
				 __atomic_load_n(&tx_frame_count, __ATOMIC_RELAXED),
				 __atomic_load_n(&rx_drop_count, __ATOMIC_RELAXED),
				 __atomic_load_n(&tx_error_count, __ATOMIC_RELAXED));
		}
	}
}

int virtio_net_bridge_init(void)
{
	esp_err_t error;

	memset(queues, 0, sizeof(queues));
	rx_ready_queue = xQueueCreateStatic(RX_SOFTWARE_QUEUE_DEPTH,
					    sizeof(struct rx_packet *),
					    rx_ready_queue_storage,
					    &rx_ready_queue_control);
	rx_free_queue = xQueueCreateStatic(RX_SOFTWARE_QUEUE_DEPTH,
					   sizeof(struct rx_packet *),
					   rx_free_queue_storage,
					   &rx_free_queue_control);
	if (rx_ready_queue == NULL || rx_free_queue == NULL)
		return -1;
	for (size_t i = 0; i < RX_SOFTWARE_QUEUE_DEPTH; i++) {
		struct rx_packet *packet = &rx_packet_pool[i];
		if (xQueueSend(rx_free_queue, &packet, 0) != pdTRUE)
			return -1;
	}

	error = esp_event_loop_create_default();
	if (error != ESP_OK && error != ESP_ERR_INVALID_STATE)
		return -1;
	if (esp_event_handler_register(ESP_HOSTED_EVENT, ESP_EVENT_ANY_ID,
				       hosted_event_handler, NULL) != ESP_OK ||
	    esp_event_handler_register(WIFI_EVENT, ESP_EVENT_ANY_ID,
				       wifi_event_handler, NULL) != ESP_OK)
		return -1;

	if (xTaskCreatePinnedToCore(bridge_task, "rv32-net", 8192, NULL,
				    BRIDGE_TASK_PRIORITY,
				    &bridge_task_handle, 1) != pdPASS)
		return -1;
	return 0;
}

bool virtio_net_bridge_contains(uint32_t address, size_t width)
{
	return width != 0 && width <= sizeof(uint32_t) &&
		address >= VIRTIO_NET_GUEST_BASE &&
		address - VIRTIO_NET_GUEST_BASE <= VIRTIO_NET_GUEST_SIZE - width;
}

static uint32_t config_load(uint32_t offset, size_t width)
{
	uint8_t config[8];
	memcpy(config, station_mac, sizeof(station_mac));
	uint16_t status =
		__atomic_load_n(&station_connected, __ATOMIC_ACQUIRE) ?
		VIRTIO_NET_S_LINK_UP : 0;
	memcpy(config + 6u, &status, sizeof(status));

	if (offset >= sizeof(config) || width > sizeof(config) - offset)
		return 0;
	uint32_t value = 0;
	memcpy(&value, config + offset, width);
	return value;
}

uint32_t virtio_net_bridge_load(uint32_t address, size_t width)
{
	uint32_t offset = address - VIRTIO_NET_GUEST_BASE;
	struct virtio_queue_state *queue =
		queue_sel < VIRTIO_NET_QUEUE_COUNT ? &queues[queue_sel] : NULL;

	if (offset >= VIRTIO_MMIO_CONFIG)
		return config_load(offset - VIRTIO_MMIO_CONFIG, width);
	if (width != sizeof(uint32_t))
		return 0;

	switch (offset) {
	case VIRTIO_MMIO_MAGIC_VALUE: return 0x74726976u; /* "virt" */
	case VIRTIO_MMIO_VERSION: return 2u;
	case VIRTIO_MMIO_DEVICE_ID: return 1u; /* network */
	case VIRTIO_MMIO_VENDOR_ID: return 0x505345u; /* "ESP" */
	case VIRTIO_MMIO_DEVICE_FEATURES:
		if (device_features_sel == 0)
			return (1u << VIRTIO_NET_F_MAC) |
			       (1u << VIRTIO_NET_F_STATUS);
		if (device_features_sel == 1)
			return 1u << (VIRTIO_F_VERSION_1 - 32u);
		return 0;
	case VIRTIO_MMIO_QUEUE_NUM_MAX:
		return queue ? VIRTIO_NET_QUEUE_MAX : 0;
	case VIRTIO_MMIO_QUEUE_READY:
		return queue ? __atomic_load_n(&queue->ready,
						       __ATOMIC_ACQUIRE) : 0;
	case VIRTIO_MMIO_INTERRUPT_STATUS:
		return __atomic_load_n(&interrupt_status, __ATOMIC_ACQUIRE);
	case VIRTIO_MMIO_STATUS:
		return __atomic_load_n(&device_status, __ATOMIC_ACQUIRE);
	case VIRTIO_MMIO_CONFIG_GENERATION:
		return __atomic_load_n(&config_generation, __ATOMIC_RELAXED);
	default: return 0;
	}
}

static void reset_device(void)
{
	for (size_t i = 0; i < VIRTIO_NET_QUEUE_COUNT; i++)
		__atomic_store_n(&queues[i].ready, 0, __ATOMIC_RELEASE);
	__atomic_thread_fence(__ATOMIC_SEQ_CST);
	memset(queues, 0, sizeof(queues));
	memset(driver_features, 0, sizeof(driver_features));
	device_features_sel = 0;
	driver_features_sel = 0;
	queue_sel = 0;
	__atomic_store_n(&device_status, 0, __ATOMIC_RELEASE);
	__atomic_store_n(&interrupt_status, 0, __ATOMIC_RELEASE);
}

void virtio_net_bridge_store(uint32_t address, uint32_t value, size_t width)
{
	uint32_t offset = address - VIRTIO_NET_GUEST_BASE;
	struct virtio_queue_state *queue =
		queue_sel < VIRTIO_NET_QUEUE_COUNT ? &queues[queue_sel] : NULL;

	if (width != sizeof(uint32_t) || offset >= VIRTIO_MMIO_CONFIG)
		return;

	switch (offset) {
	case VIRTIO_MMIO_DEVICE_FEATURES_SEL: device_features_sel = value; break;
	case VIRTIO_MMIO_DRIVER_FEATURES_SEL: driver_features_sel = value; break;
	case VIRTIO_MMIO_DRIVER_FEATURES:
		if (driver_features_sel < 2u)
			driver_features[driver_features_sel] = value;
		break;
	case VIRTIO_MMIO_QUEUE_SEL:
		queue_sel = value;
		break;
	case VIRTIO_MMIO_QUEUE_NUM:
		if (queue && value <= VIRTIO_NET_QUEUE_MAX)
			queue->size = value;
		break;
	case VIRTIO_MMIO_QUEUE_READY:
		if (queue)
			__atomic_store_n(&queue->ready, value & 1u,
					 __ATOMIC_RELEASE);
		break;
	case VIRTIO_MMIO_QUEUE_DESC_LOW:
		if (queue) queue->desc = (queue->desc & 0xffffffff00000000ULL) | value;
		break;
	case VIRTIO_MMIO_QUEUE_DESC_HIGH:
		if (queue) queue->desc = (queue->desc & 0xffffffffULL) |
					 ((uint64_t)value << 32);
		break;
	case VIRTIO_MMIO_QUEUE_AVAIL_LOW:
		if (queue) queue->avail = (queue->avail & 0xffffffff00000000ULL) | value;
		break;
	case VIRTIO_MMIO_QUEUE_AVAIL_HIGH:
		if (queue) queue->avail = (queue->avail & 0xffffffffULL) |
					  ((uint64_t)value << 32);
		break;
	case VIRTIO_MMIO_QUEUE_USED_LOW:
		if (queue) queue->used = (queue->used & 0xffffffff00000000ULL) | value;
		break;
	case VIRTIO_MMIO_QUEUE_USED_HIGH:
		if (queue) queue->used = (queue->used & 0xffffffffULL) |
					 ((uint64_t)value << 32);
		break;
	case VIRTIO_MMIO_QUEUE_NOTIFY:
		if (value < VIRTIO_NET_QUEUE_COUNT && bridge_task_handle)
			xTaskNotifyGive(bridge_task_handle);
		break;
	case VIRTIO_MMIO_INTERRUPT_ACK:
		__atomic_fetch_and(&interrupt_status, ~value, __ATOMIC_RELEASE);
		break;
	case VIRTIO_MMIO_STATUS:
		if (value == 0)
			reset_device();
		else
			__atomic_store_n(&device_status, value, __ATOMIC_RELEASE);
		if ((value & VIRTIO_STATUS_DRIVER_OK) && bridge_task_handle)
			xTaskNotifyGive(bridge_task_handle);
		break;
	default:
		break;
	}
}

bool virtio_net_bridge_irq_pending(void)
{
	return __atomic_load_n(&interrupt_status, __ATOMIC_ACQUIRE) != 0;
}
