/*
 * ESP32-P4 SDMMC slot 0 to RV32 Linux virtio-blk bridge.
 *
 * Linux uses its unmodified virtio-mmio block driver. The physical card is
 * owned exclusively by the guest; ESP-IDF never mounts its filesystem. Slow
 * SD transfers run in a FreeRTOS worker on CPU1 rather than in the emulator's
 * MMIO path on CPU0.
 *
 * SPDX-License-Identifier: BSD-3-Clause
 */

#include "virtio_blk_bridge.h"

#include <inttypes.h>
#include <stdbool.h>
#include <stdint.h>
#include <stdio.h>
#include <string.h>

#include "driver/sdmmc_host.h"
#include "esp_cache.h"
#include "esp_err.h"
#include "esp_heap_caps.h"
#include "esp_ldo_regulator.h"
#include "esp_log.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "psram.h"
#include "sdkconfig.h"
#include "sdmmc_cmd.h"

#define TAG "rv32-sd"

#define VIRTIO_MMIO_MAGIC_VALUE          0x000u
#define VIRTIO_MMIO_VERSION              0x004u
#define VIRTIO_MMIO_DEVICE_ID            0x008u
#define VIRTIO_MMIO_VENDOR_ID            0x00cu
#define VIRTIO_MMIO_DEVICE_FEATURES      0x010u
#define VIRTIO_MMIO_DEVICE_FEATURES_SEL  0x014u
#define VIRTIO_MMIO_DRIVER_FEATURES      0x020u
#define VIRTIO_MMIO_DRIVER_FEATURES_SEL  0x024u
#define VIRTIO_MMIO_QUEUE_SEL            0x030u
#define VIRTIO_MMIO_QUEUE_NUM_MAX        0x034u
#define VIRTIO_MMIO_QUEUE_NUM            0x038u
#define VIRTIO_MMIO_QUEUE_READY          0x044u
#define VIRTIO_MMIO_QUEUE_NOTIFY         0x050u
#define VIRTIO_MMIO_INTERRUPT_STATUS     0x060u
#define VIRTIO_MMIO_INTERRUPT_ACK        0x064u
#define VIRTIO_MMIO_STATUS               0x070u
#define VIRTIO_MMIO_QUEUE_DESC_LOW       0x080u
#define VIRTIO_MMIO_QUEUE_DESC_HIGH      0x084u
#define VIRTIO_MMIO_QUEUE_AVAIL_LOW      0x090u
#define VIRTIO_MMIO_QUEUE_AVAIL_HIGH     0x094u
#define VIRTIO_MMIO_QUEUE_USED_LOW       0x0a0u
#define VIRTIO_MMIO_QUEUE_USED_HIGH      0x0a4u
#define VIRTIO_MMIO_CONFIG_GENERATION    0x0fcu
#define VIRTIO_MMIO_CONFIG               0x100u

#define VIRTIO_MMIO_INT_VRING            (1u << 0)
#define VIRTIO_STATUS_DRIVER_OK           (1u << 2)
#define VIRTIO_F_VERSION_1                32u

#define VIRTQ_DESC_F_NEXT                 1u
#define VIRTQ_DESC_F_WRITE                2u
#define VIRTQ_DESC_F_INDIRECT             4u
#define VIRTQ_AVAIL_F_NO_INTERRUPT        1u

#define VIRTIO_BLK_T_IN                   0u
#define VIRTIO_BLK_T_OUT                  1u
#define VIRTIO_BLK_T_FLUSH                4u
#define VIRTIO_BLK_S_OK                   0u
#define VIRTIO_BLK_S_IOERR                1u
#define VIRTIO_BLK_S_UNSUPP               2u

#define VIRTIO_BLK_QUEUE_MAX              64u
#define VIRTIO_BLK_SECTOR_SIZE            512u
#define VIRTIO_BLK_BOUNCE_BYTES           8192u
#define VIRTIO_BLK_CACHE_LINE_BYTES        4096u
#define VIRTIO_BLK_CACHE_LINE_SECTORS      \
	(VIRTIO_BLK_CACHE_LINE_BYTES / VIRTIO_BLK_SECTOR_SIZE)
#define VIRTIO_BLK_READ_AHEAD_LINES        16u
#define VIRTIO_BLK_TASK_PRIORITY          18u
#define VIRTIO_BLK_TASK_STACK_BYTES       6144u
#define VIRTIO_BLK_MAX_REQUEST_BYTES      (1024u * 1024u)

#define WAVESHARE_SD_LDO_CHANNEL          4

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

struct virtio_blk_request_header {
	uint32_t type;
	uint32_t reserved;
	uint64_t sector;
} __attribute__((packed));

struct virtio_queue_state {
	uint32_t size;
	uint32_t ready;
	uint64_t desc;
	uint64_t avail;
	uint64_t used;
	uint16_t last_avail;
};

struct virtio_blk_chain {
	struct virtq_desc descriptors[VIRTIO_BLK_QUEUE_MAX];
	uint16_t count;
};

static struct virtio_queue_state queue_state;
static uint32_t device_features_sel;
static uint32_t driver_features_sel;
static uint32_t driver_features[2];
static uint32_t queue_sel;
static uint32_t device_status;
static uint32_t interrupt_status;
static bool card_ready;
static sdmmc_card_t sd_card;
static uint8_t *bounce_buffer;
static uint64_t *cache_tags;
static uint8_t *cache_data;
static size_t cache_line_count;
static uint64_t cache_last_miss_sector = UINT64_MAX;
static uint64_t cache_next_miss_sector = UINT64_MAX;
static struct virtio_blk_bridge_perf_stats perf_stats;
static TaskHandle_t worker_task_handle;
static esp_ldo_channel_handle_t sd_power_handle;

#if CONFIG_RV32_EMULATOR_PERF_STATS || CONFIG_RV32_CACHE_PERF_STATS
#define BLK_PERF_INC(field) \
	__atomic_add_fetch(&perf_stats.field, 1u, __ATOMIC_RELAXED)
#define BLK_PERF_ADD(field, value) \
	__atomic_add_fetch(&perf_stats.field, (uint32_t)(value), __ATOMIC_RELAXED)
#else
#define BLK_PERF_INC(field) do { } while (0)
#define BLK_PERF_ADD(field, value) do { (void)(value); } while (0)
#endif

static bool guest_range_valid(uint64_t address, size_t length)
{
	if ((address >> 32) != 0 || length == 0 || address < GUEST_DMA_BASE ||
	    address >= GUEST_PHYS_END)
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

static bool queue_snapshot(struct virtio_queue_state *queue)
{
	if (!__atomic_load_n(&queue_state.ready, __ATOMIC_ACQUIRE))
		return false;
	*queue = queue_state;
	return queue->size != 0 && queue->size <= VIRTIO_BLK_QUEUE_MAX &&
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
	return index < queue->size &&
	       guest_read(queue->desc + index * sizeof(*descriptor), descriptor,
			  sizeof(*descriptor));
}

static bool queue_pop_available(struct virtio_queue_state *queue,
				uint16_t *head)
{
	uint16_t available_index;
	uint16_t ring_entry;

	if (!queue_snapshot(queue) ||
	    !guest_read(queue->avail + 2u, &available_index,
			 sizeof(available_index)))
		return false;
	__atomic_thread_fence(__ATOMIC_ACQUIRE);
	if (queue_state.last_avail == available_index)
		return false;

	uint16_t slot = queue_state.last_avail % queue->size;
	if (!guest_read(queue->avail + 4u + slot * sizeof(uint16_t),
			&ring_entry, sizeof(ring_entry)) ||
	    ring_entry >= queue->size)
		return false;
	queue_state.last_avail++;
	*head = ring_entry;
	return true;
}

static bool queue_complete(const struct virtio_queue_state *queue,
			   uint16_t head, uint32_t length)
{
	struct virtq_used_elem element = { .id = head, .length = length };
	uint16_t used_index;
	uint16_t available_flags = 0;

	if (!guest_read(queue->used + 2u, &used_index, sizeof(used_index)))
		return false;
	uint16_t slot = used_index % queue->size;
	if (!guest_write(queue->used + 4u + slot * sizeof(element), &element,
			 sizeof(element)))
		return false;
	used_index++;
	__atomic_thread_fence(__ATOMIC_RELEASE);
	if (!guest_write(queue->used + 2u, &used_index, sizeof(used_index)))
		return false;
	__atomic_thread_fence(__ATOMIC_RELEASE);

	if (!guest_read(queue->avail, &available_flags, sizeof(available_flags)) ||
	    !(available_flags & VIRTQ_AVAIL_F_NO_INTERRUPT))
		__atomic_fetch_or(&interrupt_status, VIRTIO_MMIO_INT_VRING,
				  __ATOMIC_RELEASE);
	return true;
}

static bool read_chain(const struct virtio_queue_state *queue, uint16_t head,
		       struct virtio_blk_chain *chain)
{
	uint16_t index = head;

	chain->count = 0;
	for (uint32_t visited = 0; visited < queue->size; visited++) {
		struct virtq_desc *descriptor =
			&chain->descriptors[chain->count++];

		if (!queue_read_desc(queue, index, descriptor) ||
		    (descriptor->flags & VIRTQ_DESC_F_INDIRECT))
			return false;
		if (!(descriptor->flags & VIRTQ_DESC_F_NEXT))
			return chain->count >= 2u;
		index = descriptor->next;
	}
	return false;
}

static bool validate_data_descriptors(const struct virtio_blk_chain *chain,
				      bool device_writes,
				      size_t *total_length)
{
	size_t total = 0;

	for (uint16_t i = 1; i + 1u < chain->count; i++) {
		const struct virtq_desc *descriptor = &chain->descriptors[i];
		bool writable = (descriptor->flags & VIRTQ_DESC_F_WRITE) != 0;

		if (writable != device_writes || descriptor->length == 0 ||
		    !guest_range_valid(descriptor->address, descriptor->length) ||
		    descriptor->length > VIRTIO_BLK_MAX_REQUEST_BYTES - total)
			return false;
		total += descriptor->length;
	}
	*total_length = total;
	return true;
}

static bool copy_from_guest_data(const struct virtio_blk_chain *chain,
				 size_t offset, void *destination,
				 size_t length)
{
	uint8_t *output = destination;

	for (uint16_t i = 1; i + 1u < chain->count && length != 0; i++) {
		const struct virtq_desc *descriptor = &chain->descriptors[i];

		if (offset >= descriptor->length) {
			offset -= descriptor->length;
			continue;
		}
		size_t available = descriptor->length - offset;
		size_t chunk = available < length ? available : length;
		if (!guest_read(descriptor->address + offset, output, chunk))
			return false;
		output += chunk;
		length -= chunk;
		offset = 0;
	}
	return length == 0;
}

static bool copy_to_guest_data(const struct virtio_blk_chain *chain,
			       size_t offset, const void *source,
			       size_t length)
{
	const uint8_t *input = source;

	for (uint16_t i = 1; i + 1u < chain->count && length != 0; i++) {
		const struct virtq_desc *descriptor = &chain->descriptors[i];

		if (offset >= descriptor->length) {
			offset -= descriptor->length;
			continue;
		}
		size_t available = descriptor->length - offset;
		size_t chunk = available < length ? available : length;
		if (!guest_write(descriptor->address + offset, input, chunk))
			return false;
		input += chunk;
		length -= chunk;
		offset = 0;
	}
	return length == 0;
}

static bool block_cache_init(void)
{
	void *region;
	size_t region_size;
	size_t maximum_lines;
	size_t tag_bytes;
	size_t dma_alignment;
	uintptr_t data_start;

	if (!psram_get_host_cache_region(&region, &region_size) ||
	    region_size < VIRTIO_BLK_CACHE_LINE_BYTES * 2u)
		return false;

	/* Tags and data share Linux's unused PSRAM prefix.  Recalculate after
	 * aligning the data so every tag has a complete 4 KiB cache line. */
	maximum_lines = region_size /
		(VIRTIO_BLK_CACHE_LINE_BYTES + sizeof(*cache_tags));
	tag_bytes = maximum_lines * sizeof(*cache_tags);
	dma_alignment = esp_cache_get_line_size_by_addr(region);
	if (dma_alignment == 0u)
		dma_alignment = 64u;
	data_start = ((uintptr_t)region + tag_bytes + dma_alignment - 1u) /
		     dma_alignment * dma_alignment;
	cache_line_count = ((uintptr_t)region + region_size - data_start) /
		VIRTIO_BLK_CACHE_LINE_BYTES;
	if (cache_line_count == 0)
		return false;

	cache_tags = region;
	cache_data = (uint8_t *)data_start;
	memset(cache_tags, 0xff, cache_line_count * sizeof(*cache_tags));
	cache_last_miss_sector = UINT64_MAX;
	cache_next_miss_sector = UINT64_MAX;
	return true;
}

static uint8_t *block_cache_lookup(uint64_t line_sector)
{
	if (cache_line_count == 0)
		return NULL;
	size_t slot = (size_t)(line_sector / VIRTIO_BLK_CACHE_LINE_SECTORS) %
		cache_line_count;
	if (cache_tags[slot] != line_sector)
		return NULL;
	BLK_PERF_INC(cache_hits);
	return cache_data + slot * VIRTIO_BLK_CACHE_LINE_BYTES;
}

static uint8_t *block_cache_fill(uint64_t line_sector)
{
	if (cache_line_count == 0)
		return NULL;

	size_t slot = (size_t)(line_sector / VIRTIO_BLK_CACHE_LINE_SECTORS) %
		cache_line_count;
	uint8_t *line = cache_data + slot * VIRTIO_BLK_CACHE_LINE_BYTES;
	uint64_t remaining = sd_card.csd.capacity - line_sector;
	bool forward_local = cache_last_miss_sector != UINT64_MAX &&
		line_sector > cache_last_miss_sector &&
		line_sector - cache_last_miss_sector <=
			VIRTIO_BLK_READ_AHEAD_LINES *
			VIRTIO_BLK_CACHE_LINE_SECTORS;
	size_t lines = line_sector == cache_next_miss_sector || forward_local ?
		VIRTIO_BLK_READ_AHEAD_LINES : 1u;
	size_t contiguous_lines = cache_line_count - slot;
	size_t blocks;

	if (lines > contiguous_lines)
		lines = contiguous_lines;
	blocks = lines * VIRTIO_BLK_CACHE_LINE_SECTORS;
	if (remaining < blocks)
		blocks = (size_t)remaining;
	lines = (blocks + VIRTIO_BLK_CACHE_LINE_SECTORS - 1u) /
		VIRTIO_BLK_CACHE_LINE_SECTORS;

	BLK_PERF_INC(cache_misses);
	for (size_t index = 0; index < lines; index++)
		cache_tags[slot + index] = UINT64_MAX;

	/* ESP32-P4 SDMMC can DMA directly into aligned PSRAM.  Reading a detected
	 * sequential run into adjacent cache lines avoids one command and one
	 * bounce-buffer copy per 4 KiB while retaining small random cache lines. */
	if (blocks == 0 ||
	    sdmmc_read_sectors(&sd_card, line, (size_t)line_sector,
			       blocks) != ESP_OK)
	{
		cache_last_miss_sector = UINT64_MAX;
		cache_next_miss_sector = UINT64_MAX;
		return NULL;
	}

	if (blocks % VIRTIO_BLK_CACHE_LINE_SECTORS != 0u) {
		size_t valid_bytes = blocks * VIRTIO_BLK_SECTOR_SIZE;
		memset(line + valid_bytes, 0,
		       lines * VIRTIO_BLK_CACHE_LINE_BYTES - valid_bytes);
	}
	for (size_t index = 0; index < lines; index++)
		cache_tags[slot + index] = line_sector +
			index * VIRTIO_BLK_CACHE_LINE_SECTORS;
	cache_last_miss_sector = line_sector;
	cache_next_miss_sector = line_sector +
		lines * VIRTIO_BLK_CACHE_LINE_SECTORS;
	return line;
}

static void block_cache_invalidate(uint64_t first_sector, size_t blocks)
{
	if (cache_line_count == 0 || blocks == 0)
		return;

	uint64_t last_sector = first_sector + blocks - 1u;
	uint64_t line = first_sector &
		~(uint64_t)(VIRTIO_BLK_CACHE_LINE_SECTORS - 1u);
	uint64_t last_line = last_sector &
		~(uint64_t)(VIRTIO_BLK_CACHE_LINE_SECTORS - 1u);
	for (;;) {
		size_t slot = (size_t)(line / VIRTIO_BLK_CACHE_LINE_SECTORS) %
			cache_line_count;
		if (cache_tags[slot] == line)
			cache_tags[slot] = UINT64_MAX;
		if (line == last_line)
			break;
		line += VIRTIO_BLK_CACHE_LINE_SECTORS;
	}
}

static uint8_t transfer_read_cached(const struct virtio_blk_chain *chain,
				    uint64_t sector, size_t data_length)
{
	size_t offset = 0;

	while (offset < data_length) {
		uint64_t line_sector = sector &
			~(uint64_t)(VIRTIO_BLK_CACHE_LINE_SECTORS - 1u);
		size_t line_offset = (size_t)(sector - line_sector) *
			VIRTIO_BLK_SECTOR_SIZE;
		size_t chunk = VIRTIO_BLK_CACHE_LINE_BYTES - line_offset;
		if (chunk > data_length - offset)
			chunk = data_length - offset;

		uint8_t *line = block_cache_lookup(line_sector);
		if (line == NULL)
			line = block_cache_fill(line_sector);
		if (line == NULL) {
			ESP_LOGE(TAG, "read failed at sector %" PRIu64, sector);
			return VIRTIO_BLK_S_IOERR;
		}
		if (!copy_to_guest_data(chain, offset, line + line_offset, chunk))
			return VIRTIO_BLK_S_IOERR;
		offset += chunk;
		sector += chunk / VIRTIO_BLK_SECTOR_SIZE;
	}
	return VIRTIO_BLK_S_OK;
}

static uint8_t transfer_request(const struct virtio_blk_chain *chain,
				const struct virtio_blk_request_header *header,
				size_t data_length)
{
	if (header->type == VIRTIO_BLK_T_FLUSH)
		return data_length == 0 ? VIRTIO_BLK_S_OK : VIRTIO_BLK_S_IOERR;
	if (header->type != VIRTIO_BLK_T_IN &&
	    header->type != VIRTIO_BLK_T_OUT)
		return VIRTIO_BLK_S_UNSUPP;
	if (data_length == 0 || data_length % VIRTIO_BLK_SECTOR_SIZE != 0)
		return VIRTIO_BLK_S_IOERR;

	uint64_t sector_count = data_length / VIRTIO_BLK_SECTOR_SIZE;
	uint64_t capacity = sd_card.csd.capacity;
	if (header->sector > capacity || sector_count > capacity - header->sector)
		return VIRTIO_BLK_S_IOERR;
	if (header->type == VIRTIO_BLK_T_IN && cache_line_count != 0)
		return transfer_read_cached(chain, header->sector, data_length);

	size_t offset = 0;
	uint64_t sector = header->sector;
	while (offset < data_length) {
		size_t chunk = data_length - offset;
		if (chunk > VIRTIO_BLK_BOUNCE_BYTES)
			chunk = VIRTIO_BLK_BOUNCE_BYTES;
		size_t blocks = chunk / VIRTIO_BLK_SECTOR_SIZE;
		esp_err_t error;

		if (header->type == VIRTIO_BLK_T_OUT) {
			if (!copy_from_guest_data(chain, offset, bounce_buffer, chunk))
				return VIRTIO_BLK_S_IOERR;
			error = sdmmc_write_sectors(&sd_card, bounce_buffer,
						  (size_t)sector, blocks);
			if (error == ESP_OK)
				block_cache_invalidate(sector, blocks);
		} else {
			error = sdmmc_read_sectors(&sd_card, bounce_buffer,
						 (size_t)sector, blocks);
			if (error == ESP_OK &&
			    !copy_to_guest_data(chain, offset, bounce_buffer, chunk))
				return VIRTIO_BLK_S_IOERR;
		}
		if (error != ESP_OK) {
			ESP_LOGE(TAG, "%s failed at sector %" PRIu64 ": %s",
				 header->type == VIRTIO_BLK_T_OUT ? "write" : "read",
				 sector, esp_err_to_name(error));
			return VIRTIO_BLK_S_IOERR;
		}
		offset += chunk;
		sector += blocks;
	}
	return VIRTIO_BLK_S_OK;
}

static void process_one_request(const struct virtio_queue_state *queue,
				uint16_t head)
{
	struct virtio_blk_chain chain;
	struct virtio_blk_request_header header;
	uint8_t status = VIRTIO_BLK_S_IOERR;
	uint32_t used_length = 0;
	size_t data_length = 0;

	if (!read_chain(queue, head, &chain)) {
		(void)queue_complete(queue, head, 0);
		return;
	}
	const struct virtq_desc *header_descriptor = &chain.descriptors[0];
	const struct virtq_desc *status_descriptor =
		&chain.descriptors[chain.count - 1u];
	if ((header_descriptor->flags & VIRTQ_DESC_F_WRITE) ||
	    header_descriptor->length < sizeof(header) ||
	    !guest_read(header_descriptor->address, &header, sizeof(header)) ||
	    !(status_descriptor->flags & VIRTQ_DESC_F_WRITE) ||
	    status_descriptor->length < 1u ||
	    !guest_range_valid(status_descriptor->address, 1u)) {
		(void)queue_complete(queue, head, 0);
		return;
	}

	bool device_writes = header.type == VIRTIO_BLK_T_IN;
	if (!validate_data_descriptors(&chain, device_writes, &data_length))
		status = VIRTIO_BLK_S_IOERR;
	else
		status = transfer_request(&chain, &header, data_length);
	BLK_PERF_INC(requests);
	if (status == VIRTIO_BLK_S_OK) {
		if (header.type == VIRTIO_BLK_T_IN)
			BLK_PERF_ADD(read_bytes, data_length);
		else if (header.type == VIRTIO_BLK_T_OUT)
			BLK_PERF_ADD(write_bytes, data_length);
	}

	if (guest_write(status_descriptor->address, &status, sizeof(status))) {
		used_length = 1u;
		if (header.type == VIRTIO_BLK_T_IN && status == VIRTIO_BLK_S_OK)
			used_length += (uint32_t)data_length;
	}
	(void)queue_complete(queue, head, used_length);
}

static void worker_task(void *argument)
{
	(void)argument;
	for (;;) {
		ulTaskNotifyTake(pdTRUE, portMAX_DELAY);
		if (!card_ready ||
		    !(__atomic_load_n(&device_status, __ATOMIC_ACQUIRE) &
		      VIRTIO_STATUS_DRIVER_OK))
			continue;

		for (;;) {
			struct virtio_queue_state queue;
			uint16_t head;

			if (!queue_pop_available(&queue, &head))
				break;
			process_one_request(&queue, head);
		}
	}
}

static void reset_device(void)
{
	__atomic_store_n(&queue_state.ready, 0u, __ATOMIC_RELEASE);
	__atomic_thread_fence(__ATOMIC_SEQ_CST);
	memset(&queue_state, 0, sizeof(queue_state));
	memset(driver_features, 0, sizeof(driver_features));
	device_features_sel = 0;
	driver_features_sel = 0;
	queue_sel = 0;
	__atomic_store_n(&device_status, 0u, __ATOMIC_RELEASE);
	__atomic_store_n(&interrupt_status, 0u, __ATOMIC_RELEASE);
}

int virtio_blk_bridge_init(void)
{
	const esp_ldo_channel_config_t power_config = {
		.chan_id = WAVESHARE_SD_LDO_CHANNEL,
		.voltage_mv = 3300,
	};
	sdmmc_host_t host = SDMMC_HOST_DEFAULT();
	sdmmc_slot_config_t slot = SDMMC_SLOT_CONFIG_DEFAULT();
	esp_err_t error;

	reset_device();
	error = esp_ldo_acquire_channel(&power_config, &sd_power_handle);
	if (error != ESP_OK) {
		ESP_LOGE(TAG, "could not enable SD power: %s",
			 esp_err_to_name(error));
		return -1;
	}

	host.slot = SDMMC_HOST_SLOT_0;
	host.max_freq_khz = SDMMC_FREQ_HIGHSPEED;
	slot.width = 4;
	slot.cd = SDMMC_SLOT_NO_CD;
	slot.wp = SDMMC_SLOT_NO_WP;
	slot.flags = 0;

	error = sdmmc_host_init();
	if (error == ESP_OK)
		error = sdmmc_host_init_slot(SDMMC_HOST_SLOT_0, &slot);
	if (error == ESP_OK)
		error = sdmmc_card_init(&host, &sd_card);
	if (error != ESP_OK) {
		ESP_LOGW(TAG, "microSD unavailable on slot 0: %s",
			 esp_err_to_name(error));
		return -1;
	}
	if (sd_card.csd.sector_size != VIRTIO_BLK_SECTOR_SIZE) {
		ESP_LOGE(TAG, "unsupported card sector size: %u",
			 (unsigned)sd_card.csd.sector_size);
		return -1;
	}

	bounce_buffer = heap_caps_aligned_alloc(
		64u, VIRTIO_BLK_BOUNCE_BYTES, MALLOC_CAP_INTERNAL | MALLOC_CAP_DMA);
	if (bounce_buffer == NULL) {
		ESP_LOGE(TAG, "could not allocate %u-byte DMA buffer",
			 VIRTIO_BLK_BOUNCE_BYTES);
		return -1;
	}
	if (block_cache_init())
		printf("virtio-blk: %zu KiB host-PSRAM read cache, %zu x %u-byte lines\n",
		       cache_line_count * VIRTIO_BLK_CACHE_LINE_BYTES / 1024u,
		       cache_line_count, VIRTIO_BLK_CACHE_LINE_BYTES);
	else
		ESP_LOGW(TAG, "host-PSRAM read cache unavailable");
	card_ready = true;
	BaseType_t worker_core = CONFIG_FREERTOS_NUMBER_OF_CORES > 1 ? 1 : 0;
	if (xTaskCreatePinnedToCore(worker_task, "rv32-sd",
				    VIRTIO_BLK_TASK_STACK_BYTES, NULL,
				    VIRTIO_BLK_TASK_PRIORITY,
				    &worker_task_handle, worker_core) != pdPASS) {
		card_ready = false;
		heap_caps_free(bounce_buffer);
		bounce_buffer = NULL;
		return -1;
	}

	sdmmc_card_print_info(stdout, &sd_card);
	printf("virtio-blk: microSD %" PRIu64
	       " MiB at 0x%08x, PLIC source %u; I/O worker on CPU%d\n",
	       ((uint64_t)sd_card.csd.capacity * VIRTIO_BLK_SECTOR_SIZE) >> 20,
	       VIRTIO_BLK_GUEST_BASE, VIRTIO_BLK_PLIC_SOURCE,
	       (int)worker_core);
	return 0;
}

bool virtio_blk_bridge_contains(uint32_t address, size_t width)
{
	return width != 0 && width <= sizeof(uint32_t) &&
	       address >= VIRTIO_BLK_GUEST_BASE &&
	       address - VIRTIO_BLK_GUEST_BASE <= VIRTIO_BLK_GUEST_SIZE - width;
}

static uint32_t config_load(uint32_t offset, size_t width)
{
	uint64_t capacity = card_ready ? sd_card.csd.capacity : 0;
	uint32_t value = 0;

	if (offset >= sizeof(capacity) || width > sizeof(capacity) - offset)
		return 0;
	memcpy(&value, (const uint8_t *)&capacity + offset, width);
	return value;
}

uint32_t virtio_blk_bridge_load(uint32_t address, size_t width)
{
	uint32_t offset = address - VIRTIO_BLK_GUEST_BASE;

	if (offset >= VIRTIO_MMIO_CONFIG)
		return config_load(offset - VIRTIO_MMIO_CONFIG, width);
	if (width != sizeof(uint32_t))
		return 0;

	switch (offset) {
	case VIRTIO_MMIO_MAGIC_VALUE: return 0x74726976u; /* "virt" */
	case VIRTIO_MMIO_VERSION: return 2u;
	case VIRTIO_MMIO_DEVICE_ID: return card_ready ? 2u : 0u;
	case VIRTIO_MMIO_VENDOR_ID: return 0x505345u; /* "ESP" */
	case VIRTIO_MMIO_DEVICE_FEATURES:
		if (device_features_sel == 1u)
			return 1u << (VIRTIO_F_VERSION_1 - 32u);
		return 0;
	case VIRTIO_MMIO_QUEUE_NUM_MAX:
		return queue_sel == 0u ? VIRTIO_BLK_QUEUE_MAX : 0u;
	case VIRTIO_MMIO_QUEUE_READY:
		return queue_sel == 0u ?
			__atomic_load_n(&queue_state.ready, __ATOMIC_ACQUIRE) : 0;
	case VIRTIO_MMIO_INTERRUPT_STATUS:
		return __atomic_load_n(&interrupt_status, __ATOMIC_ACQUIRE);
	case VIRTIO_MMIO_STATUS:
		return __atomic_load_n(&device_status, __ATOMIC_ACQUIRE);
	case VIRTIO_MMIO_CONFIG_GENERATION: return 0;
	default: return 0;
	}
}

void virtio_blk_bridge_store(uint32_t address, uint32_t value, size_t width)
{
	uint32_t offset = address - VIRTIO_BLK_GUEST_BASE;
	struct virtio_queue_state *queue = queue_sel == 0u ? &queue_state : NULL;

	if (width != sizeof(uint32_t) || offset >= VIRTIO_MMIO_CONFIG)
		return;

	switch (offset) {
	case VIRTIO_MMIO_DEVICE_FEATURES_SEL: device_features_sel = value; break;
	case VIRTIO_MMIO_DRIVER_FEATURES_SEL: driver_features_sel = value; break;
	case VIRTIO_MMIO_DRIVER_FEATURES:
		if (driver_features_sel < 2u)
			driver_features[driver_features_sel] = value;
		break;
	case VIRTIO_MMIO_QUEUE_SEL: queue_sel = value; break;
	case VIRTIO_MMIO_QUEUE_NUM:
		if (queue && value != 0 && value <= VIRTIO_BLK_QUEUE_MAX)
			queue->size = value;
		break;
	case VIRTIO_MMIO_QUEUE_READY:
		if (queue)
			__atomic_store_n(&queue->ready, value & 1u,
					 __ATOMIC_RELEASE);
		break;
	case VIRTIO_MMIO_QUEUE_DESC_LOW:
		if (queue)
			queue->desc = (queue->desc & 0xffffffff00000000ULL) | value;
		break;
	case VIRTIO_MMIO_QUEUE_DESC_HIGH:
		if (queue)
			queue->desc = (queue->desc & 0xffffffffULL) |
				      ((uint64_t)value << 32);
		break;
	case VIRTIO_MMIO_QUEUE_AVAIL_LOW:
		if (queue)
			queue->avail = (queue->avail & 0xffffffff00000000ULL) | value;
		break;
	case VIRTIO_MMIO_QUEUE_AVAIL_HIGH:
		if (queue)
			queue->avail = (queue->avail & 0xffffffffULL) |
				       ((uint64_t)value << 32);
		break;
	case VIRTIO_MMIO_QUEUE_USED_LOW:
		if (queue)
			queue->used = (queue->used & 0xffffffff00000000ULL) | value;
		break;
	case VIRTIO_MMIO_QUEUE_USED_HIGH:
		if (queue)
			queue->used = (queue->used & 0xffffffffULL) |
				      ((uint64_t)value << 32);
		break;
	case VIRTIO_MMIO_QUEUE_NOTIFY:
		if (value == 0u && worker_task_handle)
			xTaskNotifyGive(worker_task_handle);
		break;
	case VIRTIO_MMIO_INTERRUPT_ACK:
		__atomic_fetch_and(&interrupt_status, ~value, __ATOMIC_RELEASE);
		break;
	case VIRTIO_MMIO_STATUS:
		if (value == 0)
			reset_device();
		else {
			__atomic_store_n(&device_status, value, __ATOMIC_RELEASE);
			if ((value & VIRTIO_STATUS_DRIVER_OK) && worker_task_handle)
				xTaskNotifyGive(worker_task_handle);
		}
		break;
	default: break;
	}
}

bool virtio_blk_bridge_irq_pending(void)
{
	return __atomic_load_n(&interrupt_status, __ATOMIC_ACQUIRE) != 0;
}

void virtio_blk_bridge_perf_snapshot(
	struct virtio_blk_bridge_perf_stats *stats)
{
	if (stats == NULL)
		return;

#define BLK_PERF_LOAD(field) \
	stats->field = __atomic_load_n(&perf_stats.field, __ATOMIC_RELAXED)
	BLK_PERF_LOAD(cache_hits);
	BLK_PERF_LOAD(cache_misses);
	BLK_PERF_LOAD(read_bytes);
	BLK_PERF_LOAD(write_bytes);
	BLK_PERF_LOAD(requests);
#undef BLK_PERF_LOAD
}
