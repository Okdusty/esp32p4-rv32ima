/*
 * Standard virtio-gpu 2D scanout backed by the ESP32-P4 display bridge.
 *
 * The device intentionally implements only the upstream 2D/KMS protocol.
 * Linux draws XRGB8888 resources with its normal DRM stack. A CPU1 worker
 * gathers dirty guest pages and lets the display bridge/PPA convert them to
 * the physical backend's RGB565 scanout. No guest-private graphics ABI or
 * guest-PC-specific shortcut is involved.
 *
 * SPDX-License-Identifier: BSD-3-Clause
 */

#include "virtio_gpu_bridge.h"

#include <stdbool.h>
#include <stdint.h>
#include <stdio.h>
#include <string.h>

#include "display_bridge.h"
#include "esp_heap_caps.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "psram.h"

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
#define VIRTIO_RING_F_INDIRECT_DESC       28u

#define VIRTQ_DESC_F_NEXT                 1u
#define VIRTQ_DESC_F_WRITE                2u
#define VIRTQ_DESC_F_INDIRECT             4u
#define VIRTQ_AVAIL_F_NO_INTERRUPT        1u

#define VIRTIO_GPU_CONTROL_QUEUE          0u
#define VIRTIO_GPU_CURSOR_QUEUE           1u
#define VIRTIO_GPU_QUEUE_COUNT            2u
#define VIRTIO_GPU_QUEUE_MAX             64u
#define VIRTIO_GPU_INDIRECT_MAX        1024u
#define VIRTIO_GPU_RESOURCE_MAX          32u
#define VIRTIO_GPU_BACKING_MAX         1024u
#define VIRTIO_GPU_TASK_PRIORITY         18u
#define VIRTIO_GPU_TASK_STACK_BYTES    8192u
#define VIRTIO_GPU_SERVICE_BUDGET        32u
#define VIRTIO_GPU_DEVICE_ID_VALUE       16u

#define VIRTIO_GPU_FLAG_FENCE             (1u << 0)
#define VIRTIO_GPU_FLAG_INFO_RING_IDX     (1u << 1)

#define VIRTIO_GPU_CMD_GET_DISPLAY_INFO          0x0100u
#define VIRTIO_GPU_CMD_RESOURCE_CREATE_2D        0x0101u
#define VIRTIO_GPU_CMD_RESOURCE_UNREF             0x0102u
#define VIRTIO_GPU_CMD_SET_SCANOUT                0x0103u
#define VIRTIO_GPU_CMD_RESOURCE_FLUSH             0x0104u
#define VIRTIO_GPU_CMD_TRANSFER_TO_HOST_2D        0x0105u
#define VIRTIO_GPU_CMD_RESOURCE_ATTACH_BACKING    0x0106u
#define VIRTIO_GPU_CMD_RESOURCE_DETACH_BACKING    0x0107u
#define VIRTIO_GPU_CMD_UPDATE_CURSOR              0x0300u
#define VIRTIO_GPU_CMD_MOVE_CURSOR                0x0301u

#define VIRTIO_GPU_RESP_OK_NODATA                 0x1100u
#define VIRTIO_GPU_RESP_OK_DISPLAY_INFO           0x1101u
#define VIRTIO_GPU_RESP_ERR_UNSPEC                 0x1200u
#define VIRTIO_GPU_RESP_ERR_OUT_OF_MEMORY          0x1201u
#define VIRTIO_GPU_RESP_ERR_INVALID_SCANOUT_ID     0x1202u
#define VIRTIO_GPU_RESP_ERR_INVALID_RESOURCE_ID    0x1203u
#define VIRTIO_GPU_RESP_ERR_INVALID_PARAMETER      0x1205u

#define VIRTIO_GPU_FORMAT_B8G8R8A8_UNORM          1u
#define VIRTIO_GPU_FORMAT_B8G8R8X8_UNORM          2u
#define VIRTIO_GPU_MAX_SCANOUTS                   16u

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

struct virtio_gpu_ctrl_hdr {
	uint32_t type;
	uint32_t flags;
	uint64_t fence_id;
	uint32_t ctx_id;
	uint8_t ring_idx;
	uint8_t padding[3];
} __attribute__((packed));

struct virtio_gpu_rect {
	uint32_t x;
	uint32_t y;
	uint32_t width;
	uint32_t height;
} __attribute__((packed));

struct virtio_gpu_resource_create_2d {
	struct virtio_gpu_ctrl_hdr hdr;
	uint32_t resource_id;
	uint32_t format;
	uint32_t width;
	uint32_t height;
} __attribute__((packed));

struct virtio_gpu_resource_unref {
	struct virtio_gpu_ctrl_hdr hdr;
	uint32_t resource_id;
	uint32_t padding;
} __attribute__((packed));

struct virtio_gpu_set_scanout {
	struct virtio_gpu_ctrl_hdr hdr;
	struct virtio_gpu_rect rect;
	uint32_t scanout_id;
	uint32_t resource_id;
} __attribute__((packed));

struct virtio_gpu_resource_flush {
	struct virtio_gpu_ctrl_hdr hdr;
	struct virtio_gpu_rect rect;
	uint32_t resource_id;
	uint32_t padding;
} __attribute__((packed));

struct virtio_gpu_transfer_to_host_2d {
	struct virtio_gpu_ctrl_hdr hdr;
	struct virtio_gpu_rect rect;
	uint64_t offset;
	uint32_t resource_id;
	uint32_t padding;
} __attribute__((packed));

struct virtio_gpu_resource_attach_backing {
	struct virtio_gpu_ctrl_hdr hdr;
	uint32_t resource_id;
	uint32_t nr_entries;
} __attribute__((packed));

struct virtio_gpu_resource_detach_backing {
	struct virtio_gpu_ctrl_hdr hdr;
	uint32_t resource_id;
	uint32_t padding;
} __attribute__((packed));

struct virtio_gpu_mem_entry {
	uint64_t address;
	uint32_t length;
	uint32_t padding;
} __attribute__((packed));

struct virtio_gpu_update_cursor {
	struct virtio_gpu_ctrl_hdr hdr;
	uint32_t scanout_id;
	uint32_t x;
	uint32_t y;
	uint32_t position_padding;
	uint32_t resource_id;
	uint32_t hot_x;
	uint32_t hot_y;
	uint32_t padding;
} __attribute__((packed));

struct virtio_gpu_display_one {
	struct virtio_gpu_rect rect;
	uint32_t enabled;
	uint32_t flags;
} __attribute__((packed));

struct virtio_gpu_resp_display_info {
	struct virtio_gpu_ctrl_hdr hdr;
	struct virtio_gpu_display_one modes[VIRTIO_GPU_MAX_SCANOUTS];
} __attribute__((packed));

_Static_assert(sizeof(struct virtq_desc) == 16u,
	       "virtio descriptor layout must match the split-ring ABI");
_Static_assert(sizeof(struct virtq_used_elem) == 8u,
	       "virtio used-ring layout must match the split-ring ABI");
_Static_assert(sizeof(struct virtio_gpu_ctrl_hdr) == 24u,
	       "virtio-gpu control header layout mismatch");
_Static_assert(sizeof(struct virtio_gpu_mem_entry) == 16u,
	       "virtio-gpu backing entry layout mismatch");
_Static_assert(sizeof(struct virtio_gpu_update_cursor) == 56u,
	       "virtio-gpu cursor command layout mismatch");
_Static_assert(sizeof(struct virtio_gpu_resp_display_info) == 408u,
	       "virtio-gpu display response layout mismatch");

struct gpu_backing_entry {
	uint32_t address;
	uint32_t length;
};

struct gpu_resource {
	uint32_t id;
	uint32_t format;
	uint32_t width;
	uint32_t height;
	struct gpu_backing_entry *backing;
	uint32_t backing_count;
	uint64_t backing_size;
	struct virtio_gpu_rect transfer_rect;
	uint64_t transfer_offset;
	bool transfer_valid;
};

struct gpu_scanout {
	uint32_t resource_id;
	struct virtio_gpu_rect rect;
};

struct descriptor_cursor {
	uint64_t table;
	uint32_t count;
	uint32_t index;
	uint32_t visited;
	bool done;
};

struct descriptor_reader {
	struct descriptor_cursor cursor;
	struct virtq_desc descriptor;
	size_t offset;
	bool loaded;
};

struct backing_reader {
	const struct gpu_resource *resource;
	uint32_t entry_index;
	uint64_t entry_start;
};

static struct virtio_queue_state queues[VIRTIO_GPU_QUEUE_COUNT];
static struct gpu_resource resources[VIRTIO_GPU_RESOURCE_MAX];
static struct gpu_scanout scanout;
static uint32_t device_features_sel;
static uint32_t driver_features_sel;
static uint32_t driver_features[2];
static uint32_t queue_sel;
static uint32_t device_status;
static uint32_t interrupt_status;
static uint32_t config_events;
static bool device_ready;
static bool resources_reset_pending;
static TaskHandle_t worker_task_handle;

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

static bool queue_snapshot(uint32_t index, struct virtio_queue_state *queue)
{
	if (index >= VIRTIO_GPU_QUEUE_COUNT ||
	    !__atomic_load_n(&queues[index].ready, __ATOMIC_ACQUIRE))
		return false;
	*queue = queues[index];
	return queue->size != 0 && queue->size <= VIRTIO_GPU_QUEUE_MAX &&
	       guest_range_valid(queue->desc,
				 queue->size * sizeof(struct virtq_desc)) &&
	       guest_range_valid(queue->avail,
				 6u + queue->size * sizeof(uint16_t)) &&
	       guest_range_valid(queue->used,
				 6u + queue->size * sizeof(struct virtq_used_elem));
}

static bool queue_pop_available(uint32_t index,
				struct virtio_queue_state *queue, uint16_t *head)
{
	uint16_t available_index;
	uint16_t ring_entry;

	if (!queue_snapshot(index, queue) ||
	    !guest_read(queue->avail + 2u, &available_index,
			 sizeof(available_index)))
		return false;
	__atomic_thread_fence(__ATOMIC_ACQUIRE);
	if (queues[index].last_avail == available_index)
		return false;

	uint16_t slot = queues[index].last_avail % queue->size;
	if (!guest_read(queue->avail + 4u + slot * sizeof(uint16_t),
			&ring_entry, sizeof(ring_entry)) ||
	    ring_entry >= queue->size)
		return false;
	queues[index].last_avail++;
	*head = ring_entry;
	return true;
}

static bool queue_complete(uint32_t index,
			   const struct virtio_queue_state *queue,
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
	(void)index;
	return true;
}

static bool descriptor_cursor_init(const struct virtio_queue_state *queue,
				   uint16_t head,
				   struct descriptor_cursor *cursor)
{
	struct virtq_desc first;

	if (head >= queue->size ||
	    !guest_read(queue->desc + (uint64_t)head * sizeof(first), &first,
			 sizeof(first)))
		return false;
	memset(cursor, 0, sizeof(*cursor));
	if (first.flags & VIRTQ_DESC_F_INDIRECT) {
		if ((first.flags & VIRTQ_DESC_F_NEXT) ||
		    first.length == 0 || first.length % sizeof(first) != 0 ||
		    first.length / sizeof(first) > VIRTIO_GPU_INDIRECT_MAX ||
		    !guest_range_valid(first.address, first.length))
			return false;
		cursor->table = first.address;
		cursor->count = first.length / sizeof(first);
		cursor->index = 0;
	} else {
		cursor->table = queue->desc;
		cursor->count = queue->size;
		cursor->index = head;
	}
	return true;
}

static bool descriptor_cursor_next(struct descriptor_cursor *cursor,
				   struct virtq_desc *descriptor)
{
	if (cursor->done || cursor->index >= cursor->count ||
	    cursor->visited++ >= cursor->count ||
	    !guest_read(cursor->table +
			(cursor->index * sizeof(*descriptor)), descriptor,
			 sizeof(*descriptor)) ||
	    (descriptor->flags & VIRTQ_DESC_F_INDIRECT) ||
	    descriptor->length == 0 ||
	    !guest_range_valid(descriptor->address, descriptor->length))
		return false;
	if (descriptor->flags & VIRTQ_DESC_F_NEXT) {
		if (descriptor->next >= cursor->count)
			return false;
		cursor->index = descriptor->next;
	} else {
		cursor->done = true;
	}
	return true;
}

static bool chain_measure(const struct virtio_queue_state *queue,
			  uint16_t head, size_t *readable,
			  size_t *writable)
{
	struct descriptor_cursor cursor;
	struct virtq_desc descriptor;
	bool in_writable = false;

	*readable = 0;
	*writable = 0;
	if (!descriptor_cursor_init(queue, head, &cursor))
		return false;
	for (;;) {
		if (!descriptor_cursor_next(&cursor, &descriptor))
			return false;
		if (descriptor.flags & VIRTQ_DESC_F_WRITE) {
			in_writable = true;
			if (descriptor.length > SIZE_MAX - *writable)
				return false;
			*writable += descriptor.length;
		} else {
			if (in_writable || descriptor.length > SIZE_MAX - *readable)
				return false;
			*readable += descriptor.length;
		}
		if (cursor.done)
			return *readable != 0;
	}
}

static bool chain_read(const struct virtio_queue_state *queue, uint16_t head,
		       size_t offset, void *destination, size_t length)
{
	struct descriptor_cursor cursor;
	struct virtq_desc descriptor;
	uint8_t *output = destination;

	if (!descriptor_cursor_init(queue, head, &cursor))
		return false;
	while (length != 0) {
		if (!descriptor_cursor_next(&cursor, &descriptor) ||
		    (descriptor.flags & VIRTQ_DESC_F_WRITE))
			return false;
		if (offset >= descriptor.length) {
			offset -= descriptor.length;
			if (cursor.done)
				return false;
			continue;
		}
		size_t chunk = descriptor.length - offset;
		if (chunk > length)
			chunk = length;
		if (!guest_read(descriptor.address + offset, output, chunk))
			return false;
		output += chunk;
		length -= chunk;
		offset = 0;
		if (length != 0 && cursor.done)
			return false;
	}
	return true;
}

static bool descriptor_reader_load(struct descriptor_reader *reader)
{
	if (reader->loaded)
		return true;
	if (!descriptor_cursor_next(&reader->cursor, &reader->descriptor) ||
	    (reader->descriptor.flags & VIRTQ_DESC_F_WRITE))
		return false;
	reader->offset = 0;
	reader->loaded = true;
	return true;
}

static bool descriptor_reader_read(struct descriptor_reader *reader,
				   void *destination, size_t length)
{
	uint8_t *output = destination;

	while (length != 0) {
		if (!descriptor_reader_load(reader))
			return false;
		size_t available = reader->descriptor.length - reader->offset;
		size_t chunk = available < length ? available : length;

		if (!guest_read(reader->descriptor.address + reader->offset,
				output, chunk))
			return false;
		reader->offset += chunk;
		output += chunk;
		length -= chunk;
		if (reader->offset == reader->descriptor.length)
			reader->loaded = false;
	}
	return true;
}

static bool descriptor_reader_init(const struct virtio_queue_state *queue,
				   uint16_t head, size_t skip,
				   struct descriptor_reader *reader)
{
	uint8_t discarded[32];

	memset(reader, 0, sizeof(*reader));
	if (!descriptor_cursor_init(queue, head, &reader->cursor))
		return false;
	while (skip != 0) {
		size_t chunk = skip < sizeof(discarded) ? skip : sizeof(discarded);

		if (!descriptor_reader_read(reader, discarded, chunk))
			return false;
		skip -= chunk;
	}
	return true;
}

static bool chain_write_response(const struct virtio_queue_state *queue,
				 uint16_t head, const void *source,
				 size_t length)
{
	struct descriptor_cursor cursor;
	struct virtq_desc descriptor;
	const uint8_t *input = source;

	if (!descriptor_cursor_init(queue, head, &cursor))
		return false;
	while (length != 0) {
		if (!descriptor_cursor_next(&cursor, &descriptor))
			return false;
		if (!(descriptor.flags & VIRTQ_DESC_F_WRITE)) {
			if (cursor.done)
				return false;
			continue;
		}
		size_t chunk = descriptor.length < length ? descriptor.length : length;
		if (!guest_write(descriptor.address, input, chunk))
			return false;
		input += chunk;
		length -= chunk;
		if (length != 0 && cursor.done)
			return false;
	}
	return true;
}

static struct gpu_resource *resource_find(uint32_t id)
{
	if (id == 0)
		return NULL;
	for (uint32_t i = 0; i < VIRTIO_GPU_RESOURCE_MAX; i++)
		if (resources[i].id == id)
			return &resources[i];
	return NULL;
}

static struct gpu_resource *resource_allocate(uint32_t id)
{
	if (resource_find(id) != NULL)
		return NULL;
	for (uint32_t i = 0; i < VIRTIO_GPU_RESOURCE_MAX; i++)
		if (resources[i].id == 0) {
			resources[i].id = id;
			return &resources[i];
		}
	return NULL;
}

static void resource_release(struct gpu_resource *resource)
{
	if (resource == NULL)
		return;
	if (scanout.resource_id == resource->id)
		memset(&scanout, 0, sizeof(scanout));
	heap_caps_free(resource->backing);
	memset(resource, 0, sizeof(*resource));
}

static void resources_release_all(void)
{
	for (uint32_t i = 0; i < VIRTIO_GPU_RESOURCE_MAX; i++)
		resource_release(&resources[i]);
	memset(&scanout, 0, sizeof(scanout));
}

static bool rect_valid(const struct gpu_resource *resource,
		       const struct virtio_gpu_rect *rect)
{
	return rect->width != 0 && rect->height != 0 &&
	       rect->x < resource->width && rect->y < resource->height &&
	       rect->width <= resource->width - rect->x &&
	       rect->height <= resource->height - rect->y;
}

static bool rect_intersection(const struct virtio_gpu_rect *a,
			      const struct virtio_gpu_rect *b,
			      struct virtio_gpu_rect *result)
{
	uint32_t x1 = a->x > b->x ? a->x : b->x;
	uint32_t y1 = a->y > b->y ? a->y : b->y;
	uint32_t ax2 = a->x + a->width;
	uint32_t ay2 = a->y + a->height;
	uint32_t bx2 = b->x + b->width;
	uint32_t by2 = b->y + b->height;
	uint32_t x2 = ax2 < bx2 ? ax2 : bx2;
	uint32_t y2 = ay2 < by2 ? ay2 : by2;

	if (x1 >= x2 || y1 >= y2)
		return false;
	*result = (struct virtio_gpu_rect){
		.x = x1, .y = y1, .width = x2 - x1, .height = y2 - y1,
	};
	return true;
}

static bool backing_read(void *context, uint64_t offset, void *destination,
			 size_t length)
{
	struct backing_reader *reader = context;
	const struct gpu_resource *resource = reader->resource;
	uint8_t *output = destination;

	if (length == 0 || offset > resource->backing_size ||
	    length > resource->backing_size - offset)
		return false;
	if (offset < reader->entry_start) {
		reader->entry_index = 0;
		reader->entry_start = 0;
	}
	while (reader->entry_index < resource->backing_count &&
	       offset >= reader->entry_start +
			 resource->backing[reader->entry_index].length) {
		reader->entry_start +=
			resource->backing[reader->entry_index].length;
		reader->entry_index++;
	}
	while (length != 0 && reader->entry_index < resource->backing_count) {
		const struct gpu_backing_entry *entry =
			&resource->backing[reader->entry_index];
		size_t inside = (size_t)(offset - reader->entry_start);
		size_t chunk = entry->length - inside;

		if (chunk > length)
			chunk = length;
		if (!guest_read((uint64_t)entry->address + inside, output, chunk))
			return false;
		output += chunk;
		length -= chunk;
		offset += chunk;
		if (inside + chunk == entry->length) {
			reader->entry_start += entry->length;
			reader->entry_index++;
		}
	}
	return length == 0;
}

static void response_header(struct virtio_gpu_ctrl_hdr *response,
			    const struct virtio_gpu_ctrl_hdr *request,
			    uint32_t type)
{
	memset(response, 0, sizeof(*response));
	response->type = type;
	if (request->flags & VIRTIO_GPU_FLAG_FENCE) {
		response->flags = request->flags &
			(VIRTIO_GPU_FLAG_FENCE | VIRTIO_GPU_FLAG_INFO_RING_IDX);
		response->fence_id = request->fence_id;
		response->ctx_id = request->ctx_id;
		response->ring_idx = request->ring_idx;
	}
}

static uint32_t command_create_2d(
	const struct virtio_gpu_resource_create_2d *command)
{
	uint64_t bytes;
	struct gpu_resource *resource;

	if (command->resource_id == 0 || command->width == 0 ||
	    command->height == 0 || command->width > 4096u ||
	    command->height > 4096u ||
	    (command->format != VIRTIO_GPU_FORMAT_B8G8R8X8_UNORM &&
	     command->format != VIRTIO_GPU_FORMAT_B8G8R8A8_UNORM))
		return VIRTIO_GPU_RESP_ERR_INVALID_PARAMETER;
	bytes = (uint64_t)command->width * command->height * sizeof(uint32_t);
	if (bytes > GUEST_RAM_SIZE)
		return VIRTIO_GPU_RESP_ERR_INVALID_PARAMETER;
	resource = resource_allocate(command->resource_id);
	if (resource == NULL)
		return resource_find(command->resource_id) != NULL
			       ? VIRTIO_GPU_RESP_ERR_INVALID_RESOURCE_ID
			       : VIRTIO_GPU_RESP_ERR_OUT_OF_MEMORY;
	resource->format = command->format;
	resource->width = command->width;
	resource->height = command->height;
	return VIRTIO_GPU_RESP_OK_NODATA;
}

static uint32_t command_attach_backing(
	const struct virtio_queue_state *queue, uint16_t head,
	const struct virtio_gpu_resource_attach_backing *command,
	size_t readable)
{
	struct gpu_resource *resource = resource_find(command->resource_id);
	struct gpu_backing_entry *backing;
	struct descriptor_reader reader;
	uint64_t entries_bytes;
	uint64_t total = 0;

	if (resource == NULL)
		return VIRTIO_GPU_RESP_ERR_INVALID_RESOURCE_ID;
	entries_bytes = (uint64_t)command->nr_entries *
			sizeof(struct virtio_gpu_mem_entry);
	if (resource->backing != NULL || command->nr_entries == 0 ||
	    command->nr_entries > VIRTIO_GPU_BACKING_MAX ||
	    readable < sizeof(*command) ||
	    entries_bytes > readable - sizeof(*command) ||
	    !descriptor_reader_init(queue, head, sizeof(*command), &reader))
		return VIRTIO_GPU_RESP_ERR_INVALID_PARAMETER;
	backing = heap_caps_calloc(command->nr_entries, sizeof(*backing),
				   MALLOC_CAP_SPIRAM);
	if (backing == NULL)
		return VIRTIO_GPU_RESP_ERR_OUT_OF_MEMORY;

	for (uint32_t i = 0; i < command->nr_entries; i++) {
		struct virtio_gpu_mem_entry entry;

		if (!descriptor_reader_read(&reader, &entry, sizeof(entry)) ||
		    !guest_range_valid(entry.address, entry.length) ||
		    total + entry.length < total) {
			heap_caps_free(backing);
			return VIRTIO_GPU_RESP_ERR_INVALID_PARAMETER;
		}
		backing[i].address = (uint32_t)entry.address;
		backing[i].length = entry.length;
		total += entry.length;
	}
	if (total < (uint64_t)resource->width * resource->height *
			    sizeof(uint32_t)) {
		heap_caps_free(backing);
		return VIRTIO_GPU_RESP_ERR_INVALID_PARAMETER;
	}
	resource->backing = backing;
	resource->backing_count = command->nr_entries;
	resource->backing_size = total;
	return VIRTIO_GPU_RESP_OK_NODATA;
}

static uint32_t command_transfer(
	const struct virtio_gpu_transfer_to_host_2d *command)
{
	struct gpu_resource *resource = resource_find(command->resource_id);
	uint64_t final_byte;

	if (resource == NULL)
		return VIRTIO_GPU_RESP_ERR_INVALID_RESOURCE_ID;
	if (resource->backing == NULL || !rect_valid(resource, &command->rect))
		return VIRTIO_GPU_RESP_ERR_INVALID_PARAMETER;
	final_byte = command->offset +
		(uint64_t)(command->rect.height - 1u) * resource->width *
			sizeof(uint32_t) +
		(uint64_t)command->rect.width * sizeof(uint32_t);
	if (final_byte < command->offset || final_byte > resource->backing_size)
		return VIRTIO_GPU_RESP_ERR_INVALID_PARAMETER;
	resource->transfer_rect = command->rect;
	resource->transfer_offset = command->offset;
	resource->transfer_valid = true;
	return VIRTIO_GPU_RESP_OK_NODATA;
}

static uint32_t command_set_scanout(
	const struct virtio_gpu_set_scanout *command)
{
	struct gpu_resource *resource;

	if (command->scanout_id != 0)
		return VIRTIO_GPU_RESP_ERR_INVALID_SCANOUT_ID;
	if (command->resource_id == 0) {
		memset(&scanout, 0, sizeof(scanout));
		return VIRTIO_GPU_RESP_OK_NODATA;
	}
	resource = resource_find(command->resource_id);
	if (resource == NULL)
		return VIRTIO_GPU_RESP_ERR_INVALID_RESOURCE_ID;
	if (resource->format != VIRTIO_GPU_FORMAT_B8G8R8X8_UNORM ||
	    resource->backing == NULL || !rect_valid(resource, &command->rect) ||
	    command->rect.width != DISPLAY_FB_WIDTH ||
	    command->rect.height != DISPLAY_FB_HEIGHT)
		return VIRTIO_GPU_RESP_ERR_INVALID_PARAMETER;
	scanout.resource_id = command->resource_id;
	scanout.rect = command->rect;
	return VIRTIO_GPU_RESP_OK_NODATA;
}

static uint32_t command_flush(const struct virtio_gpu_resource_flush *command)
{
	struct gpu_resource *resource = resource_find(command->resource_id);
	struct virtio_gpu_rect visible;
	struct backing_reader reader;
	uint64_t offset;

	if (resource == NULL)
		return VIRTIO_GPU_RESP_ERR_INVALID_RESOURCE_ID;
	if (resource->backing == NULL || !rect_valid(resource, &command->rect))
		return VIRTIO_GPU_RESP_ERR_INVALID_PARAMETER;
	if (scanout.resource_id != resource->id ||
	    !rect_intersection(&command->rect, &scanout.rect, &visible))
		return VIRTIO_GPU_RESP_OK_NODATA;

	/* Linux normally flushes the same damage rectangle it just transferred.
	 * Preserve the transfer command's explicit byte offset when it covers the
	 * visible region; otherwise use the defined tightly packed 2D layout. */
	if (resource->transfer_valid &&
	    visible.x >= resource->transfer_rect.x &&
	    visible.y >= resource->transfer_rect.y &&
	    visible.x + visible.width <= resource->transfer_rect.x +
					 resource->transfer_rect.width &&
	    visible.y + visible.height <= resource->transfer_rect.y +
					  resource->transfer_rect.height) {
		offset = resource->transfer_offset +
			 (uint64_t)(visible.y - resource->transfer_rect.y) *
				 resource->width * sizeof(uint32_t) +
			 (uint64_t)(visible.x - resource->transfer_rect.x) *
				 sizeof(uint32_t);
	} else {
		offset = ((uint64_t)visible.y * resource->width + visible.x) *
			 sizeof(uint32_t);
	}
	reader = (struct backing_reader){ .resource = resource };
	if (!display_bridge_xrgb8888_update(
			backing_read, &reader, offset,
			resource->width * sizeof(uint32_t),
			visible.x - scanout.rect.x,
			visible.y - scanout.rect.y,
			visible.width, visible.height))
		return VIRTIO_GPU_RESP_ERR_UNSPEC;
	display_bridge_xrgb8888_flush();
	return VIRTIO_GPU_RESP_OK_NODATA;
}

static void process_control_command(const struct virtio_queue_state *queue,
				    uint16_t head)
{
	union {
		struct virtio_gpu_ctrl_hdr header;
		struct virtio_gpu_resource_create_2d create_2d;
		struct virtio_gpu_resource_unref unref;
		struct virtio_gpu_set_scanout set_scanout;
		struct virtio_gpu_resource_flush flush;
		struct virtio_gpu_transfer_to_host_2d transfer;
		struct virtio_gpu_resource_attach_backing attach;
		struct virtio_gpu_resource_detach_backing detach;
	} command;
	struct virtio_gpu_resp_display_info display_info;
	struct virtio_gpu_ctrl_hdr response;
	size_t readable;
	size_t writable;
	size_t command_size = sizeof(command.header);
	size_t response_size = sizeof(response);
	uint32_t response_type = VIRTIO_GPU_RESP_ERR_UNSPEC;

	memset(&command, 0, sizeof(command));
	if (!chain_measure(queue, head, &readable, &writable) ||
	    readable < sizeof(command.header) || writable < sizeof(response) ||
	    !chain_read(queue, head, 0, &command.header,
			sizeof(command.header))) {
		(void)queue_complete(VIRTIO_GPU_CONTROL_QUEUE, queue, head, 0);
		return;
	}

	switch (command.header.type) {
	case VIRTIO_GPU_CMD_GET_DISPLAY_INFO:
		command_size = sizeof(command.header);
		break;
	case VIRTIO_GPU_CMD_RESOURCE_CREATE_2D:
		command_size = sizeof(command.create_2d);
		break;
	case VIRTIO_GPU_CMD_RESOURCE_UNREF:
		command_size = sizeof(command.unref);
		break;
	case VIRTIO_GPU_CMD_SET_SCANOUT:
		command_size = sizeof(command.set_scanout);
		break;
	case VIRTIO_GPU_CMD_RESOURCE_FLUSH:
		command_size = sizeof(command.flush);
		break;
	case VIRTIO_GPU_CMD_TRANSFER_TO_HOST_2D:
		command_size = sizeof(command.transfer);
		break;
	case VIRTIO_GPU_CMD_RESOURCE_ATTACH_BACKING:
		command_size = sizeof(command.attach);
		break;
	case VIRTIO_GPU_CMD_RESOURCE_DETACH_BACKING:
		command_size = sizeof(command.detach);
		break;
	default:
		break;
	}
	if (readable < command_size ||
	    !chain_read(queue, head, 0, &command, command_size))
		response_type = VIRTIO_GPU_RESP_ERR_INVALID_PARAMETER;
	else switch (command.header.type) {
	case VIRTIO_GPU_CMD_GET_DISPLAY_INFO:
		memset(&display_info, 0, sizeof(display_info));
		response_header(&display_info.hdr, &command.header,
				VIRTIO_GPU_RESP_OK_DISPLAY_INFO);
		display_info.modes[0].rect.width = DISPLAY_FB_WIDTH;
		display_info.modes[0].rect.height = DISPLAY_FB_HEIGHT;
		display_info.modes[0].enabled = 1u;
		response_size = sizeof(display_info);
		if (writable < response_size ||
		    !chain_write_response(queue, head, &display_info,
					  response_size))
			response_size = 0;
		(void)queue_complete(VIRTIO_GPU_CONTROL_QUEUE, queue, head,
				     (uint32_t)response_size);
		return;
	case VIRTIO_GPU_CMD_RESOURCE_CREATE_2D:
		response_type = command_create_2d(&command.create_2d);
		break;
	case VIRTIO_GPU_CMD_RESOURCE_UNREF: {
		struct gpu_resource *resource =
			resource_find(command.unref.resource_id);
		if (resource == NULL)
			response_type = VIRTIO_GPU_RESP_ERR_INVALID_RESOURCE_ID;
		else {
			resource_release(resource);
			response_type = VIRTIO_GPU_RESP_OK_NODATA;
		}
		break;
	}
	case VIRTIO_GPU_CMD_SET_SCANOUT:
		response_type = command_set_scanout(&command.set_scanout);
		break;
	case VIRTIO_GPU_CMD_RESOURCE_FLUSH:
		response_type = command_flush(&command.flush);
		break;
	case VIRTIO_GPU_CMD_TRANSFER_TO_HOST_2D:
		response_type = command_transfer(&command.transfer);
		break;
	case VIRTIO_GPU_CMD_RESOURCE_ATTACH_BACKING:
		response_type = command_attach_backing(
			queue, head, &command.attach, readable);
		break;
	case VIRTIO_GPU_CMD_RESOURCE_DETACH_BACKING: {
		struct gpu_resource *resource =
			resource_find(command.detach.resource_id);
		if (resource == NULL)
			response_type = VIRTIO_GPU_RESP_ERR_INVALID_RESOURCE_ID;
		else {
			heap_caps_free(resource->backing);
			resource->backing = NULL;
			resource->backing_count = 0;
			resource->backing_size = 0;
			resource->transfer_valid = false;
			response_type = VIRTIO_GPU_RESP_OK_NODATA;
		}
		break;
	}
	default:
		response_type = VIRTIO_GPU_RESP_ERR_UNSPEC;
		break;
	}

	response_header(&response, &command.header, response_type);
	if (!chain_write_response(queue, head, &response, sizeof(response)))
		response_size = 0;
	(void)queue_complete(VIRTIO_GPU_CONTROL_QUEUE, queue, head,
			     (uint32_t)response_size);
}

static void process_cursor_command(const struct virtio_queue_state *queue,
				   uint16_t head)
{
	struct virtio_gpu_update_cursor command;
	size_t readable;
	size_t writable;

	/* The upstream driver expects a dedicated cursor queue without response
	 * buffers. Consume standard cursor messages so they never backpressure the
	 * control path. A software overlay can be added without changing the ABI. */
	if (chain_measure(queue, head, &readable, &writable) &&
	    writable == 0 && readable >= sizeof(command) &&
	    chain_read(queue, head, 0, &command, sizeof(command)) &&
	    (command.hdr.type == VIRTIO_GPU_CMD_UPDATE_CURSOR ||
	     command.hdr.type == VIRTIO_GPU_CMD_MOVE_CURSOR)) {
		/* Position is deliberately ignored until alpha-overlay support is on. */
	}
	(void)queue_complete(VIRTIO_GPU_CURSOR_QUEUE, queue, head, 0);
}

static bool process_queue(uint32_t index)
{
	struct virtio_queue_state queue;
	uint16_t head;

	if (!queue_pop_available(index, &queue, &head))
		return false;
	if (index == VIRTIO_GPU_CONTROL_QUEUE)
		process_control_command(&queue, head);
	else
		process_cursor_command(&queue, head);
	return true;
}

static void worker_task(void *argument)
{
	(void)argument;
	for (;;) {
		ulTaskNotifyTake(pdTRUE, portMAX_DELAY);
		if (__atomic_exchange_n(&resources_reset_pending, false,
					__ATOMIC_ACQ_REL))
			resources_release_all();
		if (!(__atomic_load_n(&device_status, __ATOMIC_ACQUIRE) &
		      VIRTIO_STATUS_DRIVER_OK))
			continue;

		for (;;) {
			uint32_t processed = 0;

			while (processed < VIRTIO_GPU_SERVICE_BUDGET &&
			       process_queue(VIRTIO_GPU_CONTROL_QUEUE))
				processed++;
			while (processed < VIRTIO_GPU_SERVICE_BUDGET &&
			       process_queue(VIRTIO_GPU_CURSOR_QUEUE))
				processed++;
			if (processed < VIRTIO_GPU_SERVICE_BUDGET)
				break;
			/* Yield between bounded batches so scanout VSYNC and networking
			 * retain latency even under a permanently busy guest producer. */
			taskYIELD();
		}
	}
}

static void reset_device(void)
{
	for (uint32_t i = 0; i < VIRTIO_GPU_QUEUE_COUNT; i++)
		__atomic_store_n(&queues[i].ready, 0u, __ATOMIC_RELEASE);
	__atomic_thread_fence(__ATOMIC_SEQ_CST);
	memset(queues, 0, sizeof(queues));
	memset(driver_features, 0, sizeof(driver_features));
	device_features_sel = 0;
	driver_features_sel = 0;
	queue_sel = 0;
	__atomic_store_n(&device_status, 0u, __ATOMIC_RELEASE);
	__atomic_store_n(&interrupt_status, 0u, __ATOMIC_RELEASE);
	__atomic_store_n(&resources_reset_pending, true, __ATOMIC_RELEASE);
	if (worker_task_handle != NULL)
		xTaskNotifyGive(worker_task_handle);
}

int virtio_gpu_bridge_init(void)
{
	BaseType_t worker_core = CONFIG_FREERTOS_NUMBER_OF_CORES > 1 ? 1 : 0;

	device_ready = display_bridge_xrgb8888_available();
	reset_device();
	if (!device_ready) {
		printf("WARNING: virtio-gpu disabled: display import path unavailable\n");
		return -1;
	}
	if (xTaskCreatePinnedToCore(worker_task, "rv32-gpu",
				    VIRTIO_GPU_TASK_STACK_BYTES, NULL,
				    VIRTIO_GPU_TASK_PRIORITY,
				    &worker_task_handle, worker_core) != pdPASS) {
		device_ready = false;
		return -1;
	}
	printf("virtio-gpu: standard 2D/KMS transport at 0x%08x, %ux%u, "
	       "PLIC source %u; XRGB8888 import/PPA scanout on CPU%d\n",
	       VIRTIO_GPU_GUEST_BASE, DISPLAY_FB_WIDTH, DISPLAY_FB_HEIGHT,
	       VIRTIO_GPU_PLIC_SOURCE, (int)worker_core);
	return 0;
}

bool virtio_gpu_bridge_contains(uint32_t address, size_t width)
{
	return width != 0 && width <= sizeof(uint32_t) &&
	       address >= VIRTIO_GPU_GUEST_BASE &&
	       address - VIRTIO_GPU_GUEST_BASE <= VIRTIO_GPU_GUEST_SIZE - width;
}

static uint32_t config_load(uint32_t offset, size_t width)
{
	uint32_t config[5] = {
		__atomic_load_n(&config_events, __ATOMIC_ACQUIRE),
		0,
		device_ready ? 1u : 0u,
		0,
		0,
	};
	uint32_t value = 0;

	if (offset >= sizeof(config) || width > sizeof(config) - offset)
		return 0;
	memcpy(&value, (const uint8_t *)config + offset, width);
	return value;
}

uint32_t virtio_gpu_bridge_load(uint32_t address, size_t width)
{
	uint32_t offset = address - VIRTIO_GPU_GUEST_BASE;
	struct virtio_queue_state *queue =
		queue_sel < VIRTIO_GPU_QUEUE_COUNT ? &queues[queue_sel] : NULL;

	if (offset >= VIRTIO_MMIO_CONFIG)
		return config_load(offset - VIRTIO_MMIO_CONFIG, width);
	if (width != sizeof(uint32_t))
		return 0;
	switch (offset) {
	case VIRTIO_MMIO_MAGIC_VALUE: return 0x74726976u; /* "virt" */
	case VIRTIO_MMIO_VERSION: return 2u;
	case VIRTIO_MMIO_DEVICE_ID:
		return device_ready ? VIRTIO_GPU_DEVICE_ID_VALUE : 0u;
	case VIRTIO_MMIO_VENDOR_ID: return 0x505345u; /* "ESP" */
	case VIRTIO_MMIO_DEVICE_FEATURES:
		if (device_features_sel == 0u)
			return 1u << VIRTIO_RING_F_INDIRECT_DESC;
		if (device_features_sel == 1u)
			return 1u << (VIRTIO_F_VERSION_1 - 32u);
		return 0;
	case VIRTIO_MMIO_QUEUE_NUM_MAX:
		return queue != NULL ? VIRTIO_GPU_QUEUE_MAX : 0;
	case VIRTIO_MMIO_QUEUE_READY:
		return queue != NULL
			       ? __atomic_load_n(&queue->ready, __ATOMIC_ACQUIRE)
			       : 0;
	case VIRTIO_MMIO_INTERRUPT_STATUS:
		return __atomic_load_n(&interrupt_status, __ATOMIC_ACQUIRE);
	case VIRTIO_MMIO_STATUS:
		return __atomic_load_n(&device_status, __ATOMIC_ACQUIRE);
	case VIRTIO_MMIO_CONFIG_GENERATION: return 0;
	default: return 0;
	}
}

void virtio_gpu_bridge_store(uint32_t address, uint32_t value, size_t width)
{
	uint32_t offset = address - VIRTIO_GPU_GUEST_BASE;
	struct virtio_queue_state *queue =
		queue_sel < VIRTIO_GPU_QUEUE_COUNT ? &queues[queue_sel] : NULL;

	if (offset >= VIRTIO_MMIO_CONFIG) {
		if (width == sizeof(uint32_t) &&
		    offset == VIRTIO_MMIO_CONFIG + sizeof(uint32_t))
			__atomic_fetch_and(&config_events, ~value, __ATOMIC_RELEASE);
		return;
	}
	if (width != sizeof(uint32_t))
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
		if (queue != NULL && value != 0 && value <= VIRTIO_GPU_QUEUE_MAX)
			queue->size = value;
		break;
	case VIRTIO_MMIO_QUEUE_READY:
		if (queue != NULL)
			__atomic_store_n(&queue->ready, value & 1u,
					 __ATOMIC_RELEASE);
		break;
	case VIRTIO_MMIO_QUEUE_DESC_LOW:
		if (queue != NULL)
			queue->desc = (queue->desc & 0xffffffff00000000ULL) | value;
		break;
	case VIRTIO_MMIO_QUEUE_DESC_HIGH:
		if (queue != NULL)
			queue->desc = (queue->desc & 0xffffffffULL) |
				      ((uint64_t)value << 32);
		break;
	case VIRTIO_MMIO_QUEUE_AVAIL_LOW:
		if (queue != NULL)
			queue->avail = (queue->avail & 0xffffffff00000000ULL) | value;
		break;
	case VIRTIO_MMIO_QUEUE_AVAIL_HIGH:
		if (queue != NULL)
			queue->avail = (queue->avail & 0xffffffffULL) |
				       ((uint64_t)value << 32);
		break;
	case VIRTIO_MMIO_QUEUE_USED_LOW:
		if (queue != NULL)
			queue->used = (queue->used & 0xffffffff00000000ULL) | value;
		break;
	case VIRTIO_MMIO_QUEUE_USED_HIGH:
		if (queue != NULL)
			queue->used = (queue->used & 0xffffffffULL) |
				      ((uint64_t)value << 32);
		break;
	case VIRTIO_MMIO_QUEUE_NOTIFY:
		if (value < VIRTIO_GPU_QUEUE_COUNT && worker_task_handle != NULL)
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
			if ((value & VIRTIO_STATUS_DRIVER_OK) &&
			    worker_task_handle != NULL)
				xTaskNotifyGive(worker_task_handle);
		}
		break;
	default: break;
	}
}

bool virtio_gpu_bridge_irq_pending(void)
{
	return __atomic_load_n(&interrupt_status, __ATOMIC_ACQUIRE) != 0;
}
