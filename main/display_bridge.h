#ifndef DISPLAY_BRIDGE_H
#define DISPLAY_BRIDGE_H

#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>

#include "sdkconfig.h"
#include "display_accel_protocol.h"

#ifndef DISPLAY_FB_GUEST_BASE
#define DISPLAY_FB_GUEST_BASE 0x11800000u
#endif
#if defined(CONFIG_RV32_ST7789_DISPLAY) && CONFIG_RV32_ST7789_DISPLAY
#define DISPLAY_FB_WIDTH 240u
#define DISPLAY_FB_HEIGHT 240u
#elif defined(CONFIG_RV32_SSD1306_DISPLAY) && CONFIG_RV32_SSD1306_DISPLAY
#define DISPLAY_FB_WIDTH 128u
#if defined(CONFIG_RV32_SSD1306_128X32) && CONFIG_RV32_SSD1306_128X32
#define DISPLAY_FB_HEIGHT 32u
#else
#define DISPLAY_FB_HEIGHT 64u
#endif
#else
#ifndef DISPLAY_FB_WIDTH
#define DISPLAY_FB_WIDTH 720u
#endif
#ifndef DISPLAY_FB_HEIGHT
#define DISPLAY_FB_HEIGHT 720u
#endif
#endif
#ifndef DISPLAY_FB_STRIDE
#define DISPLAY_FB_STRIDE (DISPLAY_FB_WIDTH * sizeof(uint16_t))
#endif
#define DISPLAY_FB_SIZE       (DISPLAY_FB_STRIDE * DISPLAY_FB_HEIGHT)
#define DISPLAY_ACCEL_STAGE_GUEST_BASE \
	(DISPLAY_FB_GUEST_BASE + DISPLAY_FB_SIZE)
#define DISPLAY_ACCEL_STATUS_GUEST_BASE \
	(DISPLAY_ACCEL_STAGE_GUEST_BASE + DISPLAY_ACCEL_STAGE_SIZE)
#define DISPLAY_ACCEL_SIZE_GUEST_BASE \
	(DISPLAY_ACCEL_STATUS_GUEST_BASE + sizeof(uint32_t))
#define DISPLAY_ACCEL_COMMAND_GUEST_BASE \
	(DISPLAY_ACCEL_SIZE_GUEST_BASE + sizeof(uint32_t))
#define DISPLAY_FB_COMMIT_GUEST_BASE \
	(DISPLAY_ACCEL_COMMAND_GUEST_BASE + sizeof(uint32_t))
#define DISPLAY_PV_REG_GUEST_BASE \
	(DISPLAY_FB_COMMIT_GUEST_BASE + sizeof(uint32_t))
/* Include page-aligned padding after the generic accelerator registers. */
#define DISPLAY_FB_APERTURE_USED \
	(DISPLAY_PV_REG_GUEST_BASE + DISPLAY_PV_REG_SIZE - DISPLAY_FB_GUEST_BASE)
#define DISPLAY_FB_APERTURE_SIZE \
	((DISPLAY_FB_APERTURE_USED + 0xfffu) & ~0xfffu)

struct display_bridge_perf_stats {
	uint32_t service_wakes;
	uint32_t service_us;
	uint32_t vsyncs;
	uint32_t frame_samples;
	uint32_t frame_total_us;
	uint32_t frame_max_us;
	uint32_t commands;
	uint32_t command_cycles;
	uint32_t command_max_cycles;
	uint32_t fill_commands;
	uint32_t fill_cycles;
	uint32_t copy_commands;
	uint32_t copy_cycles;
	uint32_t image1_commands;
	uint32_t image1_cycles;
	uint32_t bulk_write_commands;
	uint32_t bulk_write_cycles;
	uint32_t bulk_write_bytes;
	uint32_t tile_commands;
	uint32_t tile_set_commands;
	uint32_t tile_set_cycles;
	uint32_t tile_fill_commands;
	uint32_t tile_fill_cycles;
	uint32_t tile_blit_commands;
	uint32_t tile_blit_cycles;
	uint32_t tile_cursor_commands;
	uint32_t tile_cursor_cycles;
	uint32_t tile_batch_commands;
	uint32_t tile_batch_cycles;
	uint32_t tile_batch_records;
	uint32_t tile_batch_fallbacks;
	uint32_t cursor_toggles;
	uint32_t cursor_toggle_cycles;
	uint32_t cursor_toggle_max_cycles;
	uint32_t ppa_fills;
	uint32_t ppa_blits;
	uint32_t ppa_us;
	uint32_t ppa_fill_pixels;
	uint32_t cpu_fill_pixels;
	uint32_t copy_pixels;
	uint32_t tile_pixels;
	uint32_t cache_syncs;
	uint32_t cache_bytes;
	uint32_t cache_us;
	/* Raw framebuffer flow is counted independently from terminal/PPA cache
	 * traffic.  These counters are observational only: they never gate either
	 * the producer or the scanout worker. */
	uint32_t framebuffer_store_calls;
	uint32_t framebuffer_store_bytes;
	uint32_t framebuffer_commit_events;
	uint32_t framebuffer_commit_bytes;
	uint32_t framebuffer_publish_events;
	uint32_t framebuffer_publish_bytes;
	uint32_t framebuffer_present_events;
	uint32_t framebuffer_present_bytes;
	uint32_t framebuffer_present_us;
	uint32_t framebuffer_present_cpu_copies;
	uint32_t framebuffer_present_deferred;
	uint32_t framebuffer_guest_slices;
	uint32_t framebuffer_guest_wait_us;
	uint32_t framebuffer_guest_wait_max_us;
	uint32_t framebuffer_snapshot_wait_us;
	uint32_t framebuffer_snapshot_wait_max_us;
	uint32_t fifo_busy;
	uint32_t fifo_high_water;
	uint32_t fifo_slices;
	uint32_t fifo_deferred;
	uint32_t producer_submissions;
	uint32_t producer_us;
	uint32_t producer_payload_bytes;
	uint32_t producer_inline_payloads;
	uint32_t producer_external_payloads;
	uint32_t producer_shared_results;
	uint32_t producer_wakes;
	uint32_t producer_wake_us;
};

/* Read a contiguous byte range from a host-owned representation of a guest
 * graphics resource. The callback keeps the display layer independent from
 * virtio, DMA mappings, and a particular guest kernel. */
typedef bool (*display_bridge_resource_reader_t)(void *context,
						 uint64_t offset,
						 void *destination,
						 size_t length);

#if defined(CONFIG_RV32_DISPLAY_ACCEL) && CONFIG_RV32_DISPLAY_ACCEL
int display_bridge_init(void);
void display_bridge_set_guest_memory(void *host_base, uint32_t guest_base,
				     size_t size);
void *display_bridge_framebuffer_host(void);
void display_bridge_mark_framebuffer_dirty(size_t start, size_t end,
					   uint32_t store_calls,
					   uint32_t store_bytes);
void display_bridge_commit(void);
/* Keep one emulator execution slice from changing the logical framebuffer
 * while CPU1 snapshots it into a continuously scanned backend. Slice end
 * also recognizes a sustained quiet boundary for mmap writers which do not
 * issue the explicit framebuffer fence. */
void display_bridge_guest_slice_begin(void);
void display_bridge_guest_slice_end(void);
bool display_bridge_contains(uint32_t address, size_t width);
uint32_t display_bridge_load(uint32_t address, size_t width);
void display_bridge_store(uint32_t address, uint32_t value, size_t width);
/* Native clients fill the returned packed RGB565 staging buffer, then queue
 * an aspect-preserving hardware blit with display_bridge_accel_blit(). */
void *display_bridge_accel_buffer(size_t *capacity);
bool display_bridge_accel_blit(uint32_t width, uint32_t height);
void display_bridge_accel_stop(void);
/* Import a dirty rectangle from a tightly pixel-addressed XRGB8888 resource.
 * Conversion and scanout writes run on the caller's CPU (the virtio-gpu
 * worker is pinned to CPU1) and are serialized with the normal display task.
 * source_offset addresses the rectangle's first pixel; source_stride is the
 * complete resource pitch. */
bool display_bridge_xrgb8888_update(display_bridge_resource_reader_t reader,
					    void *context,
					    uint64_t source_offset,
					    uint32_t source_stride,
					    uint32_t destination_x,
					    uint32_t destination_y,
					    uint32_t width,
					    uint32_t height);
bool display_bridge_xrgb8888_available(void);
void display_bridge_xrgb8888_flush(void);
/* Standard virtio-console output enters this bounded SPSC stream. ANSI/VT
 * parsing and glyph painting then run only on the display service core. */
bool display_bridge_terminal_write(const void *buffer, size_t length);
uint32_t display_bridge_terminal_columns(void);
uint32_t display_bridge_terminal_rows(void);
void display_bridge_perf_read_and_reset(
	struct display_bridge_perf_stats *stats);
#else
static inline int display_bridge_init(void)
{
	return 0;
}

static inline void display_bridge_set_guest_memory(void *host_base,
					    uint32_t guest_base,
					    size_t size)
{
	(void)host_base;
	(void)guest_base;
	(void)size;
}

static inline void *display_bridge_framebuffer_host(void)
{
	return NULL;
}

static inline void display_bridge_mark_framebuffer_dirty(
	size_t start, size_t end, uint32_t store_calls, uint32_t store_bytes)
{
	(void)start;
	(void)end;
	(void)store_calls;
	(void)store_bytes;
}

static inline void display_bridge_commit(void)
{
}

static inline void display_bridge_guest_slice_begin(void)
{
}

static inline void display_bridge_guest_slice_end(void)
{
}

static inline bool display_bridge_contains(uint32_t address, size_t width)
{
	(void)address;
	(void)width;
	return false;
}

static inline uint32_t display_bridge_load(uint32_t address, size_t width)
{
	(void)address;
	(void)width;
	return 0;
}

static inline void display_bridge_store(uint32_t address, uint32_t value,
					 size_t width)
{
	(void)address;
	(void)value;
	(void)width;
}

static inline void *display_bridge_accel_buffer(size_t *capacity)
{
	if (capacity != NULL)
		*capacity = 0;
	return NULL;
}

static inline bool display_bridge_accel_blit(uint32_t width, uint32_t height)
{
	(void)width;
	(void)height;
	return false;
}

static inline void display_bridge_accel_stop(void)
{
}

static inline bool display_bridge_xrgb8888_update(
	display_bridge_resource_reader_t reader, void *context,
	uint64_t source_offset, uint32_t source_stride, uint32_t destination_x,
	uint32_t destination_y, uint32_t width, uint32_t height)
{
	(void)reader;
	(void)context;
	(void)source_offset;
	(void)source_stride;
	(void)destination_x;
	(void)destination_y;
	(void)width;
	(void)height;
	return false;
}

static inline void display_bridge_xrgb8888_flush(void)
{
}

static inline bool display_bridge_xrgb8888_available(void)
{
	return false;
}

static inline bool display_bridge_terminal_write(const void *buffer,
						   size_t length)
{
	(void)buffer;
	(void)length;
	return true;
}

static inline uint32_t display_bridge_terminal_columns(void)
{
	return 80u;
}

static inline uint32_t display_bridge_terminal_rows(void)
{
	return 25u;
}

static inline void display_bridge_perf_read_and_reset(
	struct display_bridge_perf_stats *stats)
{
	if (stats != NULL)
		*stats = (struct display_bridge_perf_stats){ 0 };
}
#endif

#endif /* DISPLAY_BRIDGE_H */
