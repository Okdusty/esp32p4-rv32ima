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
#ifndef DISPLAY_FB_WIDTH
#define DISPLAY_FB_WIDTH 720u
#endif
#ifndef DISPLAY_FB_HEIGHT
#define DISPLAY_FB_HEIGHT 720u
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
	uint32_t commands;
	uint32_t fill_commands;
	uint32_t copy_commands;
	uint32_t tile_commands;
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
	uint32_t fifo_busy;
	uint32_t fifo_high_water;
};

#if defined(CONFIG_RV32_DISPLAY_ACCEL) && CONFIG_RV32_DISPLAY_ACCEL
int display_bridge_init(void);
void display_bridge_set_guest_memory(void *host_base, uint32_t guest_base,
				     size_t size);
void display_bridge_commit(void);
bool display_bridge_contains(uint32_t address, size_t width);
uint32_t display_bridge_load(uint32_t address, size_t width);
void display_bridge_store(uint32_t address, uint32_t value, size_t width);
/* Native clients fill the returned packed RGB565 staging buffer, then queue
 * an aspect-preserving hardware blit with display_bridge_accel_blit(). */
void *display_bridge_accel_buffer(size_t *capacity);
bool display_bridge_accel_blit(uint32_t width, uint32_t height);
void display_bridge_accel_stop(void);
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

static inline void display_bridge_commit(void)
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

static inline void display_bridge_perf_read_and_reset(
	struct display_bridge_perf_stats *stats)
{
	if (stats != NULL)
		*stats = (struct display_bridge_perf_stats){ 0 };
}
#endif

#endif /* DISPLAY_BRIDGE_H */
