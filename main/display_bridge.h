#ifndef DISPLAY_BRIDGE_H
#define DISPLAY_BRIDGE_H

#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>

#include "sdkconfig.h"

#define DISPLAY_FB_GUEST_BASE 0x11800000u
#define DISPLAY_FB_WIDTH      720u
#define DISPLAY_FB_HEIGHT     720u
#define DISPLAY_FB_STRIDE     (DISPLAY_FB_WIDTH * sizeof(uint16_t))
#define DISPLAY_FB_SIZE       (DISPLAY_FB_STRIDE * DISPLAY_FB_HEIGHT)

#if defined(CONFIG_RV32_ST7703_DISPLAY) && CONFIG_RV32_ST7703_DISPLAY
int display_bridge_init(void);
void display_bridge_commit(void);
bool display_bridge_contains(uint32_t address, size_t width);
uint32_t display_bridge_load(uint32_t address, size_t width);
void display_bridge_store(uint32_t address, uint32_t value, size_t width);
#else
static inline int display_bridge_init(void)
{
	return 0;
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
#endif

#endif /* DISPLAY_BRIDGE_H */
