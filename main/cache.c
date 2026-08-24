/*
 * Copyright (c) 2023, Jisheng Zhang <jszhang@kernel.org>. All rights reserved.
 *
 * SPDX-License-Identifier: BSD-3-Clause
 */

#include <stdbool.h>
#include <stdint.h>
#include <string.h>

#include "cache.h"
#include "port.h"
#include "psram.h"
#include "sdkconfig.h"

#define CACHE_LINE_SIZE CONFIG_CACHE_L2_CACHE_LINE_SIZE
#define LINE_ADDRESS_MSK (~(CACHE_LINE_SIZE - 1u))

void cache_write(uint32_t ofs, void *buf, uint32_t size)
{
	psram_write(ofs, buf, size);
}

void cache_read(uint32_t ofs, void *buf, uint32_t size)
{
	psram_read(ofs, buf, size);
}

uint32_t cache_read_u32(uint32_t ofs)
{
	uint32_t value;
	psram_read(ofs, &value, sizeof(value));
	return value;
}

uint16_t cache_read_u16(uint32_t ofs)
{
	uint16_t value;
	psram_read(ofs, &value, sizeof(value));
	return value;
}

uint8_t cache_read_u8(uint32_t ofs)
{
	uint8_t value;
	psram_read(ofs, &value, sizeof(value));
	return value;
}

void cache_write_u32(uint32_t ofs, uint32_t value)
{
	psram_write(ofs, &value, sizeof(value));
}

void cache_write_u16(uint32_t ofs, uint16_t value)
{
	psram_write(ofs, &value, sizeof(value));
}

void cache_write_u8(uint32_t ofs, uint8_t value)
{
	psram_write(ofs, &value, sizeof(value));
}

static bool cache_guest_line_address(uint32_t physical_address,
				     uint32_t *line_offset)
{
	if (physical_address < GUEST_DMA_BASE ||
	    physical_address >= GUEST_PHYS_END)
		return false;

	*line_offset = (physical_address - GUEST_PHYS_BASE) &
		LINE_ADDRESS_MSK;
	return true;
}

void cache_clean_guest_line(uint32_t physical_address)
{
	uint32_t line_offset;
	if (!cache_guest_line_address(physical_address, &line_offset))
		return;

	HostDmaCacheSync(GUEST_PHYS_BASE + line_offset, CACHE_LINE_SIZE,
			     HOST_DMA_SYNC_CLEAN);
}

void cache_invalidate_guest_line(uint32_t physical_address)
{
	uint32_t line_offset;
	if (!cache_guest_line_address(physical_address, &line_offset))
		return;

	/* Make the device's writes visible to the ESP32-P4 data cache. */
	HostDmaCacheSync(GUEST_PHYS_BASE + line_offset, CACHE_LINE_SIZE,
			     HOST_DMA_SYNC_INVALIDATE);
}

void cache_flush_guest_line(uint32_t physical_address)
{
	uint32_t line_offset;
	if (!cache_guest_line_address(physical_address, &line_offset))
		return;

	HostDmaCacheSync(GUEST_PHYS_BASE + line_offset, CACHE_LINE_SIZE,
			     HOST_DMA_SYNC_FLUSH);
}

void cache_uncached_read(uint32_t ofs, void *buf, uint32_t size)
{
	uint32_t first_line = ofs & LINE_ADDRESS_MSK;
	uint32_t last_line;

	if (!size)
		return;

	last_line = (ofs + size - 1u) & LINE_ADDRESS_MSK;
	for (uint32_t line = first_line; ; line += CACHE_LINE_SIZE) {
		/* Drop the ESP32-P4 data-cache copy before a DMA-owned read. */
		cache_invalidate_guest_line(GUEST_PHYS_BASE + line);
		if (line == last_line)
			break;
	}

	psram_read(ofs, buf, size);
}

void cache_uncached_write(uint32_t ofs, const void *buf, uint32_t size)
{
	uint32_t first_line = ofs & LINE_ADDRESS_MSK;
	uint32_t last_line;

	if (!size)
		return;

	last_line = (ofs + size - 1u) & LINE_ADDRESS_MSK;
	for (uint32_t line = first_line; ; line += CACHE_LINE_SIZE) {
		/* Preserve device-owned bytes around a partial CPU write. */
		cache_invalidate_guest_line(GUEST_PHYS_BASE + line);
		if (line == last_line)
			break;
	}

	psram_write(ofs, (void *)buf, size);
	for (uint32_t line = first_line; ; line += CACHE_LINE_SIZE) {
		HostDmaCacheSync(GUEST_PHYS_BASE + line, CACHE_LINE_SIZE,
				     HOST_DMA_SYNC_FLUSH);
		if (line == last_line)
			break;
	}
}

void cache_get_stat(uint64_t *phit, uint64_t *paccessed)
{
	*phit = 0;
	*paccessed = 0;
}
