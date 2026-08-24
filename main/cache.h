/*
 * Copyright (c) 2023, Jisheng Zhang <jszhang@kernel.org>. All rights reserved.
 *
 * SPDX-License-Identifier: BSD-3-Clause
 */

#ifndef CACHE_H
#define CACHE_H

#include <stdint.h>

void cache_write(uint32_t ofs, void *buf, uint32_t size);
void cache_read(uint32_t ofs, void *buf, uint32_t size);
void cache_write_u32(uint32_t ofs, uint32_t value);
void cache_write_u16(uint32_t ofs, uint16_t value);
void cache_write_u8(uint32_t ofs, uint8_t value);
uint32_t cache_read_u32(uint32_t ofs);
uint16_t cache_read_u16(uint32_t ofs);
uint8_t cache_read_u8(uint32_t ofs);
void cache_uncached_read(uint32_t ofs, void *buf, uint32_t size);
void cache_uncached_write(uint32_t ofs, const void *buf, uint32_t size);
void cache_clean_guest_line(uint32_t physical_address);
void cache_invalidate_guest_line(uint32_t physical_address);
void cache_flush_guest_line(uint32_t physical_address);
void cache_get_stat(uint64_t *phit, uint64_t *paccessed);

#endif /* CACHE_H */
