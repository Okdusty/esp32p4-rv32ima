// ============================================================================
// psram.h - Updated for internal PSRAM
// ============================================================================
/*
 * Copyright (c) 2023, Jisheng Zhang <jszhang@kernel.org>. All rights reserved.
 *
 * SPDX-License-Identifier: BSD-3-Clause
 */

#ifndef PSRAM_H
#define PSRAM_H

#include <stdint.h>
#include <stddef.h>

#define GUEST_PHYS_BASE    0x48000000u  //for DMA, spesific sequence 
#define GUEST_RAM_SIZE     0x01E00000u  //total parition = 30MB
#define KERNEL_LOAD_OFFSET 0x00400000u  //this is required for MMU, supervisor 0x0400000 
#define DTB_LOAD_OFFSET    0x01D00000u  //last sector we use last 1MB just for DTB

/*
 * The first 4 MiB contain ESP-IDF's PSRAM XIP mappings and allocator
 * bookkeeping.  Linux already leaves this Image-header-mandated prefix
 * unavailable, so only the identity-mapped tail is exposed to its allocator
 * and to USB DMA.
 */
#define GUEST_DMA_BASE     (GUEST_PHYS_BASE + KERNEL_LOAD_OFFSET)
#define GUEST_PHYS_END     (GUEST_PHYS_BASE + GUEST_RAM_SIZE)

int psram_init(void);
int psram_read(uint32_t addr, void *buf, int len);
int psram_write(uint32_t addr, void *buf, int len);

void *psram_get_base(void);
size_t psram_get_size(void);

#endif
