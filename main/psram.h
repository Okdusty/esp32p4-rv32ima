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

#include <stdbool.h>
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

/* The ST7703 live scanout needs a complete render surface so guest painting
 * never races the panel DMA.  Keep it in the final megabyte that is already
 * excluded from the kernel Image: the DTB occupies the first few KiB and the
 * matching reserved-memory node in uc.dts keeps Linux off the remainder. */
#define HOST_DISPLAY_DTB_GAP       0x00002000u
#define HOST_DISPLAY_RESERVED_BASE \
  (GUEST_PHYS_BASE + DTB_LOAD_OFFSET + HOST_DISPLAY_DTB_GAP)
#define HOST_DISPLAY_RESERVED_SIZE \
  (GUEST_PHYS_END - HOST_DISPLAY_RESERVED_BASE)

/* ESP32-P4 maps PSRAM on both its instruction and data buses.  Reserve the
 * top of Linux's Image-header prefix for generated host code; the remaining
 * prefix stays available to the virtio-block read cache. */
#define HOST_JIT_CODE_SIZE (2048u * 1024u)
/* Per-region memory descriptors consume 512 KiB. Keep the JIT's 24 KiB
 * probation table and 24 KiB cold successor-candidate state beside them. The
 * 16 KiB active successor-link directory stays internal because generated
 * code reads it at every native region transition. */
#define HOST_JIT_METADATA_SIZE (576u * 1024u)
#define HOST_AUXILIARY_SIZE (128u * 1024u)

int psram_init(void);
int psram_read(uint32_t addr, void *buf, int len);
int psram_write(uint32_t addr, void *buf, int len);

void *psram_get_base(void);
size_t psram_get_size(void);

/*
 * Return the portion of the reserved PSRAM allocation below Linux's first
 * usable physical address.  The Image header forces Linux to ignore this
 * prefix, but it is still valid host memory and is useful for read-through
 * caches owned by ESP-side devices.  Callers must not expose it to the guest.
 */
bool psram_get_host_cache_region(void **base, size_t *size);
bool psram_get_host_jit_region(void **base, size_t *size);
bool psram_get_host_jit_metadata_region(void **base, size_t *size);

/* Return PSRAM excluded from Linux by the ST7703 DTB for a host-owned render
 * surface. Other display configurations deliberately report no region. */
bool psram_get_host_display_region(void **base, size_t *size);

/* Reserve startup-lifetime storage from the high end of the host cache
 * region. Reservations must be made before psram_get_host_cache_region();
 * the remaining low portion is still exported to the virtio-block cache. */
void *psram_host_cache_alloc(size_t size, size_t alignment);

/* Allocate startup-lifetime bulk storage from a host-only slice of the
 * reserved Image-header prefix. This monotonic allocator is available only
 * when CONFIG_RV32_HOST_BULK_BUFFERS_PSRAM is enabled. */
void *psram_host_aux_alloc(size_t size, size_t alignment);

#endif
