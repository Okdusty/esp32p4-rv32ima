/*
 * Copyright (c) 2023, Jisheng Zhang <jszhang@kernel.org>. All rights reserved.
 *
 * SPDX-License-Identifier: BSD-3-Clause
 */

#ifndef PORT_H
#define PORT_H

#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>

enum host_dma_sync_op {
	HOST_DMA_SYNC_CLEAN,
	HOST_DMA_SYNC_INVALIDATE,
	HOST_DMA_SYNC_FLUSH,
};

struct host_cache_stats {
	uint32_t l1_hits;
	uint32_t l1_misses;
	uint32_t l1_conflicts;
	uint32_t l1_next_reads;
	uint32_t l1_next_writes;
	uint32_t l2_hits;
	uint32_t l2_misses;
	uint32_t l2_conflicts;
	uint32_t l2_next_reads;
	uint32_t l2_next_writes;
};

uint64_t GetTimeMicroseconds();
int HostInputInit(void);
int HostConsoleInit(void);
int HostConsoleWrite(const void *buffer, size_t length);
void HostEnableGuestRamPrefetch(const void *base, size_t size);
void HostCacheStatsReset(void);
void HostCacheStatsReadAndReset(struct host_cache_stats *stats);
int HostDmaCacheSync(uint32_t guest_physical_address, size_t length,
		     enum host_dma_sync_op operation);
int IsKBHit();
int ReadKBByte();
int load_images(int ram_size, int *kern_len);
#endif /* PORT_H */
