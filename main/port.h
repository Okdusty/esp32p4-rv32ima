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

uint64_t GetTimeMicroseconds();
int HostInputInit(void);
int HostConsoleInit(void);
int HostConsoleWrite(const void *buffer, size_t length);
int HostDmaCacheSync(uint32_t guest_physical_address, size_t length,
		     enum host_dma_sync_op operation);
int IsKBHit();
int ReadKBByte();
int load_images(int ram_size, int *kern_len);
#endif /* PORT_H */
