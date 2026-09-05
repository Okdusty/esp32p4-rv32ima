/* SPDX-License-Identifier: BSD-3-Clause */
#ifndef VIRTIO_BLK_BRIDGE_H
#define VIRTIO_BLK_BRIDGE_H

#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>

#define VIRTIO_BLK_GUEST_BASE 0x12200000u
#define VIRTIO_BLK_GUEST_SIZE 0x00000200u
#define VIRTIO_BLK_PLIC_SOURCE 5u

struct virtio_blk_bridge_perf_stats {
	uint32_t cache_hits;
	uint32_t cache_misses;
	uint32_t read_bytes;
	uint32_t write_bytes;
	uint32_t requests;
};

int virtio_blk_bridge_init(void);
bool virtio_blk_bridge_contains(uint32_t address, size_t width);
uint32_t virtio_blk_bridge_load(uint32_t address, size_t width);
void virtio_blk_bridge_store(uint32_t address, uint32_t value, size_t width);
bool virtio_blk_bridge_irq_pending(void);
void virtio_blk_bridge_perf_snapshot(
	struct virtio_blk_bridge_perf_stats *stats);

#endif /* VIRTIO_BLK_BRIDGE_H */
