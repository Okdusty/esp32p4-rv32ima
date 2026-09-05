/* SPDX-License-Identifier: BSD-3-Clause */
#ifndef VIRTIO_GPU_BRIDGE_H
#define VIRTIO_GPU_BRIDGE_H

#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>

#include "sdkconfig.h"

#define VIRTIO_GPU_GUEST_BASE 0x12300000u
#define VIRTIO_GPU_GUEST_SIZE 0x00000200u
#define VIRTIO_GPU_PLIC_SOURCE 6u

#if defined(CONFIG_RV32_VIRTIO_GPU) && CONFIG_RV32_VIRTIO_GPU
int virtio_gpu_bridge_init(void);
bool virtio_gpu_bridge_contains(uint32_t address, size_t width);
uint32_t virtio_gpu_bridge_load(uint32_t address, size_t width);
void virtio_gpu_bridge_store(uint32_t address, uint32_t value, size_t width);
bool virtio_gpu_bridge_irq_pending(void);
#else
static inline int virtio_gpu_bridge_init(void)
{
	return 0;
}

static inline bool virtio_gpu_bridge_contains(uint32_t address, size_t width)
{
	return width != 0 && width <= sizeof(uint32_t) &&
	       address >= VIRTIO_GPU_GUEST_BASE &&
	       address - VIRTIO_GPU_GUEST_BASE <= VIRTIO_GPU_GUEST_SIZE - width;
}

static inline uint32_t virtio_gpu_bridge_load(uint32_t address, size_t width)
{
	uint32_t offset = address - VIRTIO_GPU_GUEST_BASE;

	if (width != sizeof(uint32_t))
		return 0;
	switch (offset) {
	case 0x000u: return 0x74726976u; /* "virt" */
	case 0x004u: return 2u;
	case 0x008u: return 0u;          /* Reserved: no device. */
	case 0x00cu: return 0x505345u;   /* "ESP" */
	default: return 0;
	}
}

static inline void virtio_gpu_bridge_store(uint32_t address, uint32_t value,
					    size_t width)
{
	(void)address;
	(void)value;
	(void)width;
}

static inline bool virtio_gpu_bridge_irq_pending(void)
{
	return false;
}
#endif

#endif /* VIRTIO_GPU_BRIDGE_H */
