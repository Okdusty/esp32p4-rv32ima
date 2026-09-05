/* SPDX-License-Identifier: BSD-3-Clause */
#ifndef VIRTIO_NET_BRIDGE_H
#define VIRTIO_NET_BRIDGE_H

#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>

#include "sdkconfig.h"

#define VIRTIO_NET_GUEST_BASE 0x12000000u
#define VIRTIO_NET_GUEST_SIZE 0x00000200u

/* PLIC source reserved for the optional virtio-net device in uc.dts. */
#define VIRTIO_NET_PLIC_SOURCE 3u

struct virtio_net_bridge_perf_stats {
	uint32_t rx_frames;
	uint32_t rx_bytes;
	uint32_t rx_drops;
	uint32_t tx_frames;
	uint32_t tx_bytes;
	uint32_t tx_errors;
	uint32_t rx_completions;
	uint32_t tx_completions;
	uint32_t guest_interrupts;
	uint32_t service_wakes;
	uint32_t budget_reschedules;
};

#if defined(CONFIG_RV32_WIFI_BRIDGE) && CONFIG_RV32_WIFI_BRIDGE
int virtio_net_bridge_init(void);
bool virtio_net_bridge_contains(uint32_t address, size_t width);
uint32_t virtio_net_bridge_load(uint32_t address, size_t width);
void virtio_net_bridge_store(uint32_t address, uint32_t value,
			     size_t width);
bool virtio_net_bridge_irq_pending(void);
void virtio_net_bridge_perf_snapshot(
	struct virtio_net_bridge_perf_stats *stats);
#else
/*
 * Keep the emulator's MMIO and PLIC paths branch-compatible when networking
 * is disabled, without compiling or linking the bridge task itself.
 */
static inline int virtio_net_bridge_init(void)
{
	return 0;
}

static inline bool virtio_net_bridge_contains(uint32_t address, size_t width)
{
	(void)address;
	(void)width;
	return false;
}

static inline uint32_t virtio_net_bridge_load(uint32_t address, size_t width)
{
	(void)address;
	(void)width;
	return 0;
}

static inline void virtio_net_bridge_store(uint32_t address, uint32_t value,
					   size_t width)
{
	(void)address;
	(void)value;
	(void)width;
}

static inline bool virtio_net_bridge_irq_pending(void)
{
	return false;
}

static inline void virtio_net_bridge_perf_snapshot(
	struct virtio_net_bridge_perf_stats *stats)
{
	if (stats)
		*stats = (struct virtio_net_bridge_perf_stats){ 0 };
}
#endif

#endif /* VIRTIO_NET_BRIDGE_H */
