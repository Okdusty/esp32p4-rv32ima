/* SPDX-License-Identifier: BSD-3-Clause */
#ifndef VIRTIO_CONSOLE_BRIDGE_H
#define VIRTIO_CONSOLE_BRIDGE_H

#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>

#define VIRTIO_CONSOLE_GUEST_BASE 0x12100000u
#define VIRTIO_CONSOLE_GUEST_SIZE 0x00000200u
#define VIRTIO_CONSOLE_PLIC_SOURCE 4u

int virtio_console_bridge_init(void);
bool virtio_console_bridge_contains(uint32_t address, size_t width);
uint32_t virtio_console_bridge_load(uint32_t address, size_t width);
void virtio_console_bridge_store(uint32_t address, uint32_t value,
                                 size_t width);

/* Polling is intentionally cheap. It retries a guest TX descriptor only when
 * the CPU1 terminal stream had no room, and moves UART RX into an available
 * input descriptor without introducing a timer task. */
void virtio_console_bridge_poll(void);
bool virtio_console_bridge_irq_pending(void);
bool virtio_console_bridge_owns_uart_rx(void);

#endif /* VIRTIO_CONSOLE_BRIDGE_H */
