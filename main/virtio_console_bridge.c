/*
 * Standard virtio-mmio console backed by the ESP32-P4 UART and display task.
 *
 * Linux uses its unmodified virtio_console/hvc driver. Guest output is copied
 * once from a split-ring descriptor into the CPU1 terminal stream and also
 * mirrored to UART0. UART0 input is placed directly in the guest's receive
 * virtqueue, so an interactive hvc terminal does not pass through the
 * emulated 16550 byte-by-byte.
 *
 * SPDX-License-Identifier: BSD-3-Clause
 */

#include "virtio_console_bridge.h"

#include <stdbool.h>
#include <stdint.h>
#include <stdio.h>
#include <string.h>

#include "display_bridge.h"
#include "port.h"
#include "psram.h"

#define VIRTIO_MMIO_MAGIC_VALUE         0x000u
#define VIRTIO_MMIO_VERSION             0x004u
#define VIRTIO_MMIO_DEVICE_ID           0x008u
#define VIRTIO_MMIO_VENDOR_ID           0x00cu
#define VIRTIO_MMIO_DEVICE_FEATURES     0x010u
#define VIRTIO_MMIO_DEVICE_FEATURES_SEL 0x014u
#define VIRTIO_MMIO_DRIVER_FEATURES     0x020u
#define VIRTIO_MMIO_DRIVER_FEATURES_SEL 0x024u
#define VIRTIO_MMIO_QUEUE_SEL           0x030u
#define VIRTIO_MMIO_QUEUE_NUM_MAX       0x034u
#define VIRTIO_MMIO_QUEUE_NUM           0x038u
#define VIRTIO_MMIO_QUEUE_READY         0x044u
#define VIRTIO_MMIO_QUEUE_NOTIFY        0x050u
#define VIRTIO_MMIO_INTERRUPT_STATUS    0x060u
#define VIRTIO_MMIO_INTERRUPT_ACK       0x064u
#define VIRTIO_MMIO_STATUS              0x070u
#define VIRTIO_MMIO_QUEUE_DESC_LOW      0x080u
#define VIRTIO_MMIO_QUEUE_DESC_HIGH     0x084u
#define VIRTIO_MMIO_QUEUE_AVAIL_LOW     0x090u
#define VIRTIO_MMIO_QUEUE_AVAIL_HIGH    0x094u
#define VIRTIO_MMIO_QUEUE_USED_LOW      0x0a0u
#define VIRTIO_MMIO_QUEUE_USED_HIGH     0x0a4u
#define VIRTIO_MMIO_CONFIG_GENERATION   0x0fcu
#define VIRTIO_MMIO_CONFIG              0x100u

#define VIRTIO_MMIO_INT_VRING           (1u << 0)
#define VIRTIO_STATUS_DRIVER_OK          (1u << 2)
#define VIRTIO_F_VERSION_1               32u
#define VIRTIO_CONSOLE_F_SIZE             0u

#define VIRTQ_DESC_F_NEXT                1u
#define VIRTQ_DESC_F_WRITE               2u
#define VIRTQ_DESC_F_INDIRECT            4u
#define VIRTQ_AVAIL_F_NO_INTERRUPT       1u

#define VIRTIO_CONSOLE_RX_QUEUE          0u
#define VIRTIO_CONSOLE_TX_QUEUE          1u
#define VIRTIO_CONSOLE_QUEUE_COUNT       2u
#define VIRTIO_CONSOLE_QUEUE_MAX        64u
#define VIRTIO_CONSOLE_TX_MAX         4096u
#define VIRTIO_CONSOLE_RX_BURST         256u

struct virtq_desc {
  uint64_t address;
  uint32_t length;
  uint16_t flags;
  uint16_t next;
} __attribute__((packed));

struct virtq_used_elem {
  uint32_t id;
  uint32_t length;
} __attribute__((packed));

struct virtio_queue_state {
  uint32_t size;
  uint32_t ready;
  uint64_t desc;
  uint64_t avail;
  uint64_t used;
  uint16_t last_avail;
};

static struct virtio_queue_state queues[VIRTIO_CONSOLE_QUEUE_COUNT];
static uint32_t device_features_sel;
static uint32_t driver_features_sel;
static uint32_t driver_features[2];
static uint32_t queue_sel;
static uint32_t device_status;
static uint32_t interrupt_status;
static uint8_t tx_staging[VIRTIO_CONSOLE_TX_MAX];
static uint8_t rx_staging[VIRTIO_CONSOLE_RX_BURST];

static bool guest_range_valid(uint64_t address, size_t length)
{
  if ((address >> 32) != 0 || length == 0 || address < GUEST_DMA_BASE ||
      address >= GUEST_PHYS_END)
    return false;
  return length <= (size_t)(GUEST_PHYS_END - (uint32_t)address);
}

static bool guest_read(uint64_t address, void *destination, size_t length)
{
  if (!guest_range_valid(address, length))
    return false;
  memcpy(destination, (const void *)(uintptr_t)(uint32_t)address, length);
  return true;
}

static bool guest_write(uint64_t address, const void *source, size_t length)
{
  if (!guest_range_valid(address, length))
    return false;
  memcpy((void *)(uintptr_t)(uint32_t)address, source, length);
  return true;
}

static bool queue_snapshot(uint32_t index, struct virtio_queue_state *queue)
{
  if (index >= VIRTIO_CONSOLE_QUEUE_COUNT ||
      !__atomic_load_n(&queues[index].ready, __ATOMIC_ACQUIRE))
    return false;

  *queue = queues[index];
  return queue->size != 0 && queue->size <= VIRTIO_CONSOLE_QUEUE_MAX &&
         guest_range_valid(queue->desc,
                           queue->size * sizeof(struct virtq_desc)) &&
         guest_range_valid(queue->avail,
                           6u + queue->size * sizeof(uint16_t)) &&
         guest_range_valid(queue->used,
                           6u + queue->size * sizeof(struct virtq_used_elem));
}

static bool queue_read_desc(const struct virtio_queue_state *queue,
                            uint16_t index, struct virtq_desc *descriptor)
{
  return index < queue->size &&
         guest_read(queue->desc + index * sizeof(*descriptor), descriptor,
                    sizeof(*descriptor));
}

/* Peek rather than pop: a TX descriptor remains guest-owned until the CPU1
 * terminal stream has accepted the complete record. */
static bool queue_peek_available(uint32_t index,
                                 struct virtio_queue_state *queue,
                                 uint16_t *head)
{
  uint16_t available_index;
  uint16_t ring_entry;

  if (!queue_snapshot(index, queue) ||
      !guest_read(queue->avail + 2u, &available_index,
                  sizeof(available_index)))
    return false;
  __atomic_thread_fence(__ATOMIC_ACQUIRE);
  if (queues[index].last_avail == available_index)
    return false;

  uint16_t slot = queues[index].last_avail % queue->size;
  if (!guest_read(queue->avail + 4u + slot * sizeof(uint16_t), &ring_entry,
                  sizeof(ring_entry)) ||
      ring_entry >= queue->size)
    return false;
  *head = ring_entry;
  return true;
}

static bool queue_complete(uint32_t index,
                           const struct virtio_queue_state *queue,
                           uint16_t head, uint32_t length)
{
  struct virtq_used_elem element = { .id = head, .length = length };
  uint16_t used_index;
  uint16_t available_flags = 0;

  if (!guest_read(queue->used + 2u, &used_index, sizeof(used_index)))
    return false;
  uint16_t slot = used_index % queue->size;
  if (!guest_write(queue->used + 4u + slot * sizeof(element), &element,
                   sizeof(element)))
    return false;

  used_index++;
  __atomic_thread_fence(__ATOMIC_RELEASE);
  if (!guest_write(queue->used + 2u, &used_index, sizeof(used_index)))
    return false;
  __atomic_thread_fence(__ATOMIC_RELEASE);
  queues[index].last_avail++;

  if (!guest_read(queue->avail, &available_flags, sizeof(available_flags)) ||
      !(available_flags & VIRTQ_AVAIL_F_NO_INTERRUPT))
    __atomic_fetch_or(&interrupt_status, VIRTIO_MMIO_INT_VRING,
                      __ATOMIC_RELEASE);
  return true;
}

static bool tx_copy_chain(const struct virtio_queue_state *queue,
                          uint16_t head, size_t *length)
{
  size_t total = 0;
  uint16_t index = head;

  for (uint32_t visited = 0; visited < queue->size; visited++) {
    struct virtq_desc descriptor;

    if (!queue_read_desc(queue, index, &descriptor) ||
        (descriptor.flags & (VIRTQ_DESC_F_WRITE | VIRTQ_DESC_F_INDIRECT)) ||
        descriptor.length > sizeof(tx_staging) - total ||
        !guest_read(descriptor.address, tx_staging + total,
                    descriptor.length))
      return false;
    total += descriptor.length;
    if (!(descriptor.flags & VIRTQ_DESC_F_NEXT)) {
      *length = total;
      return total != 0;
    }
    index = descriptor.next;
  }
  return false;
}

static bool rx_chain_capacity(const struct virtio_queue_state *queue,
                              uint16_t head, size_t *capacity)
{
  size_t total = 0;
  uint16_t index = head;

  for (uint32_t visited = 0; visited < queue->size; visited++) {
    struct virtq_desc descriptor;

    if (!queue_read_desc(queue, index, &descriptor) ||
        !(descriptor.flags & VIRTQ_DESC_F_WRITE) ||
        (descriptor.flags & VIRTQ_DESC_F_INDIRECT) ||
        !guest_range_valid(descriptor.address, descriptor.length) ||
        descriptor.length > SIZE_MAX - total)
      return false;
    total += descriptor.length;
    if (!(descriptor.flags & VIRTQ_DESC_F_NEXT)) {
      *capacity = total;
      return total != 0;
    }
    index = descriptor.next;
  }
  return false;
}

static bool rx_write_chain(const struct virtio_queue_state *queue,
                           uint16_t head, const uint8_t *source,
                           size_t length)
{
  uint16_t index = head;

  for (uint32_t visited = 0; visited < queue->size && length != 0;
       visited++) {
    struct virtq_desc descriptor;
    size_t chunk;

    if (!queue_read_desc(queue, index, &descriptor) ||
        !(descriptor.flags & VIRTQ_DESC_F_WRITE) ||
        (descriptor.flags & VIRTQ_DESC_F_INDIRECT))
      return false;
    chunk = descriptor.length < length ? descriptor.length : length;
    if (!guest_write(descriptor.address, source, chunk))
      return false;
    source += chunk;
    length -= chunk;
    if (length == 0)
      return true;
    if (!(descriptor.flags & VIRTQ_DESC_F_NEXT))
      return false;
    index = descriptor.next;
  }
  return length == 0;
}

static bool process_one_tx(void)
{
  struct virtio_queue_state queue;
  uint16_t head;
  size_t length;

  if (!queue_peek_available(VIRTIO_CONSOLE_TX_QUEUE, &queue, &head))
    return false;
  if (!tx_copy_chain(&queue, head, &length)) {
    (void)queue_complete(VIRTIO_CONSOLE_TX_QUEUE, &queue, head, 0);
    return true;
  }

  /* Enqueue the complete ANSI record atomically. Retrying a full stream must
   * never duplicate a prefix on either the display or UART mirror. */
  if (!display_bridge_terminal_write(tx_staging, length))
    return false;
  (void)HostConsoleWrite(tx_staging, length);
  (void)queue_complete(VIRTIO_CONSOLE_TX_QUEUE, &queue, head,
                       (uint32_t)length);
  return true;
}

static bool process_one_rx(void)
{
  struct virtio_queue_state queue;
  uint16_t head;
  size_t capacity;
  size_t length = 0;

  if (!IsKBHit() ||
      !queue_peek_available(VIRTIO_CONSOLE_RX_QUEUE, &queue, &head) ||
      !rx_chain_capacity(&queue, head, &capacity))
    return false;
  if (capacity > sizeof(rx_staging))
    capacity = sizeof(rx_staging);

  while (length < capacity && IsKBHit()) {
    int value = ReadKBByte();

    if (value < 0)
      break;
    rx_staging[length++] = (uint8_t)value;
  }
  if (length == 0)
    return false;
  if (!rx_write_chain(&queue, head, rx_staging, length))
    return false;
  (void)queue_complete(VIRTIO_CONSOLE_RX_QUEUE, &queue, head,
                       (uint32_t)length);
  return true;
}

static void reset_device(void)
{
  for (uint32_t i = 0; i < VIRTIO_CONSOLE_QUEUE_COUNT; i++)
    __atomic_store_n(&queues[i].ready, 0u, __ATOMIC_RELEASE);
  __atomic_thread_fence(__ATOMIC_SEQ_CST);
  memset(queues, 0, sizeof(queues));
  memset(driver_features, 0, sizeof(driver_features));
  device_features_sel = 0;
  driver_features_sel = 0;
  queue_sel = 0;
  __atomic_store_n(&device_status, 0u, __ATOMIC_RELEASE);
  __atomic_store_n(&interrupt_status, 0u, __ATOMIC_RELEASE);
}

int virtio_console_bridge_init(void)
{
  reset_device();
  printf("virtio-console: standard hvc transport at 0x%08x, PLIC source %u\n",
         VIRTIO_CONSOLE_GUEST_BASE, VIRTIO_CONSOLE_PLIC_SOURCE);
  return 0;
}

bool virtio_console_bridge_contains(uint32_t address, size_t width)
{
  return width != 0 && width <= sizeof(uint32_t) &&
         address >= VIRTIO_CONSOLE_GUEST_BASE &&
         address - VIRTIO_CONSOLE_GUEST_BASE <=
             VIRTIO_CONSOLE_GUEST_SIZE - width;
}

static uint32_t config_load(uint32_t offset, size_t width)
{
  uint8_t config[4];
  uint16_t columns = (uint16_t)display_bridge_terminal_columns();
  uint16_t rows = (uint16_t)display_bridge_terminal_rows();
  uint32_t value = 0;

  memcpy(config, &columns, sizeof(columns));
  memcpy(config + sizeof(columns), &rows, sizeof(rows));
  if (offset >= sizeof(config) || width > sizeof(config) - offset)
    return 0;
  memcpy(&value, config + offset, width);
  return value;
}

uint32_t virtio_console_bridge_load(uint32_t address, size_t width)
{
  uint32_t offset = address - VIRTIO_CONSOLE_GUEST_BASE;
  struct virtio_queue_state *queue =
      queue_sel < VIRTIO_CONSOLE_QUEUE_COUNT ? &queues[queue_sel] : NULL;

  if (offset >= VIRTIO_MMIO_CONFIG)
    return config_load(offset - VIRTIO_MMIO_CONFIG, width);
  if (width != sizeof(uint32_t))
    return 0;

  switch (offset) {
  case VIRTIO_MMIO_MAGIC_VALUE: return 0x74726976u; /* "virt" */
  case VIRTIO_MMIO_VERSION: return 2u;
  case VIRTIO_MMIO_DEVICE_ID: return 3u; /* console */
  case VIRTIO_MMIO_VENDOR_ID: return 0x505345u; /* "ESP" */
  case VIRTIO_MMIO_DEVICE_FEATURES:
    if (device_features_sel == 0)
      return 1u << VIRTIO_CONSOLE_F_SIZE;
    if (device_features_sel == 1)
      return 1u << (VIRTIO_F_VERSION_1 - 32u);
    return 0;
  case VIRTIO_MMIO_QUEUE_NUM_MAX:
    return queue ? VIRTIO_CONSOLE_QUEUE_MAX : 0;
  case VIRTIO_MMIO_QUEUE_READY:
    return queue ? __atomic_load_n(&queue->ready, __ATOMIC_ACQUIRE) : 0;
  case VIRTIO_MMIO_INTERRUPT_STATUS:
    return __atomic_load_n(&interrupt_status, __ATOMIC_ACQUIRE);
  case VIRTIO_MMIO_STATUS:
    return __atomic_load_n(&device_status, __ATOMIC_ACQUIRE);
  case VIRTIO_MMIO_CONFIG_GENERATION: return 0;
  default: return 0;
  }
}

void virtio_console_bridge_store(uint32_t address, uint32_t value,
                                 size_t width)
{
  uint32_t offset = address - VIRTIO_CONSOLE_GUEST_BASE;
  struct virtio_queue_state *queue =
      queue_sel < VIRTIO_CONSOLE_QUEUE_COUNT ? &queues[queue_sel] : NULL;

  if (width != sizeof(uint32_t) || offset >= VIRTIO_MMIO_CONFIG)
    return;

  switch (offset) {
  case VIRTIO_MMIO_DEVICE_FEATURES_SEL: device_features_sel = value; break;
  case VIRTIO_MMIO_DRIVER_FEATURES_SEL: driver_features_sel = value; break;
  case VIRTIO_MMIO_DRIVER_FEATURES:
    if (driver_features_sel < 2u)
      driver_features[driver_features_sel] = value;
    break;
  case VIRTIO_MMIO_QUEUE_SEL: queue_sel = value; break;
  case VIRTIO_MMIO_QUEUE_NUM:
    if (queue && value <= VIRTIO_CONSOLE_QUEUE_MAX)
      queue->size = value;
    break;
  case VIRTIO_MMIO_QUEUE_READY:
    if (queue)
      __atomic_store_n(&queue->ready, value & 1u, __ATOMIC_RELEASE);
    break;
  case VIRTIO_MMIO_QUEUE_DESC_LOW:
    if (queue)
      queue->desc = (queue->desc & 0xffffffff00000000ULL) | value;
    break;
  case VIRTIO_MMIO_QUEUE_DESC_HIGH:
    if (queue)
      queue->desc = (queue->desc & 0xffffffffULL) | ((uint64_t)value << 32);
    break;
  case VIRTIO_MMIO_QUEUE_AVAIL_LOW:
    if (queue)
      queue->avail = (queue->avail & 0xffffffff00000000ULL) | value;
    break;
  case VIRTIO_MMIO_QUEUE_AVAIL_HIGH:
    if (queue)
      queue->avail = (queue->avail & 0xffffffffULL) | ((uint64_t)value << 32);
    break;
  case VIRTIO_MMIO_QUEUE_USED_LOW:
    if (queue)
      queue->used = (queue->used & 0xffffffff00000000ULL) | value;
    break;
  case VIRTIO_MMIO_QUEUE_USED_HIGH:
    if (queue)
      queue->used = (queue->used & 0xffffffffULL) | ((uint64_t)value << 32);
    break;
  case VIRTIO_MMIO_QUEUE_NOTIFY:
    virtio_console_bridge_poll();
    break;
  case VIRTIO_MMIO_INTERRUPT_ACK:
    __atomic_fetch_and(&interrupt_status, ~value, __ATOMIC_RELEASE);
    break;
  case VIRTIO_MMIO_STATUS:
    if (value == 0)
      reset_device();
    else
      __atomic_store_n(&device_status, value, __ATOMIC_RELEASE);
    break;
  default: break;
  }
}

bool virtio_console_bridge_owns_uart_rx(void)
{
  return (__atomic_load_n(&device_status, __ATOMIC_ACQUIRE) &
          VIRTIO_STATUS_DRIVER_OK) &&
         __atomic_load_n(&queues[VIRTIO_CONSOLE_RX_QUEUE].ready,
                         __ATOMIC_ACQUIRE);
}

void virtio_console_bridge_poll(void)
{
  if (!(__atomic_load_n(&device_status, __ATOMIC_ACQUIRE) &
        VIRTIO_STATUS_DRIVER_OK))
    return;

  /* Bound one emulator-loop visit. Normal hvc writes are one PAGE_SIZE
   * descriptor, while adjacent descriptors are picked up on the next batch. */
  for (uint32_t i = 0; i < 4u && process_one_tx(); i++)
    ;
  (void)process_one_rx();
}

bool virtio_console_bridge_irq_pending(void)
{
  virtio_console_bridge_poll();
  return __atomic_load_n(&interrupt_status, __ATOMIC_ACQUIRE) != 0;
}
