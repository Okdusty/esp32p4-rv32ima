/*
 * Give the framebuffer VT and emulated 16550 UART one interactive shell.
 *
 * The shell owns a pseudo-terminal. A router copies its output into independent
 * tty0 and ttyS0 queues; dedicated workers are the only processes that write
 * those devices. The display path is lossless and runs at normal priority. The
 * serial mirror runs at background priority and drops complete queued bursts if
 * it falls behind, so 8250 transmit work can never hold up framebuffer output.
 * While applying display backpressure the router continues forwarding UART RX
 * through the PTY line discipline, keeping editing, echo and signals live.
 *
 * SPDX-License-Identifier: BSD-3-Clause
 */

#define _GNU_SOURCE

#include <errno.h>
#include <fcntl.h>
#include <limits.h>
#include <poll.h>
#include <sched.h>
#include <signal.h>
#include <stdbool.h>
#include <stdint.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <sys/ioctl.h>
#include <sys/mman.h>
#include <sys/resource.h>
#include <sys/types.h>
#include <sys/uio.h>
#include <sys/wait.h>
#include <termios.h>
#include <unistd.h>

#include "pv_console_protocol.h"

struct output_frame_header {
  uint32_t length;
  uint32_t sequence;
};

#define OUTPUT_BATCH_CAPACITY (PIPE_BUF - sizeof(struct output_frame_header))

struct mux_state {
  uint32_t display_submitted;
  uint32_t display_completed;
  uint32_t serial_mailbox_lock;
  uint32_t serial_mailbox_active;
  uint32_t serial_mailbox_generation;
  uint32_t serial_mailbox_sequence;
  uint32_t serial_mailbox_length;
  unsigned char serial_mailbox[OUTPUT_BATCH_CAPACITY];
};

#define DISPLAY_PIPE_CAPACITY PIPE_BUF
#define SERIAL_PIPE_CAPACITY PIPE_BUF
#define SERIAL_WORKER_NICE 5
#define SERIAL_WRITE_QUANTUM 64u

static volatile uint32_t *pv_console_registers;

enum enqueue_result {
  ENQUEUE_OK,
  ENQUEUE_FULL,
  ENQUEUE_FAILED,
};

static int write_all(int fd, const void *buffer, size_t length) {
  const unsigned char *cursor = buffer;

  while (length) {
    ssize_t written = write(fd, cursor, length);

    if (written > 0) {
      cursor += written;
      length -= (size_t)written;
      continue;
    }
    if (written < 0 && errno == EINTR)
      continue;
    return -1;
  }
  return 0;
}

static int read_all(int fd, void *buffer, size_t length) {
  unsigned char *cursor = buffer;

  while (length) {
    ssize_t received = read(fd, cursor, length);

    if (received > 0) {
      cursor += received;
      length -= (size_t)received;
      continue;
    }
    if (received < 0 && errno == EINTR)
      continue;
    return -1;
  }
  return 0;
}

static int make_raw_serial(int fd) {
  struct termios settings;

  if (tcgetattr(fd, &settings) < 0)
    return -1;
  cfmakeraw(&settings);
  settings.c_cflag |= CLOCAL | CREAD;
  cfsetispeed(&settings, B921600);
  cfsetospeed(&settings, B921600);
  return tcsetattr(fd, TCSANOW, &settings);
}

static void init_paravirtual_console(void) {
  int memory = open("/dev/mem", O_RDWR | O_SYNC | O_CLOEXEC);
  void *mapping;

  if (memory < 0)
    return;
  mapping = mmap(NULL, RV32_PV_CONSOLE_APERTURE_SIZE,
                 PROT_READ | PROT_WRITE, MAP_SHARED, memory,
                 RV32_PV_CONSOLE_BASE);
  close(memory);
  if (mapping == MAP_FAILED)
    return;

  pv_console_registers = mapping;
  if (pv_console_registers[RV32_PV_CONSOLE_MAGIC_OFFSET /
                           sizeof(uint32_t)] != RV32_PV_CONSOLE_MAGIC ||
      pv_console_registers[RV32_PV_CONSOLE_MAX_LENGTH_OFFSET /
                           sizeof(uint32_t)] < OUTPUT_BATCH_CAPACITY) {
    munmap(mapping, RV32_PV_CONSOLE_APERTURE_SIZE);
    pv_console_registers = NULL;
  }
}

static int write_serial_output(int fallback, const void *buffer,
                               size_t length) {
  if (pv_console_registers == NULL)
    return write_all(fallback, buffer, length);
  if (length == 0 || length > RV32_PV_CONSOLE_MAX_LENGTH)
    return -1;

  pv_console_registers[RV32_PV_CONSOLE_POINTER_OFFSET /
                       sizeof(uint32_t)] = (uint32_t)(uintptr_t)buffer;
  __atomic_thread_fence(__ATOMIC_RELEASE);
  pv_console_registers[RV32_PV_CONSOLE_LENGTH_OFFSET /
                       sizeof(uint32_t)] = (uint32_t)length;
  return 0;
}

static int wait_for_display_idle(const struct mux_state *state,
                                 int wake_read) {
  for (;;) {
    uint32_t submitted =
        __atomic_load_n(&state->display_submitted, __ATOMIC_ACQUIRE);
    uint32_t completed =
        __atomic_load_n(&state->display_completed, __ATOMIC_ACQUIRE);

    if (submitted == completed)
      return 0;

    for (;;) {
      struct pollfd descriptor = {.fd = wake_read, .events = POLLIN};
      int ready = poll(&descriptor, 1, -1);

      if (ready < 0 && errno == EINTR)
        continue;
      if (ready < 0 ||
          (descriptor.revents & (POLLERR | POLLHUP | POLLNVAL)))
        return -1;
      if (descriptor.revents & POLLIN) {
        unsigned char notifications[32];

        while (read(wake_read, notifications, sizeof(notifications)) > 0)
          ;
        break;
      }
    }
  }
}

static void mark_display_complete(struct mux_state *state, int wake_write,
                                  uint32_t sequence) {
  static const unsigned char wake = 1;

  __atomic_store_n(&state->display_completed, sequence, __ATOMIC_RELEASE);
  /* This pipe is only an edge notification; the shared sequence is the state.
   * A full pipe already contains a wakeup, so EAGAIN is harmless. */
  (void)write(wake_write, &wake, sizeof(wake));
}

static bool sequence_after(uint32_t candidate, uint32_t reference) {
  return (int32_t)(candidate - reference) > 0;
}

static void lock_serial_mailbox(struct mux_state *state) {
  while (__atomic_exchange_n(&state->serial_mailbox_lock, 1u,
                             __ATOMIC_ACQUIRE) != 0u)
    (void)sched_yield();
}

static void serial_mailbox_store_locked(struct mux_state *state,
                                        const void *buffer, size_t length,
                                        uint32_t sequence) {
  memcpy(state->serial_mailbox, buffer, length);
  state->serial_mailbox_length = (uint32_t)length;
  __atomic_store_n(&state->serial_mailbox_active, 1u, __ATOMIC_RELAXED);
  __atomic_store_n(&state->serial_mailbox_sequence, sequence,
                   __ATOMIC_RELAXED);
  __atomic_add_fetch(&state->serial_mailbox_generation, 1u,
                     __ATOMIC_RELEASE);
}

static void serial_mailbox_store(struct mux_state *state, const void *buffer,
                                 size_t length, uint32_t sequence) {
  lock_serial_mailbox(state);
  serial_mailbox_store_locked(state, buffer, length, sequence);
  __atomic_store_n(&state->serial_mailbox_lock, 0u, __ATOMIC_RELEASE);
}

static bool serial_mailbox_replace_if_active(struct mux_state *state,
                                             const void *buffer,
                                             size_t length,
                                             uint32_t sequence) {
  bool active;

  lock_serial_mailbox(state);
  active = __atomic_load_n(&state->serial_mailbox_active,
                           __ATOMIC_RELAXED) != 0u;
  if (active)
    serial_mailbox_store_locked(state, buffer, length, sequence);
  __atomic_store_n(&state->serial_mailbox_lock, 0u, __ATOMIC_RELEASE);
  return active;
}

static bool serial_mailbox_newer(const struct mux_state *state,
                                 uint32_t sequence) {
  uint32_t generation = __atomic_load_n(&state->serial_mailbox_generation,
                                         __ATOMIC_ACQUIRE);
  uint32_t latest;

  if (generation == 0u)
    return false;
  latest = __atomic_load_n(&state->serial_mailbox_sequence,
                           __ATOMIC_RELAXED);
  return sequence_after(latest, sequence);
}

static bool serial_mailbox_load(struct mux_state *state,
                                uint32_t *seen_generation,
                                struct output_frame_header *header,
                                void *buffer) {
  uint32_t generation;

  lock_serial_mailbox(state);

  generation = __atomic_load_n(&state->serial_mailbox_generation,
                                __ATOMIC_RELAXED);
  if (generation == *seen_generation ||
      state->serial_mailbox_length == 0u ||
      state->serial_mailbox_length > OUTPUT_BATCH_CAPACITY) {
    __atomic_store_n(&state->serial_mailbox_lock, 0u, __ATOMIC_RELEASE);
    return false;
  }

  header->length = state->serial_mailbox_length;
  header->sequence = __atomic_load_n(&state->serial_mailbox_sequence,
                                      __ATOMIC_RELAXED);
  memcpy(buffer, state->serial_mailbox, header->length);
  *seen_generation = generation;
  __atomic_store_n(&state->serial_mailbox_lock, 0u, __ATOMIC_RELEASE);
  return true;
}

static void serial_mailbox_finish(struct mux_state *state,
                                  uint32_t generation) {
  lock_serial_mailbox(state);

  if (__atomic_load_n(&state->serial_mailbox_generation,
                      __ATOMIC_RELAXED) == generation)
    __atomic_store_n(&state->serial_mailbox_active, 0u, __ATOMIC_RELEASE);
  __atomic_store_n(&state->serial_mailbox_lock, 0u, __ATOMIC_RELEASE);
}

static int serial_write_frame(int output, const unsigned char *buffer,
                              const struct output_frame_header *header,
                              struct mux_state *state, int display_wake) {
  for (size_t offset = 0; offset < header->length;) {
    size_t chunk = header->length - offset;

    if (serial_mailbox_newer(state, header->sequence))
      return 0;
    if (wait_for_display_idle(state, display_wake) < 0)
      return -1;
    if (serial_mailbox_newer(state, header->sequence))
      return 0;
    if (pv_console_registers == NULL && chunk > SERIAL_WRITE_QUANTUM)
      chunk = SERIAL_WRITE_QUANTUM;
    if (write_serial_output(output, buffer + offset, chunk) < 0)
      return -1;
    offset += chunk;

    /* Bound the amount of 8250 TX interrupt work released at once. With no
     * display work runnable this process is selected again immediately and
     * still reaches line rate. */
    (void)sched_yield();
  }
  return 0;
}

static void output_worker(int output, int queue_read, struct mux_state *state,
                          int display_wake, bool is_display) {
  unsigned char buffer[OUTPUT_BATCH_CAPACITY];
  uint32_t mailbox_seen = 0;

  for (;;) {
    struct output_frame_header header;

    if (read_all(queue_read, &header, sizeof(header)) < 0)
      break;
    if (header.length == 0 || header.length > sizeof(buffer) ||
        read_all(queue_read, buffer, header.length) < 0)
      break;
    if (is_display) {
      if (write_all(output, buffer, header.length) < 0)
        goto finished;
      mark_display_complete(state, display_wake, header.sequence);
      continue;
    }

    if (!serial_mailbox_newer(state, header.sequence) &&
        serial_write_frame(output, buffer, &header, state, display_wake) < 0)
      goto finished;

    /* If overload discarded queued UART frames, resume from the most recent
     * complete terminal burst instead of replaying stale animation output. */
    while (serial_mailbox_load(state, &mailbox_seen, &header, buffer)) {
      if (serial_write_frame(output, buffer, &header, state, display_wake) < 0)
        goto finished;
      serial_mailbox_finish(state, mailbox_seen);
    }
  }

finished:
  close(queue_read);
  close(output);
  _exit(0);
}

static pid_t start_output_worker(int output, int other_output, int queue_read,
                                 int queue_write, int other_queue_read,
                                 int other_queue_write, int nice_value,
                                 struct mux_state *state, int wake_read,
                                 int wake_write, bool is_display) {
  pid_t child = fork();

  if (child != 0)
    return child;

  close(queue_write);
  close(other_queue_read);
  close(other_queue_write);
  close(other_output);
  if (is_display)
    close(wake_read);
  else
    close(wake_write);
  if (nice_value != 0)
    (void)setpriority(PRIO_PROCESS, 0, nice_value);

  int flags = fcntl(queue_read, F_GETFL);
  if (flags >= 0)
    (void)fcntl(queue_read, F_SETFL, flags & ~O_NONBLOCK);

  output_worker(output, queue_read, state,
                is_display ? wake_write : wake_read, is_display);
  __builtin_unreachable();
}

static enum enqueue_result enqueue_frame(int queue, const void *buffer,
                                         size_t length, uint32_t sequence) {
  struct output_frame_header header;
  struct iovec vectors[2];
  ssize_t written;

  if (length == 0 || length > OUTPUT_BATCH_CAPACITY)
    return ENQUEUE_FAILED;

  header.length = (uint32_t)length;
  header.sequence = sequence;
  vectors[0].iov_base = &header;
  vectors[0].iov_len = sizeof(header);
  vectors[1].iov_base = (void *)buffer;
  vectors[1].iov_len = length;

  do {
    written = writev(queue, vectors, 2);
  } while (written < 0 && errno == EINTR);

  if (written == (ssize_t)(sizeof(header) + length))
    return ENQUEUE_OK;
  if (written < 0 && (errno == EAGAIN || errno == EWOULDBLOCK))
    return ENQUEUE_FULL;
  return ENQUEUE_FAILED;
}

static int forward_serial_input(int serial, int master) {
  unsigned char input[256];
  ssize_t length = read(serial, input, sizeof(input));

  if (length > 0)
    return master >= 0 ? write_all(master, input, (size_t)length) : 0;
  if (length < 0 && (errno == EINTR || errno == EAGAIN))
    return 0;
  return -1;
}

static int enqueue_display(int display_queue, int serial, int master,
                           const void *buffer, size_t length,
                           uint32_t sequence) {
  for (;;) {
    enum enqueue_result result =
        enqueue_frame(display_queue, buffer, length, sequence);

    if (result == ENQUEUE_OK)
      return 0;
    if (result == ENQUEUE_FAILED)
      return -1;

    /* Display output is lossless, but a full queue must not prevent Ctrl-C or
     * any other UART input from reaching the shell. */
    for (;;) {
      struct pollfd descriptors[2] = {
          {.fd = display_queue, .events = POLLOUT},
          {.fd = serial, .events = master >= 0 ? POLLIN : 0},
      };
      int ready = poll(descriptors, 2, -1);

      if (ready < 0 && errno == EINTR)
        continue;
      if (ready < 0 ||
          (descriptors[0].revents & (POLLERR | POLLHUP | POLLNVAL)))
        return -1;
      if ((descriptors[1].revents & POLLIN) &&
          forward_serial_input(serial, master) < 0)
        return -1;
      if (descriptors[0].revents & POLLOUT)
        break;
    }
  }
}

static int mirror_output(int display_queue, int serial_queue, int serial,
                         int master, struct mux_state *state,
                         const void *buffer, size_t length) {
  enum enqueue_result serial_result;
  uint32_t sequence = __atomic_add_fetch(&state->display_submitted, 1u,
                                         __ATOMIC_RELEASE);

  if (enqueue_display(display_queue, serial, master, buffer, length,
                      sequence) < 0)
    return -1;

  /* UART is a diagnostic mirror. Queue it independently and never make tty0
   * wait for an 8250 transmit slot. Atomic pipe frames ensure overload drops
   * a complete burst rather than corrupting the queue protocol. */
  if (serial_mailbox_replace_if_active(state, buffer, length, sequence)) {
    serial_result = ENQUEUE_FULL;
  } else {
    serial_result = enqueue_frame(serial_queue, buffer, length, sequence);
    if (serial_result == ENQUEUE_FULL)
      serial_mailbox_store(state, buffer, length, sequence);
  }

  /* The router and display worker share one emulated CPU. The serial worker is
   * lower priority, so this handoff selects tty0 whenever both wake. */
  (void)sched_yield();

  /* Losing a UART mirror burst or its worker must not terminate the shell. */
  (void)serial_result;
  return 0;
}

static ssize_t collect_display_burst(int master, int serial,
                                     unsigned char *buffer,
                                     size_t capacity) {
  size_t length = 0;

  while (length < capacity) {
    ssize_t count = read(master, buffer + length, capacity - length);

    if (count > 0) {
      length += (size_t)count;
    } else if (count == 0) {
      break;
    } else if (errno == EINTR) {
      continue;
    } else {
      return length ? (ssize_t)length : -1;
    }
    if (length == capacity)
      break;

    /* Drain output which is already ready into the same transaction, but do
     * not arm a timer waiting for more. UART RX remains live while adjacent
     * writes are assembled. */
    for (;;) {
      struct pollfd descriptors[2] = {
          {.fd = master, .events = POLLIN},
          {.fd = serial, .events = POLLIN},
      };
      int ready = poll(descriptors, 2, 0);

      if (ready < 0 && errno == EINTR)
        continue;
      if (ready <= 0)
        return (ssize_t)length;
      if (descriptors[1].revents & POLLIN) {
        unsigned char input[256];
        ssize_t input_length = read(serial, input, sizeof(input));

        if (input_length > 0 &&
            write_all(master, input, (size_t)input_length) < 0)
          return -1;
      }
      if (descriptors[0].revents & POLLIN)
        break;
      if (descriptors[0].revents & (POLLERR | POLLHUP | POLLNVAL))
        return (ssize_t)length;
    }
  }

  return (ssize_t)length;
}

static pid_t start_shell(int master, const char *slave_name) {
  pid_t child = fork();

  if (child != 0)
    return child;

  if (setsid() < 0)
    _exit(126);

  int slave = open(slave_name, O_RDWR);
  if (slave < 0)
    _exit(126);
  if (ioctl(slave, TIOCSCTTY, 0) < 0)
    _exit(126);

  close(master);
  if (dup2(slave, STDIN_FILENO) < 0 || dup2(slave, STDOUT_FILENO) < 0 ||
      dup2(slave, STDERR_FILENO) < 0)
    _exit(126);
  if (slave > STDERR_FILENO)
    close(slave);

  /* A login shell loads OpenWrt's banner, profile, prompt and proxy setup. */
  execl("/bin/ash", "-ash", "-i", (char *)NULL);
  _exit(127);
}

static int run_shell(int display_queue, int serial_queue, int serial,
                     struct mux_state *state) {
  int master = posix_openpt(O_RDWR | O_NOCTTY | O_CLOEXEC);
  const struct winsize display_size = {
      .ws_row = 45,
      .ws_col = 90,
      .ws_xpixel = 720,
      .ws_ypixel = 720,
  };
  char *slave_name;
  pid_t child;

  if (master < 0 || grantpt(master) < 0 || unlockpt(master) < 0)
    return -1;
  if (ioctl(master, TIOCSWINSZ, &display_size) < 0)
    return -1;
  slave_name = ptsname(master);
  if (slave_name == NULL)
    return -1;

  child = start_shell(master, slave_name);
  if (child < 0)
    return -1;

  for (;;) {
    struct pollfd descriptors[2] = {
        {.fd = master, .events = POLLIN},
        {.fd = serial, .events = POLLIN},
    };
    unsigned char buffer[OUTPUT_BATCH_CAPACITY];
    int ready = poll(descriptors, 2, 250);

    if (ready < 0 && errno != EINTR)
      break;
    if (ready > 0 && (descriptors[1].revents & POLLIN)) {
      ssize_t count = read(serial, buffer, sizeof(buffer));

      if (count > 0 && write_all(master, buffer, (size_t)count) < 0)
        break;
    }
    if (ready > 0 && (descriptors[0].revents & POLLIN)) {
      ssize_t count = collect_display_burst(master, serial, buffer,
                                             sizeof(buffer));

      if (count <= 0)
        break;
      if (mirror_output(display_queue, serial_queue, serial, master, state,
                        buffer, (size_t)count) < 0)
        break;
    }

    int status;
    if (waitpid(child, &status, WNOHANG) == child)
      break;
  }

  close(master);
  if (waitpid(child, NULL, WNOHANG) == 0) {
    kill(child, SIGHUP);
    (void)waitpid(child, NULL, 0);
  }
  return 0;
}

int main(void) {
  int display = open("/dev/tty0", O_WRONLY | O_NOCTTY | O_CLOEXEC);
  int serial = open("/dev/ttyS0", O_RDWR | O_NOCTTY | O_CLOEXEC);
  int display_pipe[2];
  int serial_pipe[2];
  int display_wake_pipe[2];
  struct mux_state *state;
  pid_t display_worker;
  pid_t serial_worker;

  if (display < 0 || serial < 0) {
    perror("console-mux: open");
    return 1;
  }
  if (make_raw_serial(serial) < 0) {
    perror("console-mux: ttyS0");
    return 1;
  }
  init_paravirtual_console();
  state = mmap(NULL, sizeof(*state), PROT_READ | PROT_WRITE,
               MAP_SHARED | MAP_ANONYMOUS, -1, 0);
  if (state == MAP_FAILED) {
    perror("console-mux: shared state");
    return 1;
  }
  if (pipe2(display_pipe, O_CLOEXEC | O_NONBLOCK) < 0 ||
      pipe2(serial_pipe, O_CLOEXEC | O_NONBLOCK) < 0 ||
      pipe2(display_wake_pipe, O_CLOEXEC | O_NONBLOCK) < 0) {
    perror("console-mux: output pipe");
    return 1;
  }
#ifdef F_SETPIPE_SZ
  /* Best effort: older or size-limited kernels keep their default capacity. */
  (void)fcntl(display_pipe[1], F_SETPIPE_SZ, DISPLAY_PIPE_CAPACITY);
  (void)fcntl(serial_pipe[1], F_SETPIPE_SZ, SERIAL_PIPE_CAPACITY);
#endif
  (void)signal(SIGPIPE, SIG_IGN);
  display_worker = start_output_worker(
      display, serial, display_pipe[0], display_pipe[1], serial_pipe[0],
      serial_pipe[1], 0, state, display_wake_pipe[0],
      display_wake_pipe[1], true);
  if (display_worker < 0) {
    perror("console-mux: display worker");
    return 1;
  }
  serial_worker = start_output_worker(
      serial, display, serial_pipe[0], serial_pipe[1], display_pipe[0],
      display_pipe[1], SERIAL_WORKER_NICE, state,
      display_wake_pipe[0], display_wake_pipe[1], false);
  if (serial_worker < 0) {
    perror("console-mux: serial worker");
    kill(display_worker, SIGTERM);
    return 1;
  }
  close(display_pipe[0]);
  close(serial_pipe[0]);
  close(display_wake_pipe[0]);
  close(display_wake_pipe[1]);
  close(display);

  for (;;) {
    if (run_shell(display_pipe[1], serial_pipe[1], serial, state) < 0) {
      static const char failure[] =
          "\r\nconsole-mux: failed to start shell\r\n";

      (void)mirror_output(display_pipe[1], serial_pipe[1], serial, -1, state,
                          failure, sizeof(failure) - 1);
    }
    usleep(250000);
  }
}
