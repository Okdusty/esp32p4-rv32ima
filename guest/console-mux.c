/*
 * Give the framebuffer VT and emulated 16550 UART one interactive shell.
 *
 * The shell owns a pseudo-terminal.  Its output is rendered by a tty0 worker
 * first and acknowledged back to the parent before the same bytes are written
 * to ttyS0.  UART and framebuffer output therefore advance in identical
 * batches instead of UART running ahead during a large fbcon redraw.  UART
 * input remains polled while an acknowledgement is pending and is sent through
 * the PTY line discipline so editing and signals remain responsive and normal.
 *
 * SPDX-License-Identifier: BSD-3-Clause
 */

#define _GNU_SOURCE

#include <errno.h>
#include <fcntl.h>
#include <linux/fb.h>
#include <poll.h>
#include <signal.h>
#include <stdbool.h>
#include <stdint.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <sys/ioctl.h>
#include <sys/mman.h>
#include <sys/types.h>
#include <sys/wait.h>
#include <termios.h>
#include <unistd.h>

#include "display_accel_protocol.h"

#define DISPLAY_BATCH_CAPACITY (8u * 1024u)
#define DISPLAY_COALESCE_MS 2

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

static volatile uint32_t *map_display_fence(void) {
  struct fb_fix_screeninfo fixed;
  struct fb_var_screeninfo variable;
  size_t visible_bytes;
  size_t commit_offset;
  unsigned char *mapping;
  int framebuffer = open("/dev/fb0", O_RDWR | O_CLOEXEC);

  if (framebuffer < 0 || ioctl(framebuffer, FBIOGET_FSCREENINFO, &fixed) < 0 ||
      ioctl(framebuffer, FBIOGET_VSCREENINFO, &variable) < 0 ||
      fixed.line_length == 0 || variable.yres > SIZE_MAX / fixed.line_length) {
    if (framebuffer >= 0)
      close(framebuffer);
    return NULL;
  }
  visible_bytes = (size_t)fixed.line_length * variable.yres;
  commit_offset = visible_bytes + DISPLAY_ACCEL_STAGE_SIZE +
                  3u * sizeof(uint32_t);
  if (commit_offset > fixed.smem_len ||
      sizeof(uint32_t) > fixed.smem_len - commit_offset) {
    close(framebuffer);
    return NULL;
  }

  mapping = mmap(NULL, fixed.smem_len, PROT_READ | PROT_WRITE, MAP_SHARED,
                 framebuffer, 0);
  close(framebuffer);
  if (mapping == MAP_FAILED)
    return NULL;
  return (volatile uint32_t *)(mapping + commit_offset);
}

static void wait_for_physical_frame(volatile uint32_t *display_fence) {
  uint32_t completed;

  if (display_fence == NULL)
    return;
  completed = *display_fence;
  *display_fence = DISPLAY_FB_COMMIT_SYNC;
  /* Six 59 Hz periods are ample; degrade to UART instead of ever wedging the
   * interactive shell if the native display service disappears. */
  for (unsigned int retry = 0; retry < 100; retry++) {
    if (*display_fence != completed)
      return;
    usleep(1000);
  }
}

static pid_t start_display_worker(int display, int serial, int pipe_read,
                                  int pipe_write, int ack_read, int ack_write,
                                  volatile uint32_t *display_fence) {
  pid_t child = fork();

  if (child != 0)
    return child;

  close(pipe_write);
  close(ack_read);
  close(serial);
  /* Run at normal guest priority so tty0 can consume display bursts promptly. */

  int flags = fcntl(pipe_read, F_GETFL);
  if (flags >= 0)
    (void)fcntl(pipe_read, F_SETFL, flags & ~O_NONBLOCK);

  for (;;) {
    unsigned char buffer[DISPLAY_BATCH_CAPACITY];
    uint32_t length;
    static const unsigned char rendered = 1;

    /* A length prefix preserves one logical tty burst even when the pipe
     * splits a multi-kilobyte write. Each burst therefore costs one physical
     * frame fence instead of one fence per pipe read. */
    if (read_all(pipe_read, &length, sizeof(length)) < 0)
      break;
    if (length == 0 || length > sizeof(buffer) ||
        read_all(pipe_read, buffer, length) < 0)
      break;
    if (write_all(display, buffer, length) < 0)
      break;
    wait_for_physical_frame(display_fence);
    if (write_all(ack_write, &rendered, sizeof(rendered)) < 0)
      break;
  }

  close(pipe_read);
  close(ack_write);
  close(display);
  _exit(0);
}

static int wait_until_rendered(int display_ack, int serial, int master) {
  for (;;) {
    struct pollfd descriptors[2] = {
        {.fd = display_ack, .events = POLLIN},
        {.fd = serial, .events = master >= 0 ? POLLIN : 0},
    };
    int ready = poll(descriptors, 2, -1);

    if (ready < 0) {
      if (errno == EINTR)
        continue;
      return -1;
    }
    if (descriptors[1].revents & POLLIN) {
      unsigned char input[256];
      ssize_t count = read(serial, input, sizeof(input));

      if (count > 0 && write_all(master, input, (size_t)count) < 0)
        return -1;
    }
    if (descriptors[0].revents & POLLIN) {
      unsigned char rendered;
      ssize_t count = read(display_ack, &rendered, sizeof(rendered));

      if (count == sizeof(rendered))
        return 0;
      if (count < 0 && errno == EINTR)
        continue;
      return -1;
    }
    if (descriptors[0].revents & (POLLERR | POLLHUP | POLLNVAL))
      return -1;
  }
}

static int mirror_output(int display_queue, int display_ack, int serial,
                         int master, const void *buffer, size_t length) {
  uint32_t batch_length;

  if (length == 0 || length > DISPLAY_BATCH_CAPACITY)
    return -1;
  batch_length = (uint32_t)length;
  /* Only one batch is in flight, so one acknowledgement maps exactly to it. */
  if (write_all(display_queue, &batch_length, sizeof(batch_length)) == 0 &&
      write_all(display_queue, buffer, length) == 0 &&
      wait_until_rendered(display_ack, serial, master) == 0)
    return write_all(serial, buffer, length);

  /* Preserve a usable serial shell if the framebuffer worker ever exits. */
  return write_all(serial, buffer, length);
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

    /* Shells commonly emit one line or escape sequence per write. Give those
     * adjacent writes a tiny window to form one display transaction. UART RX
     * remains live while the burst is being assembled. */
    for (;;) {
      struct pollfd descriptors[2] = {
          {.fd = master, .events = POLLIN},
          {.fd = serial, .events = POLLIN},
      };
      int ready = poll(descriptors, 2, DISPLAY_COALESCE_MS);

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

static int run_shell(int display_queue, int display_ack, int serial) {
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
    unsigned char buffer[DISPLAY_BATCH_CAPACITY];
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
      if (mirror_output(display_queue, display_ack, serial, master, buffer,
                        (size_t)count) < 0)
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
  int display_ack[2];
  volatile uint32_t *display_fence;
  pid_t display_worker;

  if (display < 0 || serial < 0) {
    perror("console-mux: open");
    return 1;
  }
  if (make_raw_serial(serial) < 0) {
    perror("console-mux: ttyS0");
    return 1;
  }
  display_fence = map_display_fence();
  if (pipe2(display_pipe, O_CLOEXEC) < 0 ||
      pipe2(display_ack, O_CLOEXEC) < 0) {
    perror("console-mux: display pipe");
    return 1;
  }
  (void)signal(SIGPIPE, SIG_IGN);
  display_worker = start_display_worker(display, serial, display_pipe[0],
                                        display_pipe[1], display_ack[0],
                                        display_ack[1], display_fence);
  if (display_worker < 0) {
    perror("console-mux: display worker");
    return 1;
  }
  close(display_pipe[0]);
  close(display_ack[1]);
  close(display);

  for (;;) {
    if (run_shell(display_pipe[1], display_ack[0], serial) < 0) {
      static const char failure[] =
          "\r\nconsole-mux: failed to start shell\r\n";

      (void)mirror_output(display_pipe[1], display_ack[0], serial, -1,
                          failure, sizeof(failure) - 1);
    }
    usleep(250000);
  }
}
