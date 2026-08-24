/*
 * Give the framebuffer VT and emulated 16550 UART one interactive shell.
 *
 * The shell owns a pseudo-terminal.  Its output is copied to both tty0 and
 * ttyS0, while UART input is sent through the PTY line discipline so echo,
 * editing and signals appear identically on both displays.
 *
 * SPDX-License-Identifier: BSD-3-Clause
 */

#define _GNU_SOURCE

#include <errno.h>
#include <fcntl.h>
#include <poll.h>
#include <signal.h>
#include <stdbool.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <sys/ioctl.h>
#include <sys/types.h>
#include <sys/wait.h>
#include <termios.h>
#include <unistd.h>

static int write_all(int fd, const void *buffer, size_t length)
{
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

static int make_raw_serial(int fd)
{
	struct termios settings;

	if (tcgetattr(fd, &settings) < 0)
		return -1;
	cfmakeraw(&settings);
	settings.c_cflag |= CLOCAL | CREAD;
	cfsetispeed(&settings, B115200);
	cfsetospeed(&settings, B115200);
	return tcsetattr(fd, TCSANOW, &settings);
}

static pid_t start_shell(int master, const char *slave_name)
{
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
	if (dup2(slave, STDIN_FILENO) < 0 ||
	    dup2(slave, STDOUT_FILENO) < 0 ||
	    dup2(slave, STDERR_FILENO) < 0)
		_exit(126);
	if (slave > STDERR_FILENO)
		close(slave);

	/* A login shell loads OpenWrt's banner, profile, prompt and proxy setup. */
	execl("/bin/ash", "-ash", "-i", (char *)NULL);
	_exit(127);
}

static int run_shell(int display, int serial)
{
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
			{ .fd = master, .events = POLLIN },
			{ .fd = serial, .events = POLLIN },
		};
		unsigned char buffer[256];
		int ready = poll(descriptors, 2, 250);

		if (ready < 0 && errno != EINTR)
			break;
		if (ready > 0 && (descriptors[0].revents & POLLIN)) {
			ssize_t count = read(master, buffer, sizeof(buffer));

			if (count <= 0)
				break;
			(void)write_all(display, buffer, (size_t)count);
			(void)write_all(serial, buffer, (size_t)count);
		}
		if (ready > 0 && (descriptors[1].revents & POLLIN)) {
			ssize_t count = read(serial, buffer, sizeof(buffer));

			if (count > 0 && write_all(master, buffer, (size_t)count) < 0)
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

int main(void)
{
	int display = open("/dev/tty0", O_WRONLY | O_NOCTTY);
	int serial = open("/dev/ttyS0", O_RDWR | O_NOCTTY);

	if (display < 0 || serial < 0) {
		perror("console-mux: open");
		return 1;
	}
	if (make_raw_serial(serial) < 0) {
		perror("console-mux: ttyS0");
		return 1;
	}

	for (;;) {
		if (run_shell(display, serial) < 0) {
			static const char failure[] =
				"\r\nconsole-mux: failed to start shell\r\n";

			(void)write_all(display, failure, sizeof(failure) - 1);
			(void)write_all(serial, failure, sizeof(failure) - 1);
		}
		usleep(250000);
	}
}
