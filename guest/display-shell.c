/*
 * Prefer the standard virtio-console hvc terminal. The host delivers UART0
 * RX through its input virtqueue and renders hvc output on CPU1, avoiding the
 * guest fbcon pixel path. Linux translates USB HID keyboard events through
 * its normal VT keymap; this process forwards the resulting tty1 bytes to the
 * same hvc terminal. Older kernels retain the tty0 + TIOCSTI fallback.
 *
 * tty1 stands in for the hvc terminal: Ctrl+Alt+F2..F6 switch to fbcon
 * logins and F7 to X, and the host hides its terminal while they are shown.
 *
 * SPDX-License-Identifier: BSD-3-Clause
 */

#define _GNU_SOURCE

#include <errno.h>
#include <fcntl.h>
#include <linux/kd.h>
#include <linux/vt.h>
#include <poll.h>
#include <signal.h>
#include <stdbool.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <sys/ioctl.h>
#include <sys/stat.h>
#include <sys/types.h>
#include <sys/wait.h>
#include <termios.h>
#include <unistd.h>

#define INPUT_CHUNK 128
#define DISPLAY_INPUT "/dev/ttyD"
#define TERMINAL_VT "/dev/tty1"
#define FIRST_LOGIN_VT 2
#define LAST_LOGIN_VT 6
/* Private modes understood by the host terminal (display_bridge.c). */
#define HIDE_TERMINAL "\033[?7700h"
#define SHOW_TERMINAL "\033[?7700l"

enum { EVENT_CHILD = 'C', EVENT_VT_RELEASE = 'R', EVENT_VT_ACQUIRE = 'A' };

static int child_event_write = -1;
static pid_t vt_logins[LAST_LOGIN_VT + 1];

static void console_error(int console, const char *message);

static void post_event(unsigned char event)
{
	int saved_errno = errno;

	if (child_event_write >= 0)
		(void)write(child_event_write, &event, sizeof(event));
	errno = saved_errno;
}

static void child_event(int signal_number)
{
	(void)signal_number;
	post_event(EVENT_CHILD);
}

static void vt_release_event(int signal_number)
{
	(void)signal_number;
	post_event(EVENT_VT_RELEASE);
}

static void vt_acquire_event(int signal_number)
{
	(void)signal_number;
	post_event(EVENT_VT_ACQUIRE);
}

static int configure_uart_input(int serial)
{
	struct termios settings;

	if (tcgetattr(serial, &settings) < 0)
		return -1;
	cfmakeraw(&settings);
	settings.c_cflag |= CLOCAL | CREAD;
	settings.c_cc[VMIN] = 1;
	settings.c_cc[VTIME] = 0;
	cfsetispeed(&settings, B921600);
	cfsetospeed(&settings, B921600);
	return tcsetattr(serial, TCSANOW, &settings);
}

static int configure_vt_input(int vt)
{
	struct termios settings;

	if (tcgetattr(vt, &settings) < 0)
		return -1;

	/* Keep the kernel VT keyboard translator and keymap, but expose its
	 * translated bytes immediately. Echo and signal handling belong to the
	 * foreground hvc terminal, not this input-only bridge. */
	settings.c_iflag &= ~(IGNBRK | BRKINT | PARMRK | ISTRIP | INLCR |
				IGNCR | ICRNL | IXON | IXOFF | IXANY);
	settings.c_lflag &= ~(ECHO | ECHONL | ICANON | ISIG | IEXTEN);
	settings.c_cflag |= CLOCAL | CREAD;
	settings.c_cc[VMIN] = 1;
	settings.c_cc[VTIME] = 0;

	return tcsetattr(vt, TCSANOW, &settings);
}

static int configure_console(int console, bool host_rendered)
{
	struct termios settings;

	if (tcgetattr(console, &settings) < 0)
		return -1;

	/* UART CR becomes an input newline; console newlines render as CRLF. */
	settings.c_iflag &= ~(IGNCR | INLCR | IXON | IXOFF | IXANY);
	settings.c_iflag |= ICRNL;
	settings.c_oflag &= ~(OCRNL | ONOCR | ONLRET | ONLCR);
	if (host_rendered)
		settings.c_oflag &= ~OPOST;
	else
		settings.c_oflag |= OPOST | ONLCR;

	/* Let tty0 own echo, editing, signals, and foreground job control. */
	settings.c_lflag &= ~NOFLSH;
	settings.c_lflag |= ICANON | ISIG | IEXTEN | ECHO | ECHOE | ECHOK;
	settings.c_cc[VINTR] = 3;   /* Ctrl-C */
	settings.c_cc[VQUIT] = 28;  /* Ctrl-\\ */
	settings.c_cc[VERASE] = 127;
	settings.c_cc[VKILL] = 21;  /* Ctrl-U */
	settings.c_cc[VEOF] = 4;    /* Ctrl-D */
	settings.c_cc[VSUSP] = 26;  /* Ctrl-Z */

	return tcsetattr(console, TCSANOW, &settings);
}

static int open_display_input(void)
{
	(void)unlink(DISPLAY_INPUT);
	if (mkfifo(DISPLAY_INPUT, 0600) < 0)
		return -1;
	/* O_RDWR prevents writer disconnects from causing a POLLHUP busy loop. */
	return open(DISPLAY_INPUT, O_RDWR | O_NONBLOCK | O_CLOEXEC);
}

static void inject_console_input(int console, const unsigned char *input,
				 ssize_t length, bool *error_reported)
{
	for (ssize_t index = 0; index < length; index++) {
		if (ioctl(console, TIOCSTI, &input[index]) == 0) {
			*error_reported = false;
			continue;
		}
		if (!*error_reported) {
			console_error(console, "console input injection failed");
			*error_reported = true;
		}
		break;
	}
}

static void console_error(int console, const char *message)
{
	int saved_errno = errno;

	dprintf(console, "\r\ndisplay-shell: %s: %s\r\n", message,
		strerror(saved_errno));
	errno = saved_errno;
}

static void fatal_idle(int console, const char *message)
	__attribute__((noreturn));

static void fatal_idle(int console, const char *message)
{
	if (console >= 0)
		console_error(console, message);
	else
		dprintf(STDERR_FILENO, "display-shell: %s: %s\n", message,
			strerror(errno));

	/* Idle rather than exit, which would only make init restart us. */
	for (;;)
		pause();
}

static pid_t start_login(const char *console_path, int child_event_read)
{
	pid_t child = fork();

	if (child != 0)
		return child;

	if (child_event_read >= 0)
		close(child_event_read);
	close(child_event_write);

	if (setsid() < 0)
		_exit(126);

	int console = open(console_path, O_RDWR | O_NOCTTY);

	if (console < 0 ||
	    configure_console(console, strcmp(console_path, "/dev/hvc0") == 0) < 0 ||
	    ioctl(console, TIOCSCTTY, 0) < 0 ||
	    tcsetpgrp(console, getpgrp()) < 0)
		_exit(126);
	if (dup2(console, STDIN_FILENO) < 0 ||
	    dup2(console, STDOUT_FILENO) < 0 ||
	    dup2(console, STDERR_FILENO) < 0)
		_exit(126);
	if (console > STDERR_FILENO)
		close(console);

	/* BusyBox login applies the root account's configured shell and profile. */
	char *const login_argv[] = { "login", "-f", "root", NULL };

	execv("/bin/login", login_argv);
	_exit(127);
}

static pid_t start_login_reliably(int console, const char *console_path,
				  int child_event_read)
{
	pid_t login_pid;

	while ((login_pid = start_login(console_path, child_event_read)) < 0) {
		console_error(console, "cannot start login; retrying");
		usleep(250000);
	}
	return login_pid;
}

static void start_vt_login(int vt, int child_event_read)
{
	char path[16];

	snprintf(path, sizeof(path), "/dev/tty%d", vt);
	vt_logins[vt] = start_login(path, child_event_read);
	if (vt_logins[vt] < 0)
		vt_logins[vt] = 0;
}

static pid_t reap_children(int console, const char *console_path,
			   int child_event_read, pid_t login_pid)
{
	pid_t child;
	int status;
	bool login_exited = false;

	while ((child = waitpid(-1, &status, WNOHANG)) > 0) {
		if (child == login_pid)
			login_exited = true;
		for (int vt = FIRST_LOGIN_VT; vt <= LAST_LOGIN_VT; vt++) {
			if (child == vt_logins[vt]) {
				usleep(250000);
				start_vt_login(vt, child_event_read);
			}
		}
	}

	if (!login_exited)
		return login_pid;

	/* Delay only a failed/exited login session, never the normal data path. */
	usleep(250000);
	return start_login_reliably(console, console_path, child_event_read);
}

/* Keep fbcon off tty1 and get a signal on every switch away and back. */
static bool take_terminal_vt(int vt)
{
	struct vt_mode mode = {
		.mode = VT_PROCESS,
		.relsig = SIGUSR1,
		.acqsig = SIGUSR2,
	};

	return ioctl(vt, KDSETMODE, KD_GRAPHICS) == 0 &&
	       ioctl(vt, VT_SETMODE, &mode) == 0;
}

static void write_console(int console, const char *text)
{
	(void)write(console, text, strlen(text));
}

static void release_terminal_vt(int console, int vt)
{
	/* Let the host hide the terminal before the next VT draws. */
	write_console(console, HIDE_TERMINAL);
	(void)tcdrain(console);
	usleep(20000);
	if (ioctl(vt, VT_RELDISP, 1) < 0)
		console_error(console, "VT release failed");
}

static void acquire_terminal_vt(int console, int vt)
{
	if (ioctl(vt, VT_RELDISP, VT_ACKACQ) < 0)
		console_error(console, "VT acquire failed");
	write_console(console, SHOW_TERMINAL);
}

/* The active VT from sysfs, or 0 if unknown; rereading rearms poll. */
static int read_active_vt(int active)
{
	char name[16];
	ssize_t length;

	if (lseek(active, 0, SEEK_SET) < 0)
		return 0;
	length = read(active, name, sizeof(name) - 1);
	if (length <= 0)
		return 0;
	name[length] = '\0';
	return strncmp(name, "tty", 3) == 0 ? atoi(name + 3) : 0;
}

int main(void)
{
	struct sigaction action = { 0 };
	struct pollfd watched[5];
	unsigned char input[INPUT_CHUNK];
	int child_events[2];
	int console;
	int serial;
	int serial_watch = -1;
	int vt_input = -1;
	int vt_input_watch = -1;
	int display_input;
	int display_input_watch = -1;
	int active = -1;
	int active_watch = -1;
	int child_watch;
	int watched_count = 0;
	const char *console_path = "/dev/hvc0";
	pid_t login_pid;
	bool injection_error_reported = false;

	console = open(console_path, O_RDWR | O_NOCTTY | O_CLOEXEC);
	serial = -1;
	if (console < 0) {
		console_path = "/dev/tty0";
		console = open(console_path, O_RDWR | O_NOCTTY | O_CLOEXEC);
		serial = open("/dev/ttyS0", O_RDONLY | O_NOCTTY | O_CLOEXEC);
	}
	if (console < 0 || (serial < 0 && strcmp(console_path, "/dev/tty0") == 0))
		fatal_idle(console, "cannot open hvc0 or tty0 fallback");
	if (configure_console(console, strcmp(console_path, "/dev/hvc0") == 0) < 0 ||
	    (serial >= 0 && configure_uart_input(serial) < 0)) {
		fatal_idle(console, "terminal setup failed");
	}
	if (strcmp(console_path, "/dev/hvc0") == 0) {
		vt_input = open(TERMINAL_VT,
				O_RDWR | O_NOCTTY | O_NONBLOCK | O_CLOEXEC);
		if (vt_input < 0 || configure_vt_input(vt_input) < 0) {
			console_error(console, "cannot enable tty1 keyboard input");
			if (vt_input >= 0)
				close(vt_input);
			vt_input = -1;
		}
	}
	display_input = open_display_input();
	if (display_input < 0)
		console_error(console, "cannot create " DISPLAY_INPUT);
	if (pipe2(child_events, O_NONBLOCK | O_CLOEXEC) < 0) {
		fatal_idle(console, "child event pipe failed");
	}

	child_event_write = child_events[1];
	action.sa_handler = child_event;
	action.sa_flags = SA_NOCLDSTOP;
	sigemptyset(&action.sa_mask);
	if (sigaction(SIGCHLD, &action, NULL) < 0) {
		fatal_idle(console, "SIGCHLD setup failed");
	}
	action.sa_flags = 0;
	action.sa_handler = vt_release_event;
	if (sigaction(SIGUSR1, &action, NULL) < 0)
		fatal_idle(console, "SIGUSR1 setup failed");
	action.sa_handler = vt_acquire_event;
	if (sigaction(SIGUSR2, &action, NULL) < 0)
		fatal_idle(console, "SIGUSR2 setup failed");

	if (vt_input >= 0 && take_terminal_vt(vt_input)) {
		/* A VT must exist before the keyboard can switch to it. */
		for (int vt = FIRST_LOGIN_VT; vt <= LAST_LOGIN_VT; vt++) {
			char path[16];
			int fd;

			snprintf(path, sizeof(path), "/dev/tty%d", vt);
			fd = open(path, O_RDWR | O_NOCTTY | O_CLOEXEC);
			if (fd >= 0)
				close(fd);
		}
		active = open("/sys/class/tty/tty0/active",
			      O_RDONLY | O_CLOEXEC);
		/* Restarted while another VT is shown: keep the terminal hidden. */
		write_console(console, active >= 0 && read_active_vt(active) > 1 ?
				       HIDE_TERMINAL : SHOW_TERMINAL);
	} else if (vt_input >= 0) {
		console_error(console, "VT switching unavailable");
	}

	login_pid = start_login_reliably(console, console_path, child_events[0]);

	if (serial >= 0) {
		serial_watch = watched_count++;
		watched[serial_watch].fd = serial;
		watched[serial_watch].events = POLLIN;
	}
	if (vt_input >= 0) {
		vt_input_watch = watched_count++;
		watched[vt_input_watch].fd = vt_input;
		watched[vt_input_watch].events = POLLIN;
	}
	if (display_input >= 0) {
		display_input_watch = watched_count++;
		watched[display_input_watch].fd = display_input;
		watched[display_input_watch].events = POLLIN;
	}
	if (active >= 0) {
		active_watch = watched_count++;
		watched[active_watch].fd = active;
		watched[active_watch].events = POLLPRI;
	}
	child_watch = watched_count++;
	watched[child_watch].fd = child_events[0];
	watched[child_watch].events = POLLIN;

	for (;;) {
		int ready = poll(watched, watched_count, -1);

		if (ready < 0) {
			if (errno == EINTR)
				continue;
			console_error(console, "poll failed");
			continue;
		}

		if (watched[child_watch].revents & POLLIN) {
			unsigned char events[32];
			ssize_t count;
			bool child_exited = false;

			while ((count = read(child_events[0], events,
					     sizeof(events))) > 0) {
				for (ssize_t i = 0; i < count; i++) {
					if (events[i] == EVENT_VT_RELEASE)
						release_terminal_vt(console, vt_input);
					else if (events[i] == EVENT_VT_ACQUIRE)
						acquire_terminal_vt(console, vt_input);
					else
						child_exited = true;
				}
			}
			if (child_exited)
				login_pid = reap_children(console, console_path,
							  child_events[0],
							  login_pid);
		}

		/* Start a VT's login the first time it is shown. */
		if (active_watch >= 0 &&
		    (watched[active_watch].revents & (POLLPRI | POLLERR))) {
			int vt = read_active_vt(active);

			if (vt >= FIRST_LOGIN_VT && vt <= LAST_LOGIN_VT &&
			    vt_logins[vt] == 0)
				start_vt_login(vt, child_events[0]);
		}

		if (serial_watch >= 0 &&
		    (watched[serial_watch].revents & POLLIN)) {
			ssize_t received = read(serial, input, sizeof(input));

			if (received < 0 && errno != EINTR && errno != EAGAIN)
				console_error(console, "UART read failed");
			if (received > 0)
				inject_console_input(console, input, received,
						     &injection_error_reported);
		}

		if (vt_input_watch >= 0 &&
		    (watched[vt_input_watch].revents & POLLIN)) {
			ssize_t received = read(vt_input, input, sizeof(input));

			if (received < 0 && errno != EINTR && errno != EAGAIN)
				console_error(console, "tty1 keyboard read failed");
			if (received > 0)
				inject_console_input(console, input, received,
						     &injection_error_reported);
		}

		if (display_input_watch >= 0 &&
		    (watched[display_input_watch].revents & POLLIN)) {
			ssize_t received = read(display_input, input, sizeof(input));

			if (received < 0 && errno != EINTR && errno != EAGAIN)
				console_error(console, "display input FIFO read failed");
			if (received > 0)
				inject_console_input(console, input, received,
						     &injection_error_reported);
		}
	}
}
