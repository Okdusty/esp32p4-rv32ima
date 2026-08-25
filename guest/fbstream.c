/*
 * Minimal raw RGB565 stream player for the Linux simple framebuffer.
 *
 * Input is a sequence of tightly packed little-endian RGB565 frames.  The
 * image is nearest-neighbour scaled to fit the framebuffer while preserving
 * aspect ratio.  Pair it with BusyBox nc so no media framework is required in
 * the RV32 guest.
 *
 * SPDX-License-Identifier: BSD-3-Clause
 */

#include <errno.h>
#include <fcntl.h>
#include <linux/fb.h>
#include <linux/kd.h>
#include <signal.h>
#include <stdbool.h>
#include <stdint.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <sys/ioctl.h>
#include <sys/mman.h>
#include <unistd.h>

#include "display_accel_protocol.h"

static volatile sig_atomic_t stop_requested;
static int console_fd = -1;

static void restore_console(void)
{
	if (console_fd < 0)
		return;

	(void)ioctl(console_fd, KDSETMODE, KD_TEXT);
	(void)write(console_fd, "\033[?25h", sizeof("\033[?25h") - 1u);
	close(console_fd);
	console_fd = -1;
}

static void handle_signal(int signal_number)
{
	(void)signal_number;
	stop_requested = 1;
}

static int read_exact(int fd, void *buffer, size_t length)
{
	uint8_t *cursor = buffer;

	while (length && !stop_requested) {
		ssize_t count = read(fd, cursor, length);

		if (count > 0) {
			cursor += count;
			length -= (size_t)count;
			continue;
		}
		if (count < 0 && errno == EINTR)
			continue;
		return count == 0 ? 0 : -1;
	}
	return length == 0 ? 1 : 0;
}

static unsigned int parse_dimension(const char *text, const char *name)
{
	char *end;
	unsigned long value = strtoul(text, &end, 10);

	if (*text == '\0' || *end != '\0' || value == 0 || value > 4096) {
		fprintf(stderr, "fbstream: invalid %s: %s\n", name, text);
		exit(2);
	}
	return (unsigned int)value;
}

int main(int argc, char **argv)
{
	unsigned int source_width = 256;
	unsigned int source_height = 144;
	struct fb_fix_screeninfo fixed;
	struct fb_var_screeninfo variable;
	uint8_t *framebuffer;
	uint16_t *source = NULL;
	uint16_t *scaled_row = NULL;
	unsigned int *column_map = NULL;
	uint8_t *accelerator_staging = NULL;
	volatile uint32_t *accelerator_command = NULL;
	volatile uint32_t *framebuffer_commit = NULL;
	unsigned int output_width;
	unsigned int output_height;
	unsigned int output_left;
	unsigned int output_top;
	size_t source_pixels;
	size_t source_bytes;
	size_t visible_bytes;
	int framebuffer_fd;
	uint64_t frame_count = 0;
	bool accelerated = false;
	struct sigaction signal_action = {
		.sa_handler = handle_signal,
	};

	if (argc == 3) {
		source_width = parse_dimension(argv[1], "width");
		source_height = parse_dimension(argv[2], "height");
	} else if (argc != 1) {
		fprintf(stderr, "usage: fbstream [width height] < rgb565-stream\n");
		return 2;
	}

	if ((size_t)source_width > SIZE_MAX / source_height) {
		fprintf(stderr, "fbstream: frame dimensions overflow\n");
		return 2;
	}
	source_pixels = (size_t)source_width * source_height;
	if (source_pixels > SIZE_MAX / sizeof(*source)) {
		fprintf(stderr, "fbstream: frame size overflow\n");
		return 2;
	}
	source_bytes = source_pixels * sizeof(*source);

	framebuffer_fd = open("/dev/fb0", O_RDWR | O_CLOEXEC);
	if (framebuffer_fd < 0 ||
	    ioctl(framebuffer_fd, FBIOGET_FSCREENINFO, &fixed) < 0 ||
	    ioctl(framebuffer_fd, FBIOGET_VSCREENINFO, &variable) < 0) {
		perror("fbstream: /dev/fb0");
		return 1;
	}
	if (variable.bits_per_pixel != 16 || fixed.line_length == 0 ||
	    variable.yres > SIZE_MAX / fixed.line_length) {
		fprintf(stderr, "fbstream: expected a complete 16-bit framebuffer\n");
		return 1;
	}
	visible_bytes = (size_t)fixed.line_length * variable.yres;
	if (fixed.smem_len < visible_bytes) {
		fprintf(stderr, "fbstream: incomplete framebuffer resource\n");
		return 1;
	}

	framebuffer = mmap(NULL, fixed.smem_len, PROT_READ | PROT_WRITE,
			   MAP_SHARED, framebuffer_fd, 0);
	if (framebuffer == MAP_FAILED) {
		perror("fbstream: mmap");
		return 1;
	}
	if (fixed.smem_len >= visible_bytes + DISPLAY_ACCEL_STAGE_SIZE +
	    DISPLAY_ACCEL_CONTROL_SIZE) {
		volatile uint32_t *status = (volatile uint32_t *)(framebuffer +
			visible_bytes + DISPLAY_ACCEL_STAGE_SIZE);

		framebuffer_commit = status + 3;
		if (source_width <= DISPLAY_ACCEL_MAX_WIDTH &&
		    source_height <= DISPLAY_ACCEL_MAX_HEIGHT &&
		    source_bytes <= DISPLAY_ACCEL_STAGE_SIZE &&
		    *status == DISPLAY_ACCEL_STATUS_MAGIC) {
			accelerator_staging = framebuffer + visible_bytes;
			status[1] = DISPLAY_ACCEL_PACK_SIZE(source_width,
				source_height);
			accelerator_command = status + 2;
			accelerated = true;
		}
	}

	if (!accelerated) {
		if ((uint64_t)variable.xres * source_height <=
		    (uint64_t)variable.yres * source_width) {
			output_width = variable.xres;
			output_height = (unsigned int)((uint64_t)source_height *
						       output_width / source_width);
		} else {
			output_height = variable.yres;
			output_width = (unsigned int)((uint64_t)source_width *
						      output_height / source_height);
		}
		if (output_width == 0 || output_height == 0) {
			fprintf(stderr, "fbstream: source does not fit framebuffer\n");
			return 1;
		}
		output_left = (variable.xres - output_width) / 2u;
		output_top = (variable.yres - output_height) / 2u;

		source = malloc(source_bytes);
		scaled_row = malloc((size_t)output_width * sizeof(*scaled_row));
		column_map = malloc((size_t)output_width * sizeof(*column_map));
		if (source == NULL || scaled_row == NULL || column_map == NULL) {
			fprintf(stderr, "fbstream: insufficient memory\n");
			return 1;
		}
		for (unsigned int x = 0; x < output_width; x++)
			column_map[x] = (unsigned int)((uint64_t)x * source_width /
						     output_width);
	}

	console_fd = open("/dev/tty0", O_RDWR | O_NOCTTY | O_CLOEXEC);
	if (console_fd >= 0) {
		(void)write(console_fd, "\033[?25l", sizeof("\033[?25l") - 1u);
		(void)ioctl(console_fd, KDSETMODE, KD_GRAPHICS);
		(void)atexit(restore_console);
	}
	sigemptyset(&signal_action.sa_mask);
	(void)sigaction(SIGINT, &signal_action, NULL);
	(void)sigaction(SIGTERM, &signal_action, NULL);
	(void)sigaction(SIGHUP, &signal_action, NULL);

	if (accelerated) {
		fprintf(stderr,
			"fbstream: %ux%u RGB565 -> generic PPA blit, "
			"Ctrl-C stops\n", source_width, source_height);
	} else {
		memset(framebuffer, 0, visible_bytes);
		if (framebuffer_commit != NULL)
			*framebuffer_commit = DISPLAY_FB_COMMIT_SYNC;
		fprintf(stderr,
			"fbstream: %ux%u RGB565 -> %ux%u at (%u,%u), "
			"Ctrl-C stops\n", source_width, source_height,
			output_width, output_height, output_left, output_top);
	}

	while (!stop_requested) {
		int result = read_exact(STDIN_FILENO,
			accelerated ? accelerator_staging : (uint8_t *)source,
			source_bytes);

		if (result <= 0)
			break;
		if (accelerated) {
			*accelerator_command = DISPLAY_ACCEL_COMMAND_BLIT;
			frame_count++;
			continue;
		}
		for (unsigned int y = 0; y < output_height; y++) {
			unsigned int source_y = (unsigned int)((uint64_t)y *
							 source_height /
							 output_height);
			const uint16_t *source_row =
				source + (size_t)source_y * source_width;
			uint8_t *destination = framebuffer +
				(size_t)(output_top + y) * fixed.line_length +
				(size_t)output_left * sizeof(uint16_t);

			for (unsigned int x = 0; x < output_width; x++)
				scaled_row[x] = source_row[column_map[x]];
			memcpy(destination, scaled_row,
			       (size_t)output_width * sizeof(*scaled_row));
		}
		if (framebuffer_commit != NULL)
			*framebuffer_commit = DISPLAY_FB_COMMIT_SYNC;
		frame_count++;
	}
	if (accelerated)
		*accelerator_command = DISPLAY_ACCEL_COMMAND_STOP;

	fprintf(stderr, "fbstream: received %llu complete frames\n",
		(unsigned long long)frame_count);
	free(column_map);
	free(scaled_row);
	free(source);
	munmap(framebuffer, fixed.smem_len);
	close(framebuffer_fd);
	return 0;
}
