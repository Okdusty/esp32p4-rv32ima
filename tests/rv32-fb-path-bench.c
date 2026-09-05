// SPDX-License-Identifier: MIT
/* Measure guest RAM, fbdev write(), and mmap() RGB565 store paths. */

#include <errno.h>
#include <fcntl.h>
#include <linux/fb.h>
#include <stdint.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <sys/ioctl.h>
#include <sys/mman.h>
#include <sys/stat.h>
#include <time.h>
#include <unistd.h>

static uint64_t now_ns(void)
{
	struct timespec now;

	if (clock_gettime(CLOCK_MONOTONIC, &now) != 0) {
		perror("clock_gettime");
		exit(1);
	}
	return (uint64_t)now.tv_sec * 1000000000ull + now.tv_nsec;
}

static void report(const char *name, uint64_t elapsed_ns, size_t bytes)
{
	double seconds = elapsed_ns / 1000000000.0;
	double mib_s = seconds > 0.0 ? bytes / (1024.0 * 1024.0) / seconds : 0.0;

	printf("%-24s %8.3f ms  %8.3f MiB/s\n", name,
	       elapsed_ns / 1000000.0, mib_s);
}

static __attribute__((noinline)) void convert_rgb565_2x(
	uint16_t *destination, const uint8_t *source, const uint16_t *palette)
{
	for (size_t y = 0; y < 200; ++y) {
		for (size_t repeat = 0; repeat < 2; ++repeat) {
			uint16_t *row = destination + (y * 2 + repeat) * 640;

			for (size_t x = 0; x < 320; ++x) {
				uint16_t pixel = palette[source[y * 320 + x]];

				row[x * 2] = pixel;
				row[x * 2 + 1] = pixel;
			}
		}
	}
}

static __attribute__((noinline)) void present_centered_rows(
	volatile uint16_t *framebuffer, uint32_t stride_bytes,
	uint32_t width, uint32_t height, const uint16_t *source)
{
	uint8_t *base = (uint8_t *)(uintptr_t)framebuffer;
	size_t x_offset = (width - 640u) * sizeof(uint16_t) / 2u;
	size_t y_offset = (size_t)(height - 400u) * stride_bytes / 2u;

	for (size_t y = 0; y < 400; ++y)
		memcpy(base + y_offset + y * stride_bytes + x_offset,
		       source + y * 640, 640 * sizeof(uint16_t));
}

int main(void)
{
	struct fb_var_screeninfo var;
	struct fb_fix_screeninfo fix;
	volatile uint16_t *pixels;
	uint16_t *source;
	uint16_t *ram;
	uint16_t *converted;
	uint16_t palette[256];
	uint8_t *indexed;
	uint64_t started;
	size_t size;
	ssize_t written;
	ssize_t row_written;
	size_t rows_written;
	int fd;

	fd = open("/dev/fb0", O_RDWR);
	if (fd < 0) {
		perror("open /dev/fb0");
		return 1;
	}
	if (ioctl(fd, FBIOGET_VSCREENINFO, &var) != 0 ||
	    ioctl(fd, FBIOGET_FSCREENINFO, &fix) != 0) {
		perror("framebuffer info");
		return 1;
	}
	size = (size_t)fix.line_length * var.yres;
	printf("fb=%ux%u bpp=%u stride=%u bytes=%zu\n", var.xres, var.yres,
	       var.bits_per_pixel, fix.line_length, size);

	source = malloc(size);
	ram = malloc(size);
	converted = malloc(640u * 400u * sizeof(*converted));
	indexed = malloc(320u * 200u);
	if (!source || !ram || !converted || !indexed) {
		perror("malloc");
		return 1;
	}
	for (size_t i = 0; i < 256; ++i)
		palette[i] = (uint16_t)(((i & 0xf8u) << 8) |
		                        ((i & 0xfcu) << 3) | (i >> 3));
	for (size_t i = 0; i < 320u * 200u; ++i)
		indexed[i] = (uint8_t)i;
	for (size_t i = 0; i < size / sizeof(*source); ++i) {
		unsigned int x = i % var.xres;

		source[i] = x < var.xres / 3 ? 0xf800u :
		            x < 2 * var.xres / 3 ? 0x07e0u : 0x001fu;
	}

	started = now_ns();
	memcpy(ram, source, size);
	report("RAM memcpy", now_ns() - started, size);
	if (ram[size / sizeof(*ram) - 1] != source[size / sizeof(*source) - 1])
		return 2;

	pixels = mmap(NULL, size, PROT_READ | PROT_WRITE, MAP_SHARED, fd, 0);
	if (pixels == MAP_FAILED) {
		perror("mmap /dev/fb0");
		return 1;
	}
	started = now_ns();
	for (size_t i = 0; i < size / sizeof(*pixels); ++i)
		pixels[i] = source[i];
	report("mmap volatile u16 loop", now_ns() - started, size);

	started = now_ns();
	memcpy((void *)pixels, source, size);
	report("mmap memcpy", now_ns() - started, size);

	/* Warm the exact loops once so compilation and first-touch faults are not
	 * charged to the steady-state rendering measurements. */
	convert_rgb565_2x(converted, indexed, palette);
	present_centered_rows(pixels, fix.line_length, var.xres, var.yres,
	                      converted);
	started = now_ns();
	for (unsigned int iteration = 0; iteration < 5; ++iteration)
		convert_rgb565_2x(converted, indexed, palette);
	report("2x palette conversion", now_ns() - started,
	       5u * 640u * 400u * sizeof(*converted));
	started = now_ns();
	for (unsigned int iteration = 0; iteration < 5; ++iteration)
		present_centered_rows(pixels, fix.line_length, var.xres, var.yres,
		                      converted);
	report("centered mmap rows", now_ns() - started,
	       5u * 640u * 400u * sizeof(*converted));

	if (lseek(fd, 0, SEEK_SET) < 0) {
		perror("lseek");
		return 1;
	}
	started = now_ns();
	written = write(fd, source, size);
	report("single fbdev write", now_ns() - started,
	       written > 0 ? (size_t)written : 0);
	if (written < 0)
		fprintf(stderr, "write: %s\n", strerror(errno));
	else if ((size_t)written != size)
		fprintf(stderr, "short write: %zd/%zu\n", written, size);

	/* Exercise the common scanline writer shape separately.  It moves the
	 * same bytes as the single write above, but exposes whether the fbdev
	 * driver merges adjacent calls instead of synchronizing every row. */
	if (lseek(fd, 0, SEEK_SET) < 0) {
		perror("lseek rows");
		return 1;
	}
	rows_written = 0;
	row_written = 0;
	started = now_ns();
	for (unsigned int row = 0; row < var.yres; ++row) {
		row_written = write(fd,
			(const uint8_t *)source + (size_t)row * fix.line_length,
			fix.line_length);
		if (row_written != (ssize_t)fix.line_length)
			break;
		rows_written += (size_t)row_written;
	}
	report("row-sized fbdev writes", now_ns() - started, rows_written);
	if (rows_written != size)
		fprintf(stderr, "short row writes: %zu/%zu (%s)\n",
			rows_written, size,
			row_written < 0 ? strerror(errno) : "short write");

	munmap((void *)pixels, size);
	free(indexed);
	free(converted);
	free(ram);
	free(source);
	close(fd);
	return written == (ssize_t)size && rows_written == size ? 0 : 1;
}
