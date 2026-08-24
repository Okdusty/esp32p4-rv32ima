#include <stdio.h>
#include <string.h>
#include <stdint.h>
#include <stdlib.h>
#include <termios.h>
#include <unistd.h>
#include <signal.h>
#include <fcntl.h>
#include <sys/time.h>
#include <sys/ioctl.h>

#include "port.h"
#include "psram.h"

extern struct MiniRV32IMAState core;
extern void DumpState(struct MiniRV32IMAState *core);
extern void app_main(void);
extern char kernel_start[], kernel_end[];

static int ramfd;
static int is_eofd;

static void ResetKeyboardInput(void)
{
	// Re-enable echo, etc. on keyboard.
	struct termios term;
	tcgetattr(0, &term);
	term.c_lflag |= ICANON | ECHO;
	tcsetattr(0, TCSANOW, &term);
}

static void CtrlC(int sig)
{
	DumpState(&core);
	ResetKeyboardInput();
	exit(0);
}

static void CaptureKeyboardInput(void)
{
    struct termios term;

    signal(SIGINT, CtrlC);

    tcgetattr(STDIN_FILENO, &term);

    term.c_lflag &= ~(ICANON | ECHO);
    term.c_cc[VMIN] = 0;
    term.c_cc[VTIME] = 0;

    tcsetattr(STDIN_FILENO, TCSANOW, &term);
}

uint64_t GetTimeMicroseconds()
{
	struct timeval tv;
	gettimeofday(&tv, 0);
	return tv.tv_usec + ((uint64_t)(tv.tv_sec)) * 1000000LL;
}

int ReadKBByte(void)
{
    unsigned char rxchar;

    if (read(STDIN_FILENO, &rxchar, 1) == 1)
        return rxchar;

    return -1;
}

int HostInputInit(void)
{
	return 0;
}

int HostConsoleInit(void)
{
	return 0;
}

int HostConsoleWrite(const void *buffer, size_t length)
{
	return write(STDOUT_FILENO, buffer, length);
}

int HostDmaCacheSync(uint32_t guest_physical_address, size_t length,
		     enum host_dma_sync_op operation)
{
	(void)guest_physical_address;
	(void)length;
	(void)operation;
	return 0;
}

int IsKBHit(void)
{
    int byteswaiting = 0;

    if (ioctl(STDIN_FILENO, FIONREAD, &byteswaiting) < 0)
        return 0;

    return byteswaiting > 0;
}

int psram_init(void)
{
	ramfd = open("/tmp/ram", O_RDWR | O_CREAT, S_IRUSR | S_IWUSR);
	if (ramfd < 0) {
		perror("open\n");
		return -1;
	}

	return 0;
}

int psram_read(uint32_t addr, void *buf, int len)
{
	lseek(ramfd, addr, SEEK_SET);
	read(ramfd, buf, len);
}

int psram_write(uint32_t addr, void *buf, int len)
{
	lseek(ramfd, addr, SEEK_SET);
	write(ramfd, buf, len);
}

int load_images(int ram_size, int *kern_len)
{
	long flen;

	flen = kernel_end - kernel_start;
	if (flen > ram_size) {
		fprintf(stderr, "Error: Could not fit RAM image (%ld bytes) into %d\n", flen, ram_size);
		return -1;
	}
	if (kern_len)
		*kern_len = flen;

	lseek(ramfd, KERNEL_LOAD_OFFSET, SEEK_SET);
	write(ramfd, kernel_start, flen);

	return 0;
}

int main(int argc, char **argv)
{
	CaptureKeyboardInput();
	app_main();
}
