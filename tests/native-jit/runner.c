#include <stddef.h>

/* Freestanding support for the GNU RV32 instruction-set simulator. */
void *memset(void *destination, int value, size_t length)
{
	unsigned char *out = destination;
	while (length--)
		*out++ = (unsigned char)value;
	return destination;
}

void *memcpy(void *destination, const void *source, size_t length)
{
	unsigned char *out = destination;
	const unsigned char *in = source;
	while (length--)
		*out++ = *in++;
	return destination;
}

void *memmove(void *destination, const void *source, size_t length)
{
	unsigned char *out = destination;
	const unsigned char *in = source;
	if (out < in)
		return memcpy(destination, source, length);
	while (length) {
		length--;
		out[length] = in[length];
	}
	return destination;
}

__asm__(".section .text.startup,\"ax\"\n"
	".global _start\n"
	"_start:\n"
	"lui sp, 0xf00\n"
	"call test_main\n"
	"li a7, 93\n"
	"ecall\n"
	"1: j 1b\n"
	".previous\n");
