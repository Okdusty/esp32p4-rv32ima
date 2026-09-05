#ifndef ESP_HEAP_CAPS_H
#define ESP_HEAP_CAPS_H
#include <stddef.h>
#include <string.h>
#define MALLOC_CAP_EXEC 1
#define MALLOC_CAP_INTERNAL 2
#define MALLOC_CAP_32BIT 4
static inline void *heap_caps_aligned_calloc(size_t alignment,
					      size_t count, size_t size,
					      unsigned int caps)
{
	static _Alignas(128) unsigned char memory[64 * 1024];
	size_t bytes = count * size;

	(void)caps;
	if (alignment > 128 || bytes > sizeof(memory))
		return NULL;
	memset(memory, 0, bytes);
	return memory;
}
#endif
