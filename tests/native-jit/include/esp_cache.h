#ifndef ESP_CACHE_H
#define ESP_CACHE_H
#include <stddef.h>
typedef int esp_err_t;
#define ESP_OK 0
#define ESP_CACHE_MSYNC_FLAG_DIR_C2M 1
#define ESP_CACHE_MSYNC_FLAG_DIR_M2C 2
#define ESP_CACHE_MSYNC_FLAG_TYPE_DATA 4
#define ESP_CACHE_MSYNC_FLAG_TYPE_INST 8
#define ESP_CACHE_MSYNC_FLAG_INVALIDATE 16
static inline esp_err_t esp_cache_msync(void *address, size_t size,
					unsigned int flags)
{
	(void)address;
	(void)size;
	(void)flags;
	return ESP_OK;
}
#endif
