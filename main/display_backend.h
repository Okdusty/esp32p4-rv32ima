#ifndef DISPLAY_BACKEND_H
#define DISPLAY_BACKEND_H

#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>

#include "esp_err.h"

typedef bool (*display_backend_vsync_callback_t)(void *context);

#define DISPLAY_BACKEND_FORMAT_RGB565 1u

typedef struct {
  uint8_t *framebuffer;
  size_t framebuffer_size;
  uint32_t width;
  uint32_t height;
  uint32_t stride;
  uint32_t format;
  uint32_t refresh_hz;
  const char *name;
} display_backend_surface_t;

/* Initialize a packed RGB565 scanout but leave it disabled until start(). */
esp_err_t display_backend_init(display_backend_vsync_callback_t vsync_callback,
                               void *vsync_context,
                               display_backend_surface_t *surface);
esp_err_t display_backend_start(void);

#endif /* DISPLAY_BACKEND_H */
