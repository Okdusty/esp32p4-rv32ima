#include <stdbool.h>
#include <stdint.h>
#include <stddef.h>
#include <stdio.h>
#define DISPLAY_TERMINAL_READ_CHUNK 64u
#define DISPLAY_TERMINAL_SERVICE_BYTES 128u
#define DISPLAY_TERMINAL_ROWS 45u
static int storage;
static void *terminal_stream = &storage;
static struct { void *cells; bool active; bool pixels_valid; } terminal = { &storage, true, true };
static bool terminal_owns_framebuffer, ppa_fill_touched_framebuffer;
static bool tile_cursor_active, tile_cursor_drawn;
static unsigned marked_rows, flush_calls, parsed_bytes;
static size_t queued;
static int64_t esp_timer_get_time(void) { return 123000; }
static void display_terminal_mark_rows(unsigned first, unsigned last) { marked_rows += last - first + 1u; }
static void display_terminal_hide_cursor(void) {}
static size_t xStreamBufferReceive(void *stream, void *buffer, size_t count, unsigned ticks) {
 (void)stream; (void)ticks;
 size_t length = queued < count ? queued : count;
 for (size_t i = 0; i < length; i++) ((uint8_t *)buffer)[i] = (uint8_t)('A' + i);
 queued -= length;
 return length;
}
static size_t xStreamBufferBytesAvailable(void *stream) { (void)stream; return queued; }
static void display_terminal_parse_byte(uint8_t byte) { (void)byte; parsed_bytes++; }
static bool display_terminal_flush_dirty(bool cursor) { (void)cursor; flush_calls++; return true; }
static bool display_flush_accel_dirty(void) { return true; }
/* The following two functions are copied verbatim from the diagnostic source.
 * Queue/render/time helpers are controlled doubles, not P4 cache/DMA models. */
static void display_terminal_release_framebuffer(void) {
  bool released_terminal = terminal_owns_framebuffer;

  /* A raw writer may already have replaced the pixels below the XOR cursor.
   * Do not XOR them a second time: forget the overlay and let the raw frame
   * replace it.  The next terminal byte forces a complete clean redraw. */
  tile_cursor_active = false;
  tile_cursor_drawn = false;
  terminal.pixels_valid = false;
  terminal_owns_framebuffer = false;
  if (released_terminal)
    printf("Display diagnostic: terminal release at %u ms\n",
           (unsigned int)(esp_timer_get_time() / 1000));
}

static bool display_terminal_process_stream(void) {
  uint8_t buffer[DISPLAY_TERMINAL_READ_CHUNK];
  size_t processed = 0;
  bool claimed_terminal = false;

  if (terminal_stream == NULL || terminal.cells == NULL)
    return false;
  if (!terminal.active || !terminal_owns_framebuffer) {
    claimed_terminal = true;
    terminal.active = true;
    terminal_owns_framebuffer = true;
    terminal.pixels_valid = false;
    display_terminal_mark_rows(0, DISPLAY_TERMINAL_ROWS - 1u);
  }
  display_terminal_hide_cursor();
  while (processed < DISPLAY_TERMINAL_SERVICE_BYTES) {
    size_t limit = DISPLAY_TERMINAL_SERVICE_BYTES - processed;
    size_t length;

    if (limit > sizeof(buffer))
      limit = sizeof(buffer);
    length = xStreamBufferReceive(terminal_stream, buffer, limit, 0);
    if (length == 0)
      break;
    for (size_t index = 0; index < length; index++)
      display_terminal_parse_byte(buffer[index]);
    processed += length;
  }
  (void)display_terminal_flush_dirty(true);
  if (ppa_fill_touched_framebuffer && display_flush_accel_dirty())
    ppa_fill_touched_framebuffer = false;
  /* Event-only instrumentation preserves the existing ownership decisions.
   * A zero-byte takeover distinguishes the reproduced empty-wake hazard from
   * genuine terminal output; neither event alone diagnoses an MCU reset. */
  if (claimed_terminal)
    printf("Display diagnostic: terminal takeover at %u ms, input=%u bytes\n",
           (unsigned int)(esp_timer_get_time() / 1000),
           (unsigned int)processed);
  return xStreamBufferBytesAvailable(terminal_stream) != 0u;
}

int main(void) {
 /* This intentionally preserves the reproduced zero-byte takeover hazard. */
 if (display_terminal_process_stream() || !terminal_owns_framebuffer ||
     marked_rows != DISPLAY_TERMINAL_ROWS || flush_calls != 1u || parsed_bytes != 0u)
   return 1;
 tile_cursor_active = tile_cursor_drawn = true;
 display_terminal_release_framebuffer();
 if (terminal_owns_framebuffer || tile_cursor_active || tile_cursor_drawn || terminal.pixels_valid)
   return 2;
 /* Repeated release must not emit another ownership-change event. */
 display_terminal_release_framebuffer();
 marked_rows = flush_calls = parsed_bytes = 0u;
 queued = 4u;
 if (display_terminal_process_stream() || !terminal_owns_framebuffer ||
     marked_rows != DISPLAY_TERMINAL_ROWS || flush_calls != 1u || parsed_bytes != 4u)
   return 3;
 /* Empty wake when already owned must not emit a new takeover event. */
 if (display_terminal_process_stream() || !terminal_owns_framebuffer ||
     marked_rows != DISPLAY_TERMINAL_ROWS || flush_calls != 2u || parsed_bytes != 4u)
   return 4;
 terminal_stream = NULL;
 if (display_terminal_process_stream() || flush_calls != 2u)
   return 5;
 puts("diagnostic ownership/state assertions passed; zero-byte hazard remains");
 return 0;
}
