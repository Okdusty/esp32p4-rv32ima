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
static bool terminal_owns_framebuffer;
static bool ppa_fill_touched_framebuffer;
static unsigned marked_rows, flush_calls, parsed_bytes, receive_calls;
static void display_terminal_mark_rows(unsigned first, unsigned last) { marked_rows += last - first + 1u; }
static void display_terminal_hide_cursor(void) {}
static size_t xStreamBufferReceive(void *stream, void *buffer, size_t count, unsigned ticks) {
 (void)stream; (void)buffer; (void)count; (void)ticks; receive_calls++; return 0u;
}
static size_t xStreamBufferBytesAvailable(void *stream) { (void)stream; return 0u; }
static void display_terminal_parse_byte(uint8_t byte) { (void)byte; parsed_bytes++; }
static bool display_terminal_flush_dirty(bool cursor) { (void)cursor; flush_calls++; return true; }
static bool display_flush_accel_dirty(void) { return true; }
/* Function below is copied verbatim from current production source.
 * The queue/render helpers above are controlled host-test doubles. */
static bool display_terminal_process_stream(void) {
  uint8_t buffer[DISPLAY_TERMINAL_READ_CHUNK];
  size_t processed = 0;

  if (terminal_stream == NULL || terminal.cells == NULL)
    return false;
  if (!terminal.active || !terminal_owns_framebuffer) {
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
  return xStreamBufferBytesAvailable(terminal_stream) != 0u;
}

int main(void) {
 bool pending = display_terminal_process_stream();
 printf("empty_queue=1 reclaimed=%u marked_rows=%u flush_calls=%u parsed_bytes=%u receives=%u pending=%u\n",
  (unsigned)terminal_owns_framebuffer, marked_rows, flush_calls, parsed_bytes, receive_calls, (unsigned)pending);
 /* Success means reproducing the unwanted redraw request, not validating a fix. */
 return !(terminal_owns_framebuffer && marked_rows == DISPLAY_TERMINAL_ROWS &&
          flush_calls == 1u && parsed_bytes == 0u && !pending);
}
