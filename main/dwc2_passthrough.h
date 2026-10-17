#ifndef DWC2_PASSTHROUGH_H
#define DWC2_PASSTHROUGH_H

#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>

#define DWC2_GUEST_BASE 0x50000000u
#define DWC2_GUEST_SIZE 0x00040000u

int dwc2_passthrough_init(void);
void dwc2_passthrough_service(void);
bool dwc2_passthrough_contains(uint32_t address, size_t width);
uint32_t dwc2_passthrough_load(uint32_t address, size_t width);
void dwc2_passthrough_store(uint32_t address, uint32_t value, size_t width);
bool dwc2_passthrough_irq_pending(void);
void dwc2_passthrough_irq_complete(void);

/* Monotonic interrupt counters and controller status for the stall report. */
struct dwc2_passthrough_stats {
	uint32_t physical;  /* host ISR entries, SOF included */
	uint32_t forwarded; /* interrupts raised to the guest */
	uint32_t completed; /* guest PLIC completions */
	bool pending;
	uint32_t gintsts;
	uint32_t gintmsk;
	uint32_t hprt;
};
void dwc2_passthrough_stats(struct dwc2_passthrough_stats *stats);
/* Transfer outcomes seen on the guest's HCINT reads, plus the last IN data. */
struct dwc2_passthrough_channel_stats {
	uint32_t completions;
	uint32_t in_completions;
	uint32_t in_bytes;
	uint32_t invalidations;
	uint32_t naks;
	uint32_t stalls;
	uint32_t errors;
	uint32_t toggle_errors;
	uint32_t frame_overruns;
	uint32_t last_in_ep;
	uint32_t last_in_len;
	uint8_t last_in_data[8];
};
void dwc2_passthrough_channel_stats(struct dwc2_passthrough_channel_stats *stats);
/* Print the global and per-channel host registers through ROM output. */
void dwc2_passthrough_dump(void);

#endif /* DWC2_PASSTHROUGH_H */
