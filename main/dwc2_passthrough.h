// DEAD CODE
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

#endif /* DWC2_PASSTHROUGH_H */
