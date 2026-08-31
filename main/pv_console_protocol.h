/*
 * Bulk guest-to-host console transport.
 *
 * The guest publishes one userspace buffer address and rings the length
 * doorbell.  The emulator translates and copies the buffer synchronously into
 * its CPU1 UART queue, so the guest never has to transmit it byte-by-byte
 * through the emulated 16550.
 *
 * SPDX-License-Identifier: BSD-3-Clause
 */

#ifndef RV32_PV_CONSOLE_PROTOCOL_H
#define RV32_PV_CONSOLE_PROTOCOL_H

#define RV32_PV_CONSOLE_BASE             0x10001000u
#define RV32_PV_CONSOLE_APERTURE_SIZE    0x00001000u

#define RV32_PV_CONSOLE_POINTER_OFFSET   0x00u
#define RV32_PV_CONSOLE_LENGTH_OFFSET    0x04u
#define RV32_PV_CONSOLE_MAGIC_OFFSET     0x08u
#define RV32_PV_CONSOLE_MAX_LENGTH_OFFSET 0x0cu

#define RV32_PV_CONSOLE_MAGIC            0x52564331u /* "RVC1" */
#define RV32_PV_CONSOLE_MAX_LENGTH       4096u

#endif
