#ifndef DISPLAY_ACCEL_PROTOCOL_H
#define DISPLAY_ACCEL_PROTOCOL_H

/*
 * Generic RGB565 staging/blit aperture appended to the Linux framebuffer.
 * Source pixels are tightly packed.  A BLIT scales them proportionally to
 * the 720x720 scanout and centers the result using the ESP32-P4 PPA.
 * The four control words following the staging buffer are status, source
 * size, accelerator command, and framebuffer commit/fence.  After completing
 * a direct framebuffer update, read the final word, write
 * DISPLAY_FB_COMMIT_SYNC to it, and wait for its value to change.  The returned
 * sequence advances only after a physical VSYNC cache-cleaned the update.
 */
#define DISPLAY_ACCEL_MAX_WIDTH          256u
#define DISPLAY_ACCEL_MAX_HEIGHT         256u
#define DISPLAY_ACCEL_BYTES_PER_PIXEL    2u
#define DISPLAY_ACCEL_STAGE_SIZE         \
	(DISPLAY_ACCEL_MAX_WIDTH * DISPLAY_ACCEL_MAX_HEIGHT * \
	 DISPLAY_ACCEL_BYTES_PER_PIXEL)
#define DISPLAY_ACCEL_CONTROL_SIZE       16u
#define DISPLAY_ACCEL_STATUS_MAGIC       0x31415050u /* "PPA1" */
#define DISPLAY_ACCEL_COMMAND_BLIT       0x54494c42u /* "BLIT" */
#define DISPLAY_ACCEL_COMMAND_STOP       0x504f5453u /* "STOP" */
#define DISPLAY_FB_COMMIT_SYNC           0x434e5953u /* "SYNC" */

/*
 * Optional generic fbdev acceleration mailbox.  The Linux driver still
 * exposes an ordinary RGB565 /dev/fb0 and falls back to software drawing when
 * this block is absent. Version 3 decouples command descriptors from reusable
 * payload blocks.  This keeps control commands flowing when all bitmap blocks
 * are busy and lets guests discover the maximum payload and surface geometry.
 * ACCEPTED acknowledges ownership transfer immediately; COMPLETED advances
 * after the display worker executes an entry.  A guest must wait for COMPLETED
 * before declaring a frame synchronized. Version 4 also accepts a command
 * descriptor and payload directly from guest RAM. The doorbell copies both
 * while still running on CPU0, avoiding repeated emulated MMIO stores.
 */
#define DISPLAY_PV_MAGIC                 0x31424650u /* "PFB1" */
#define DISPLAY_PV_VERSION               4u
#define DISPLAY_PV_SUBMIT                0x54494d53u /* "SMIT" */
#define DISPLAY_PV_SHARED_COMMAND_MAGIC  0x34444d43u /* "CMD4" */

#define DISPLAY_PV_FEATURE_FILL          (1u << 0)
#define DISPLAY_PV_FEATURE_COPY          (1u << 1)
#define DISPLAY_PV_FEATURE_IMAGE1        (1u << 2)
#define DISPLAY_PV_FEATURE_VSYNC_FENCE   (1u << 3)
#define DISPLAY_PV_FEATURE_ASYNC_FIFO    (1u << 4)
#define DISPLAY_PV_FEATURE_TILE          (1u << 5)
#define DISPLAY_PV_FEATURE_PAYLOAD_POOL  (1u << 6)
#define DISPLAY_PV_FEATURE_SURFACE_INFO  (1u << 7)
#define DISPLAY_PV_FEATURE_SHARED_COMMAND (1u << 8)

#define DISPLAY_PV_FORMAT_RGB565         1u

#define DISPLAY_PV_OP_FILL               1u
#define DISPLAY_PV_OP_COPY               2u
#define DISPLAY_PV_OP_IMAGE1             3u
#define DISPLAY_PV_OP_SET_TILE           4u
#define DISPLAY_PV_OP_TILE_FILL          5u
#define DISPLAY_PV_OP_TILE_BLIT          6u
#define DISPLAY_PV_OP_TILE_CURSOR        7u

#define DISPLAY_PV_ROP_COPY              0u
#define DISPLAY_PV_ROP_XOR               1u

#define DISPLAY_PV_STATUS_OK             0u
#define DISPLAY_PV_STATUS_INVALID        1u
#define DISPLAY_PV_STATUS_BUSY           2u
#define DISPLAY_PV_STATUS_FAILED         3u

/* Register offsets relative to DISPLAY_PV_REG_GUEST_BASE. */
#define DISPLAY_PV_REG_MAGIC             0x00u
#define DISPLAY_PV_REG_VERSION           0x04u
#define DISPLAY_PV_REG_FEATURES          0x08u
#define DISPLAY_PV_REG_COMPLETED         0x0cu
#define DISPLAY_PV_REG_STATUS            0x10u
#define DISPLAY_PV_REG_SEQUENCE          0x14u
#define DISPLAY_PV_REG_OPERATION         0x18u
#define DISPLAY_PV_REG_ARG0              0x1cu
#define DISPLAY_PV_REG_ARG1              0x20u
#define DISPLAY_PV_REG_ARG2              0x24u
#define DISPLAY_PV_REG_ARG3              0x28u
#define DISPLAY_PV_REG_ARG4              0x2cu
#define DISPLAY_PV_REG_ARG5              0x30u
#define DISPLAY_PV_REG_ARG6              0x34u
#define DISPLAY_PV_REG_ARG7              0x38u
#define DISPLAY_PV_REG_DOORBELL          0x3cu
#define DISPLAY_PV_REG_ACCEPTED          0x40u
#define DISPLAY_PV_REG_ACCEPT_STATUS     0x44u
#define DISPLAY_PV_REG_QUEUE_DEPTH       0x48u
#define DISPLAY_PV_REG_QUEUE_FREE        0x4cu
#define DISPLAY_PV_REG_PAYLOAD_LIMIT     0x50u
#define DISPLAY_PV_REG_PAYLOAD_BLOCKS    0x54u
#define DISPLAY_PV_REG_PAYLOAD_FREE      0x58u
#define DISPLAY_PV_REG_SURFACE_WIDTH     0x5cu
#define DISPLAY_PV_REG_SURFACE_HEIGHT    0x60u
#define DISPLAY_PV_REG_SURFACE_STRIDE    0x64u
#define DISPLAY_PV_REG_SURFACE_FORMAT    0x68u
#define DISPLAY_PV_REG_FIFO_ERRORS       0x6cu
#define DISPLAY_PV_REG_SHARED_COMMAND    0x70u
#define DISPLAY_PV_REG_SIZE              0x74u

typedef struct {
	uint32_t magic;
	uint32_t sequence;
	uint32_t operation;
	uint32_t args[8];
	uint32_t payload_address;
	uint32_t payload_length;
} display_pv_shared_command_t;

#define DISPLAY_ACCEL_PACK_SIZE(width, height) \
	(((width) & 0xffffu) | (((height) & 0xffffu) << 16))

#endif /* DISPLAY_ACCEL_PROTOCOL_H */
