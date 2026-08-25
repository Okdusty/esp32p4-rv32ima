// SPDX-License-Identifier: GPL-2.0-only
/*
 * Standard fbdev frontend for the optional mini-rv32ima display mailbox.
 *
 * Userspace still gets an ordinary packed-pixel /dev/fb0 with read, write and
 * mmap support.  fbcon's fillrect, copyarea and 1-bpp imageblit hooks are
 * compact MMIO commands when the emulator advertises them, and transparently
 * fall back to the generic cfb helpers otherwise.
 */

#include <linux/aperture.h>
#include <linux/bitops.h>
#include <linux/delay.h>
#include <linux/fb.h>
#include <linux/io.h>
#include <linux/ktime.h>
#include <linux/mm.h>
#include <linux/module.h>
#include <linux/of.h>
#include <linux/overflow.h>
#include <linux/platform_device.h>

#define RV32PV_MAGIC                  0x31424650u /* "PFB1" */
#define RV32PV_VERSION                4u
#define RV32PV_SUBMIT                 0x54494d53u /* "SMIT" */
#define RV32PV_SHARED_COMMAND_MAGIC   0x34444d43u /* "CMD4" */

#define RV32PV_FEATURE_FILL           BIT(0)
#define RV32PV_FEATURE_COPY           BIT(1)
#define RV32PV_FEATURE_IMAGE1         BIT(2)
#define RV32PV_FEATURE_VSYNC_FENCE    BIT(3)
#define RV32PV_FEATURE_ASYNC_FIFO     BIT(4)
#define RV32PV_FEATURE_TILE           BIT(5)
#define RV32PV_FEATURE_PAYLOAD_POOL   BIT(6)
#define RV32PV_FEATURE_SURFACE_INFO   BIT(7)
#define RV32PV_FEATURE_SHARED_COMMAND BIT(8)

#define RV32PV_FORMAT_RGB565          1u

#define RV32PV_OP_FILL                1u
#define RV32PV_OP_COPY                2u
#define RV32PV_OP_IMAGE1              3u
#define RV32PV_OP_SET_TILE            4u
#define RV32PV_OP_TILE_FILL           5u
#define RV32PV_OP_TILE_BLIT           6u
#define RV32PV_OP_TILE_CURSOR         7u

#define RV32PV_ROP_COPY               0u
#define RV32PV_ROP_XOR                1u
#define RV32PV_STATUS_OK              0u
#define RV32PV_STATUS_BUSY            2u
#define RV32PV_FENCE_SYNC             0x434e5953u /* "SYNC" */

#define RV32PV_REG_MAGIC              0x00u
#define RV32PV_REG_VERSION            0x04u
#define RV32PV_REG_FEATURES           0x08u
#define RV32PV_REG_COMPLETED          0x0cu
#define RV32PV_REG_STATUS             0x10u
#define RV32PV_REG_SEQUENCE           0x14u
#define RV32PV_REG_OPERATION          0x18u
#define RV32PV_REG_ARG0               0x1cu
#define RV32PV_REG_DOORBELL           0x3cu
#define RV32PV_REG_ACCEPTED           0x40u
#define RV32PV_REG_ACCEPT_STATUS      0x44u
#define RV32PV_REG_QUEUE_DEPTH        0x48u
#define RV32PV_REG_QUEUE_FREE         0x4cu
#define RV32PV_REG_PAYLOAD_LIMIT      0x50u
#define RV32PV_REG_PAYLOAD_BLOCKS     0x54u
#define RV32PV_REG_PAYLOAD_FREE       0x58u
#define RV32PV_REG_SURFACE_WIDTH      0x5cu
#define RV32PV_REG_SURFACE_HEIGHT     0x60u
#define RV32PV_REG_SURFACE_STRIDE     0x64u
#define RV32PV_REG_SURFACE_FORMAT     0x68u
#define RV32PV_REG_FIFO_ERRORS        0x6cu
#define RV32PV_REG_SHARED_COMMAND     0x70u
#define RV32PV_REG_SIZE               0x74u

#define RV32PV_COMMAND_TIMEOUT_NS      (100ull * NSEC_PER_MSEC)
#define RV32PV_LEGACY_PAYLOAD_LIMIT    2048u

struct rv32pvfb_par {
	u32 palette[16];
	struct resource *mem;
	void __iomem *registers;
	void __iomem *staging;
	void __iomem *fence;
	u32 staging_size;
	u32 payload_limit;
	u32 payload_blocks;
	u32 features;
	u32 queue_depth;
	u32 sequence;
	u32 last_accepted;
	u32 tile_width;
	u32 tile_height;
	unsigned long command_busy;
};

struct rv32pvfb_shared_command {
	u32 magic;
	u32 sequence;
	u32 operation;
	u32 arguments[8];
	u32 payload_address;
	u32 payload_length;
};

static const struct fb_fix_screeninfo rv32pvfb_fix = {
	.id = "rv32-pvfb",
	.type = FB_TYPE_PACKED_PIXELS,
	.visual = FB_VISUAL_TRUECOLOR,
	.accel = FB_ACCEL_NONE,
};

static const struct fb_var_screeninfo rv32pvfb_var = {
	.height = -1,
	.width = -1,
	.activate = FB_ACTIVATE_NOW,
	.vmode = FB_VMODE_NONINTERLACED,
	.bits_per_pixel = 16,
	.red = { .offset = 11, .length = 5 },
	.green = { .offset = 5, .length = 6 },
	.blue = { .offset = 0, .length = 5 },
};

static u32 rv32pvfb_color(struct fb_info *info, u32 color)
{
	struct rv32pvfb_par *par = info->par;

	if (color < ARRAY_SIZE(par->palette))
		return par->palette[color] & 0xffffu;
	return color & 0xffffu;
}

static int rv32pvfb_setcolreg(unsigned int regno, unsigned int red,
			      unsigned int green, unsigned int blue,
			      unsigned int transp, struct fb_info *info)
{
	struct rv32pvfb_par *par = info->par;
	u32 value;

	if (regno >= ARRAY_SIZE(par->palette))
		return -EINVAL;
	value = ((red >> 11) << 11) | ((green >> 10) << 5) | (blue >> 11);
	par->palette[regno] = value;
	return 0;
}

static int rv32pvfb_wait_completed(struct rv32pvfb_par *par, u32 sequence)
{
	u64 deadline;
	u32 completed;
	unsigned int iteration = 0;

	deadline = ktime_get_mono_fast_ns() + RV32PV_COMMAND_TIMEOUT_NS;
	do {
		completed = ioread32(par->registers + RV32PV_REG_COMPLETED);
		if (completed == sequence)
			return ioread32(par->registers + RV32PV_REG_STATUS) ==
			       RV32PV_STATUS_OK ? 0 : -EIO;
		cpu_relax();
		iteration++;
	} while ((iteration & 0xffu) != 0 ||
		 ktime_get_mono_fast_ns() < deadline);
	return -ETIMEDOUT;
}

static bool rv32pvfb_shared_range(const void *pointer, size_t length,
				  u32 *physical_address)
{
	const u8 *last;
	uintptr_t last_address;
	phys_addr_t first_physical;
	phys_addr_t last_physical;

	if (!length) {
		*physical_address = 0;
		return true;
	}
	if (!pointer || check_add_overflow((uintptr_t)pointer, length - 1,
					 &last_address))
		return false;
	last = (const u8 *)last_address;
	if (
	    !virt_addr_valid(pointer) || !virt_addr_valid(last))
		return false;
	first_physical = virt_to_phys((void *)pointer);
	last_physical = virt_to_phys((void *)last);
	if (first_physical > U32_MAX || last_physical > U32_MAX ||
	    last_physical - first_physical != length - 1)
		return false;
	*physical_address = first_physical;
	return true;
}

static bool rv32pvfb_submit(struct fb_info *info, u32 operation,
			    const u32 arguments[8], const void *payload,
			    size_t payload_length)
{
	struct rv32pvfb_par *par = info->par;
	struct rv32pvfb_shared_command shared_command;
	u64 deadline;
	u32 sequence;
	u32 command_address;
	u32 payload_address;
	unsigned int iteration = 0;
	bool use_shared_command;
	bool success = false;

	/*
	 * fbcon serializes normal calls.  A nested emergency-console operation
	 * takes the software path instead of waiting on its interrupted command.
	 */
	if ((payload == NULL) != (payload_length == 0) ||
	    payload_length > par->staging_size ||
	    payload_length > par->payload_limit ||
	    test_and_set_bit(0, &par->command_busy))
		return false;
	sequence = ++par->sequence;
	if (!sequence)
		sequence = ++par->sequence;

	use_shared_command =
		(par->features & RV32PV_FEATURE_SHARED_COMMAND) &&
		rv32pvfb_shared_range(payload, payload_length, &payload_address);
	if (use_shared_command) {
		shared_command.magic = RV32PV_SHARED_COMMAND_MAGIC;
		shared_command.sequence = sequence;
		shared_command.operation = operation;
		memcpy(shared_command.arguments, arguments,
		       sizeof(shared_command.arguments));
		shared_command.payload_address = payload_address;
		shared_command.payload_length = payload_length;
		use_shared_command = rv32pvfb_shared_range(
			&shared_command, sizeof(shared_command), &command_address);
	}
	if (use_shared_command) {
		/* The host copies this stack descriptor and payload synchronously when
		 * it emulates the doorbell store, before this function can return. */
		wmb();
		iowrite32(command_address,
			  par->registers + RV32PV_REG_SHARED_COMMAND);
	} else {
		if (payload)
			memcpy_toio(par->staging, payload, payload_length);
		for (unsigned int index = 0; index < 8; index++)
			iowrite32(arguments[index], par->registers + RV32PV_REG_ARG0 +
				  index * sizeof(u32));
		iowrite32(sequence, par->registers + RV32PV_REG_SEQUENCE);
		iowrite32(operation, par->registers + RV32PV_REG_OPERATION);
	}
	/* Publish every argument and any staging bytes before the doorbell. */
	wmb();
	if (par->features & RV32PV_FEATURE_ASYNC_FIFO) {
		/* The MMIO store snapshots the command and bitmap into a FreeRTOS
		 * queue before returning. Usually this loop executes exactly once;
		 * retrying BUSY briefly avoids an extremely expensive cfb fallback.
		 */
		deadline = ktime_get_mono_fast_ns() + RV32PV_COMMAND_TIMEOUT_NS;
		do {
			u32 accepted;
			u32 status;

			iowrite32(RV32PV_SUBMIT,
				  par->registers + RV32PV_REG_DOORBELL);
			accepted = ioread32(par->registers + RV32PV_REG_ACCEPTED);
			status = ioread32(par->registers +
					  RV32PV_REG_ACCEPT_STATUS);
			if (accepted == sequence && status == RV32PV_STATUS_OK) {
				par->last_accepted = sequence;
				success = true;
				break;
			}
			if (accepted != sequence || status != RV32PV_STATUS_BUSY)
				break;
			cpu_relax();
			iteration++;
		} while ((iteration & 0x3fu) != 0 ||
			 ktime_get_mono_fast_ns() < deadline);
	} else {
		iowrite32(RV32PV_SUBMIT,
			  par->registers + RV32PV_REG_DOORBELL);
		success = rv32pvfb_wait_completed(par, sequence) == 0;
	}
	clear_bit(0, &par->command_busy);
	return success;
}

static void rv32pvfb_fillrect(struct fb_info *info,
			      const struct fb_fillrect *rect)
{
	struct rv32pvfb_par *par = info->par;
	u32 arguments[8] = {
		rect->dx, rect->dy, rect->width, rect->height,
		rv32pvfb_color(info, rect->color),
		rect->rop == ROP_XOR ? RV32PV_ROP_XOR : RV32PV_ROP_COPY,
	};

	if (!(par->features & RV32PV_FEATURE_FILL) ||
	    !rv32pvfb_submit(info, RV32PV_OP_FILL, arguments, NULL, 0))
		cfb_fillrect(info, rect);
}

static void rv32pvfb_copyarea(struct fb_info *info,
			      const struct fb_copyarea *area)
{
	struct rv32pvfb_par *par = info->par;
	u32 arguments[8] = {
		area->sx, area->sy, area->dx, area->dy,
		area->width, area->height,
	};

	if (!(par->features & RV32PV_FEATURE_COPY) ||
	    !rv32pvfb_submit(info, RV32PV_OP_COPY, arguments, NULL, 0))
		cfb_copyarea(info, area);
}

static void rv32pvfb_imageblit(struct fb_info *info,
			       const struct fb_image *image)
{
	struct rv32pvfb_par *par = info->par;
	size_t pitch;
	size_t data_length;
	u32 arguments[8];

	if (!(par->features & RV32PV_FEATURE_IMAGE1) || image->depth != 1)
		goto software;
	pitch = DIV_ROUND_UP(image->width, 8u);
	if (image->height && pitch > SIZE_MAX / image->height)
		goto software;
	data_length = pitch * image->height;
	if (!data_length || data_length > par->staging_size ||
	    data_length > par->payload_limit)
		goto software;

	arguments[0] = image->dx;
	arguments[1] = image->dy;
	arguments[2] = image->width;
	arguments[3] = image->height;
	arguments[4] = rv32pvfb_color(info, image->fg_color);
	arguments[5] = rv32pvfb_color(info, image->bg_color);
	arguments[6] = image->depth;
	arguments[7] = data_length;
	if (rv32pvfb_submit(info, RV32PV_OP_IMAGE1, arguments, image->data,
			  data_length))
		return;

software:
	cfb_imageblit(info, image);
}

static int rv32pvfb_sync(struct fb_info *info)
{
	struct rv32pvfb_par *par = info->par;
	u64 deadline;
	u32 completed;
	unsigned int iteration = 0;
	int ret;

	if ((par->features & RV32PV_FEATURE_ASYNC_FIFO) &&
	    par->last_accepted) {
		if (test_and_set_bit(0, &par->command_busy))
			return -EBUSY;
		ret = rv32pvfb_wait_completed(par, par->last_accepted);
		clear_bit(0, &par->command_busy);
		if (ret)
			return ret;
	}

	if (!(par->features & RV32PV_FEATURE_VSYNC_FENCE) || !par->fence)
		return 0;
	completed = ioread32(par->fence);
	iowrite32(RV32PV_FENCE_SYNC, par->fence);
	deadline = ktime_get_mono_fast_ns() + RV32PV_COMMAND_TIMEOUT_NS;
	do {
		if (ioread32(par->fence) != completed)
			return 0;
		cpu_relax();
		iteration++;
	} while ((iteration & 0xffu) != 0 ||
		 ktime_get_mono_fast_ns() < deadline);
	return -ETIMEDOUT;
}

#ifdef CONFIG_FB_TILEBLITTING
static void rv32pvfb_settile(struct fb_info *info, struct fb_tilemap *map)
{
	struct rv32pvfb_par *par = info->par;
	size_t pitch;
	size_t tile_size;
	size_t data_length;
	u32 arguments[8] = { 0 };

	if (!(par->features & RV32PV_FEATURE_TILE) || map->depth != 1 ||
	    !map->width || !map->height || !map->length)
		return;
	pitch = DIV_ROUND_UP(map->width, 8u);
	if (check_mul_overflow(pitch, (size_t)map->height, &tile_size) ||
	    check_mul_overflow(tile_size, (size_t)map->length, &data_length) ||
	    !data_length || data_length > par->staging_size ||
	    data_length > par->payload_limit)
		return;
	/* A font change is rare. Drain the preceding generation before replacing
	 * its host-side bitmap, then subsequent tile commands can remain async.
	 */
	if (par->tile_width && rv32pvfb_sync(info))
		return;
	arguments[0] = map->width;
	arguments[1] = map->height;
	arguments[2] = map->depth;
	arguments[3] = map->length;
	arguments[4] = data_length;
	if (rv32pvfb_submit(info, RV32PV_OP_SET_TILE, arguments, map->data,
			  data_length)) {
		par->tile_width = map->width;
		par->tile_height = map->height;
	}
}

static void rv32pvfb_tilecopy(struct fb_info *info, struct fb_tilearea *area)
{
	struct rv32pvfb_par *par = info->par;
	u32 arguments[8];

	if (!par->tile_width || !par->tile_height ||
	    area->sx > U32_MAX / par->tile_width ||
	    area->sy > U32_MAX / par->tile_height ||
	    area->dx > U32_MAX / par->tile_width ||
	    area->dy > U32_MAX / par->tile_height ||
	    area->width > U32_MAX / par->tile_width ||
	    area->height > U32_MAX / par->tile_height)
		return;
	arguments[0] = area->sx * par->tile_width;
	arguments[1] = area->sy * par->tile_height;
	arguments[2] = area->dx * par->tile_width;
	arguments[3] = area->dy * par->tile_height;
	arguments[4] = area->width * par->tile_width;
	arguments[5] = area->height * par->tile_height;
	arguments[6] = 0;
	arguments[7] = 0;
	(void)rv32pvfb_submit(info, RV32PV_OP_COPY, arguments, NULL, 0);
}

static void rv32pvfb_tilefill(struct fb_info *info, struct fb_tilerect *rect)
{
	u32 arguments[8] = {
		rect->sx, rect->sy, rect->width, rect->height, rect->index,
		rv32pvfb_color(info, rect->fg), rv32pvfb_color(info, rect->bg),
		rect->rop == ROP_XOR ? RV32PV_ROP_XOR : RV32PV_ROP_COPY,
	};

	(void)rv32pvfb_submit(info, RV32PV_OP_TILE_FILL, arguments, NULL, 0);
}

static void rv32pvfb_tileblit(struct fb_info *info, struct fb_tileblit *blit)
{
	struct rv32pvfb_par *par = info->par;
	size_t data_length;
	u32 arguments[8];
	u32 first_index;
	bool uniform = true;

	if (!blit->length ||
	    check_mul_overflow((size_t)blit->length, sizeof(*blit->indices),
			       &data_length) ||
	    data_length > par->staging_size ||
	    data_length > par->payload_limit)
		return;
	first_index = blit->indices[0];
	for (u32 tile = 1; tile < blit->length; tile++) {
		if (blit->indices[tile] != first_index) {
			uniform = false;
			break;
		}
	}
	if (uniform) {
		u32 fill_arguments[8] = {
			blit->sx, blit->sy, blit->width, blit->height,
			first_index, rv32pvfb_color(info, blit->fg),
			rv32pvfb_color(info, blit->bg), RV32PV_ROP_COPY,
		};

		(void)rv32pvfb_submit(info, RV32PV_OP_TILE_FILL,
				    fill_arguments, NULL, 0);
		return;
	}
	arguments[0] = blit->sx;
	arguments[1] = blit->sy;
	arguments[2] = blit->width;
	arguments[3] = blit->height;
	arguments[4] = rv32pvfb_color(info, blit->fg);
	arguments[5] = rv32pvfb_color(info, blit->bg);
	arguments[6] = blit->length;
	arguments[7] = data_length;
	(void)rv32pvfb_submit(info, RV32PV_OP_TILE_BLIT, arguments,
			    blit->indices, data_length);
}

static void rv32pvfb_tilecursor(struct fb_info *info,
				struct fb_tilecursor *cursor)
{
	u32 arguments[8] = {
		cursor->sx, cursor->sy, cursor->mode, cursor->shape,
		rv32pvfb_color(info, cursor->fg),
		rv32pvfb_color(info, cursor->bg),
	};

	(void)rv32pvfb_submit(info, RV32PV_OP_TILE_CURSOR, arguments, NULL, 0);
}

static int rv32pvfb_get_tilemax(struct fb_info *info)
{
	(void)info;
	return 256;
}

static struct fb_tile_ops rv32pvfb_tile_ops = {
	.fb_settile = rv32pvfb_settile,
	.fb_tilecopy = rv32pvfb_tilecopy,
	.fb_tilefill = rv32pvfb_tilefill,
	.fb_tileblit = rv32pvfb_tileblit,
	.fb_tilecursor = rv32pvfb_tilecursor,
	.fb_get_tilemax = rv32pvfb_get_tilemax,
};
#endif

static void rv32pvfb_destroy(struct fb_info *info)
{
	struct rv32pvfb_par *par = info->par;
	struct resource *mem = par->mem;

	if (info->screen_base)
		iounmap(info->screen_base);
	framebuffer_release(info);
	if (mem)
		release_mem_region(mem->start, resource_size(mem));
}

static const struct fb_ops rv32pvfb_ops = {
	.owner = THIS_MODULE,
	__FB_DEFAULT_IOMEM_OPS_RDWR,
	__FB_DEFAULT_IOMEM_OPS_MMAP,
	.fb_destroy = rv32pvfb_destroy,
	.fb_setcolreg = rv32pvfb_setcolreg,
	.fb_fillrect = rv32pvfb_fillrect,
	.fb_copyarea = rv32pvfb_copyarea,
	.fb_imageblit = rv32pvfb_imageblit,
	.fb_sync = rv32pvfb_sync,
};

static int rv32pvfb_probe(struct platform_device *pdev)
{
	struct device_node *node = pdev->dev.of_node;
	struct resource *resource;
	struct resource *mem;
	struct fb_info *info;
	struct rv32pvfb_par *par;
	const char *format;
	u32 width, height, stride, version;
	u32 accel_offset, staging_offset, staging_size, fence_offset;
	resource_size_t aperture_size;
	int ret;

	if (!node || of_property_read_u32(node, "width", &width) ||
	    of_property_read_u32(node, "height", &height) ||
	    of_property_read_u32(node, "stride", &stride) ||
	    of_property_read_string(node, "format", &format) ||
	    strcmp(format, "r5g6b5"))
		return -EINVAL;
	if (of_property_read_u32(node, "mini-rv32ima,accel-offset",
				 &accel_offset) ||
	    of_property_read_u32(node, "mini-rv32ima,staging-offset",
				 &staging_offset) ||
	    of_property_read_u32(node, "mini-rv32ima,staging-size",
				 &staging_size) ||
	    of_property_read_u32(node, "mini-rv32ima,fence-offset",
				 &fence_offset))
		return -EINVAL;

	resource = platform_get_resource(pdev, IORESOURCE_MEM, 0);
	if (!resource)
		return -EINVAL;
	aperture_size = resource_size(resource);
	if ((u64)stride * height > aperture_size ||
	    accel_offset > aperture_size || RV32PV_REG_SIZE >
		aperture_size - accel_offset || staging_offset > aperture_size ||
	    staging_size > aperture_size - staging_offset ||
	    fence_offset > aperture_size || sizeof(u32) >
		aperture_size - fence_offset)
		return -EINVAL;

	mem = request_mem_region(resource->start, aperture_size, "rv32-pvfb");
	if (!mem) {
		dev_warn(&pdev->dev, "cannot reserve framebuffer memory at %pR\n",
			 resource);
		mem = resource;
	}
	info = framebuffer_alloc(sizeof(*par), &pdev->dev);
	if (!info) {
		ret = -ENOMEM;
		goto error_release_region;
	}
	platform_set_drvdata(pdev, info);
	par = info->par;
	info->screen_base = ioremap_wc(mem->start, aperture_size);
	if (!info->screen_base) {
		ret = -ENOMEM;
		goto error_release_info;
	}

	info->fix = rv32pvfb_fix;
	info->fix.smem_start = mem->start;
	info->fix.smem_len = aperture_size;
	info->fix.line_length = stride;
	info->var = rv32pvfb_var;
	info->var.xres = width;
	info->var.yres = height;
	info->var.xres_virtual = width;
	info->var.yres_virtual = height;
	info->fbops = &rv32pvfb_ops;
	/*
	 * Tell fbcon that scrolling can use one native copy followed by a fill.
	 * Deliberately leave HWACCEL_IMAGEBLIT clear: the legacy fbcon heuristic
	 * otherwise redraws every visible text row instead of using copyarea.
	 * The imageblit callback itself remains accelerated for normal text.
	 */
	info->flags = FBINFO_HWACCEL_COPYAREA | FBINFO_HWACCEL_FILLRECT;
	info->pseudo_palette = par->palette;
	par->registers = info->screen_base + accel_offset;
	par->staging = info->screen_base + staging_offset;
	par->fence = info->screen_base + fence_offset;
	par->staging_size = staging_size;
	par->payload_limit = min(staging_size, RV32PV_LEGACY_PAYLOAD_LIMIT);
	version = ioread32(par->registers + RV32PV_REG_VERSION);
	if (ioread32(par->registers + RV32PV_REG_MAGIC) == RV32PV_MAGIC &&
	    version >= 1 && version <= RV32PV_VERSION) {
		par->features = ioread32(par->registers + RV32PV_REG_FEATURES);
		if (par->features & RV32PV_FEATURE_ASYNC_FIFO)
			par->queue_depth = ioread32(par->registers +
						    RV32PV_REG_QUEUE_DEPTH);
		if (par->features & RV32PV_FEATURE_PAYLOAD_POOL) {
			u32 limit = ioread32(par->registers +
					     RV32PV_REG_PAYLOAD_LIMIT);

			par->payload_blocks = ioread32(par->registers +
						       RV32PV_REG_PAYLOAD_BLOCKS);
			if (!limit || limit > staging_size || !par->payload_blocks) {
				dev_warn(&pdev->dev,
					 "invalid payload pool; disabling payload operations\n");
				par->features &= ~(RV32PV_FEATURE_PAYLOAD_POOL |
						   RV32PV_FEATURE_IMAGE1 |
						   RV32PV_FEATURE_TILE);
			} else {
				par->payload_limit = limit;
			}
		}
		if ((par->features & RV32PV_FEATURE_SURFACE_INFO) &&
		    (ioread32(par->registers + RV32PV_REG_SURFACE_WIDTH) != width ||
		     ioread32(par->registers + RV32PV_REG_SURFACE_HEIGHT) != height ||
		     ioread32(par->registers + RV32PV_REG_SURFACE_STRIDE) != stride ||
		     ioread32(par->registers + RV32PV_REG_SURFACE_FORMAT) !=
			     RV32PV_FORMAT_RGB565)) {
			dev_warn(&pdev->dev,
				 "DT and display backend geometry differ; disabling native drawing\n");
			par->features &= ~(RV32PV_FEATURE_FILL |
					   RV32PV_FEATURE_COPY |
					   RV32PV_FEATURE_IMAGE1 |
					   RV32PV_FEATURE_TILE);
		}
#ifdef CONFIG_FB_TILEBLITTING
		if (par->features & RV32PV_FEATURE_TILE) {
			info->flags |= FBINFO_MISC_TILEBLITTING;
			info->tileops = &rv32pvfb_tile_ops;
		}
#endif
	} else {
		dev_warn(&pdev->dev,
			 "acceleration mailbox unavailable; using software fbdev ops\n");
	}
	if (mem != resource)
		par->mem = mem;

	ret = devm_aperture_acquire_for_platform_device(pdev, mem->start,
							 aperture_size);
	if (ret)
		goto error_unmap;
	ret = register_framebuffer(info);
	if (ret)
		goto error_unmap;
	dev_info(&pdev->dev,
		 "fb%d: %ux%u RGB565, features=0x%x, queue=%u, payload=%u bytes/%u blocks, standard mmap enabled\n",
		 info->node, width, height, par->features, par->queue_depth,
		 par->payload_limit, par->payload_blocks);
	return 0;

error_unmap:
	iounmap(info->screen_base);
error_release_info:
	framebuffer_release(info);
error_release_region:
	if (mem != resource)
		release_mem_region(mem->start, resource_size(mem));
	return ret;
}

static void rv32pvfb_remove(struct platform_device *pdev)
{
	unregister_framebuffer(platform_get_drvdata(pdev));
}

static const struct of_device_id rv32pvfb_of_match[] = {
	{ .compatible = "mini-rv32ima,pvfb" },
	{ }
};
MODULE_DEVICE_TABLE(of, rv32pvfb_of_match);

static struct platform_driver rv32pvfb_driver = {
	.driver = {
		.name = "rv32-pvfb",
		.of_match_table = rv32pvfb_of_match,
	},
	.probe = rv32pvfb_probe,
	.remove = rv32pvfb_remove,
};
module_platform_driver(rv32pvfb_driver);

MODULE_AUTHOR("mini-rv32ima contributors");
MODULE_DESCRIPTION("mini-rv32ima paravirtual accelerated framebuffer");
MODULE_LICENSE("GPL");
