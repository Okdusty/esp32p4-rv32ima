#include <assert.h>
#include <stddef.h>
#include <stdint.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>

#define MINI_RV32_RAM_SIZE (16u * 1024u * 1024u)

static uint8_t linear_mmio[32];
static uint32_t linear_publish_start;
static uint32_t linear_publish_end;
static uint32_t linear_publish_calls;
static uint32_t linear_publish_bytes;
static uint32_t linear_publish_count;

static void publish_linear_mmio(uint32_t start, uint32_t end,
				uint32_t calls, uint32_t bytes)
{
	linear_publish_start = start;
	linear_publish_end = end;
	linear_publish_calls = calls;
	linear_publish_bytes = bytes;
	linear_publish_count++;
}

#define MINIRV32_LINEAR_MMIO_APERTURE
#define MINIRV32_LINEAR_MMIO_BASE 0x11800000u
#define MINIRV32_LINEAR_MMIO_SIZE sizeof(linear_mmio)
#define MINIRV32_LINEAR_MMIO_HOST linear_mmio
#define MINIRV32_LINEAR_MMIO_PUBLISH(start, end, calls, bytes) \
	publish_linear_mmio((start), (end), (calls), (bytes))
#define MINIRV32_PERF_STATS
#define MINIRV32_IMPLEMENTATION
#include "../main/mini-rv32ima.h"

static void put32(uint8_t *image, uint32_t offset, uint32_t value)
{
	memcpy(image + offset, &value, sizeof(value));
}

static uint32_t get32(const uint8_t *image, uint32_t offset)
{
	uint32_t value;

	memcpy(&value, image + offset, sizeof(value));
	return value;
}

static uint32_t encode_store(uint32_t funct3, uint32_t rs1, uint32_t rs2,
			     uint32_t immediate)
{
	return ((immediate >> 5) << 25) | (rs2 << 20) | (rs1 << 15) |
	       (funct3 << 12) | ((immediate & 0x1fu) << 7) | 0x23u;
}

static uint32_t encode_load(uint32_t funct3, uint32_t rd, uint32_t rs1,
			    uint32_t immediate)
{
	return (immediate << 20) | (rs1 << 15) | (funct3 << 12) |
	       (rd << 7) | 0x03u;
}

static uint32_t encode_op_immediate(uint32_t funct3, uint32_t rd,
				    uint32_t rs1, uint32_t immediate)
{
	return ((immediate & 0xfffu) << 20) | (rs1 << 15) |
	       (funct3 << 12) | (rd << 7) | 0x13u;
}

static uint32_t encode_branch(uint32_t funct3, uint32_t rs1, uint32_t rs2,
			      uint32_t immediate)
{
	return ((immediate & 0x1000u) << 19) |
	       ((immediate & 0x07e0u) << 20) | (rs2 << 20) | (rs1 << 15) |
	       (funct3 << 12) | ((immediate & 0x001eu) << 7) |
	       ((immediate & 0x0800u) >> 4) | 0x63u;
}

static uint32_t encode_jump(uint32_t rd, uint32_t immediate)
{
	return ((immediate & 0x100000u) << 11) |
	       ((immediate & 0x0007feu) << 20) |
	       ((immediate & 0x000800u) << 9) |
	       (immediate & 0x0ff000u) | (rd << 7) | 0x6fu;
}

static void put_cache_test_program(uint8_t *image, uint32_t code_offset)
{
	/* lw x5, 0(x6); lw x7, 4(x6); sw x8, 8(x6); sw x8, 12(x6) */
	put32(image, code_offset + 0u, 0x00032283u);
	put32(image, code_offset + 4u, 0x00432383u);
	put32(image, code_offset + 8u, 0x00832423u);
	put32(image, code_offset + 12u, 0x00832623u);
}

static void test_supervisor_linear_page_cache(uint8_t *image)
{
	struct MiniRV32IMAState state = { 0 };
	const uint32_t code_offset = MINIRV32_KERNEL_LINEAR_PHYS_OFFSET;
	const uint32_t data_offset = code_offset + 0x1000u;

	memset(image, 0, MINI_RV32_RAM_SIZE);
	put_cache_test_program(image, code_offset);
	put32(image, data_offset + 0u, 0x12345678u);
	put32(image, data_offset + 4u, 0x89abcdefu);

	state.pc = MINIRV32_KERNEL_LINEAR_BASE;
	state.regs[6] = MINIRV32_KERNEL_LINEAR_BASE + 0x1000u;
	state.regs[8] = 0xa5a55a5au;
	state.extraflags = 1u;
	state.satp = 0x80000000u;
	assert(MiniRV32IMAStep(&state, image, 0, 0, 4) == 0);
	assert(state.regs[5] == 0x12345678u);
	assert(state.regs[7] == 0x89abcdefu);
	assert(get32(image, data_offset + 8u) == 0xa5a55a5au);
	assert(get32(image, data_offset + 12u) == 0xa5a55a5au);
	assert(state.perf.read_ram_page_hits == 1u);
	assert(state.perf.write_ram_page_hits == 1u);
}

static void test_user_sv32_page_cache(uint8_t *image)
{
	struct MiniRV32IMAState state = { 0 };
	const uint32_t root_offset = 0x0000u;
	const uint32_t code_offset = 0x1000u;
	const uint32_t data_offset = 0x2000u;
	const uint32_t leaf_offset = 0x3000u;
	const uint32_t code_va = 0x00100000u;
	const uint32_t data_va = 0x00200000u;
	const uint32_t root_ppn =
		(MINIRV32_RAM_IMAGE_OFFSET + root_offset) >> 12;
	const uint32_t leaf_ppn =
		(MINIRV32_RAM_IMAGE_OFFSET + leaf_offset) >> 12;
	const uint32_t code_ppn =
		(MINIRV32_RAM_IMAGE_OFFSET + code_offset) >> 12;
	const uint32_t data_ppn =
		(MINIRV32_RAM_IMAGE_OFFSET + data_offset) >> 12;

	memset(image, 0, MINI_RV32_RAM_SIZE);
	put_cache_test_program(image, code_offset);
	put32(image, data_offset + 0u, 0x0badc0deu);
	put32(image, data_offset + 4u, 0xfeedfaceu);
	/* Both virtual pages share VPN[1] zero and this second-level table. */
	put32(image, root_offset, (leaf_ppn << 10) | 0x001u);
	/* V|R|X|U|A */
	put32(image, leaf_offset + (((code_va >> 12) & 0x3ffu) * 4u),
	      (code_ppn << 10) | 0x05bu);
	/* V|R|W|U|A|D */
	put32(image, leaf_offset + (((data_va >> 12) & 0x3ffu) * 4u),
	      (data_ppn << 10) | 0x0d7u);

	state.pc = code_va;
	state.regs[6] = data_va;
	state.regs[8] = 0x13579bdfu;
	state.extraflags = 0u;
	state.satp = 0x80000000u | root_ppn;
	assert(MiniRV32IMAStep(&state, image, 0, 0, 4) == 0);
	assert(state.regs[5] == 0x0badc0deu);
	assert(state.regs[7] == 0xfeedfaceu);
	assert(get32(image, data_offset + 8u) == 0x13579bdfu);
	assert(get32(image, data_offset + 12u) == 0x13579bdfu);
	assert(state.perf.tlb_walks == 2u);
	assert(state.perf.read_ram_page_hits == 1u);
	assert(state.perf.write_ram_page_hits == 1u);
}

static void test_linear_mmio_aperture(uint8_t *image)
{
	struct MiniRV32IMAState state = { 0 };

	memset(image, 0, MINI_RV32_RAM_SIZE);
	memset(linear_mmio, 0, sizeof(linear_mmio));
	linear_publish_start = 0;
	linear_publish_end = 0;
	linear_publish_calls = 0;
	linear_publish_bytes = 0;
	linear_publish_count = 0;
	put32(image, 0u, encode_store(0u, 6u, 8u, 1u));
	put32(image, 4u, encode_store(1u, 6u, 9u, 2u));
	put32(image, 8u, encode_store(2u, 6u, 10u, 4u));
	put32(image, 12u, encode_load(0u, 11u, 6u, 1u));
	put32(image, 16u, encode_load(4u, 12u, 6u, 1u));
	put32(image, 20u, encode_load(1u, 13u, 6u, 2u));
	put32(image, 24u, encode_load(5u, 14u, 6u, 2u));
	put32(image, 28u, encode_load(2u, 15u, 6u, 4u));

	state.pc = MINIRV32_RAM_IMAGE_OFFSET;
	state.regs[6] = MINIRV32_LINEAR_MMIO_BASE;
	state.regs[8] = 0xabu;
	state.regs[9] = 0xcdefu;
	state.regs[10] = 0x12345678u;
	state.extraflags = 3u;
	assert(MiniRV32IMAStep(&state, image, 0, 0, 8) == 0);
	assert(linear_mmio[1] == 0xabu);
	assert(linear_mmio[2] == 0xefu && linear_mmio[3] == 0xcdu);
	assert(linear_mmio[4] == 0x78u && linear_mmio[5] == 0x56u &&
	       linear_mmio[6] == 0x34u && linear_mmio[7] == 0x12u);
	assert(state.regs[11] == 0xffffffabu);
	assert(state.regs[12] == 0x000000abu);
	assert(state.regs[13] == 0xffffcdefu);
	assert(state.regs[14] == 0x0000cdefu);
	assert(state.regs[15] == 0x12345678u);
	assert(linear_publish_count == 1u);
	assert(linear_publish_start == 1u);
	assert(linear_publish_end == 8u);
	assert(linear_publish_calls == 3u);
	assert(linear_publish_bytes == 7u);
}

static void test_negative_immediates(uint8_t *image)
{
	struct MiniRV32IMAState state = { 0 };
	const uint32_t code_offset = 0x4000u;
	const uint32_t data_offset = 0x5000u;
	const uint32_t code_pa = MINIRV32_RAM_IMAGE_OFFSET + code_offset;

	memset(image, 0, MINI_RV32_RAM_SIZE);
	/* x5 counts down through a negative ADDI and negative BNE displacement. */
	put32(image, code_offset + 0u,
	      encode_op_immediate(0u, 5u, 0u, 3u));
	put32(image, code_offset + 4u,
	      encode_op_immediate(0u, 5u, 5u, (uint32_t)-1));
	put32(image, code_offset + 8u,
	      encode_branch(1u, 5u, 0u, (uint32_t)-4));
	/* Store and reload through negative S- and I-type displacements. */
	put32(image, code_offset + 12u,
	      encode_store(2u, 6u, 8u, (uint32_t)-4));
	put32(image, code_offset + 16u,
	      encode_load(2u, 9u, 6u, (uint32_t)-4));

	state.pc = code_pa;
	state.regs[6] = MINIRV32_RAM_IMAGE_OFFSET + data_offset + 4u;
	state.regs[8] = 0xa55a0ff0u;
	state.extraflags = 3u;
	assert(MiniRV32IMAStep(&state, image, 0, 0, 9) == 0);
	assert(state.regs[5] == 0u);
	assert(get32(image, data_offset) == 0xa55a0ff0u);
	assert(state.regs[9] == 0xa55a0ff0u);
	assert(state.pc == code_pa + 20u);

	/* Exercise both signs of the J-type immediate without relying on x0. */
	memset(&state, 0, sizeof(state));
	put32(image, code_offset + 20u, encode_jump(11u, 8u));
	put32(image, code_offset + 24u,
	      encode_op_immediate(0u, 12u, 0u, 1u));
	put32(image, code_offset + 28u,
	      encode_jump(0u, (uint32_t)-4));
	state.pc = code_pa + 20u;
	state.extraflags = 3u;
	assert(MiniRV32IMAStep(&state, image, 0, 0, 3) == 0);
	assert(state.regs[11] == code_pa + 24u);
	assert(state.regs[12] == 1u);
	assert(state.pc == code_pa + 28u);
}

static void test_instruction_fetch_page_budget(uint8_t *image)
{
	struct MiniRV32IMAState state = { 0 };
	const uint32_t root_offset = 0x0000u;
	const uint32_t leaf_offset = 0x3000u;
	const uint32_t first_code_offset = 0x6000u;
	const uint32_t second_code_offset = 0x9000u;
	const uint32_t code_va = 0x00100000u;
	const uint32_t root_ppn =
		(MINIRV32_RAM_IMAGE_OFFSET + root_offset) >> 12;
	const uint32_t leaf_ppn =
		(MINIRV32_RAM_IMAGE_OFFSET + leaf_offset) >> 12;
	const uint32_t first_code_ppn =
		(MINIRV32_RAM_IMAGE_OFFSET + first_code_offset) >> 12;
	const uint32_t second_code_ppn =
		(MINIRV32_RAM_IMAGE_OFFSET + second_code_offset) >> 12;
	const uint32_t executable_user_pte = 0x05bu; /* V|R|X|U|A */

	memset(image, 0, MINI_RV32_RAM_SIZE);
	put32(image, root_offset, (leaf_ppn << 10) | 0x001u);
	put32(image, leaf_offset + (((code_va >> 12) & 0x3ffu) * 4u),
	      (first_code_ppn << 10) | executable_user_pte);
	put32(image,
	      leaf_offset + ((((code_va + 0x1000u) >> 12) & 0x3ffu) * 4u),
	      (second_code_ppn << 10) | executable_user_pte);

	/* Straight-line fetch must translate the non-contiguous second page. */
	put32(image, first_code_offset + 0xff8u,
	      encode_op_immediate(0u, 5u, 5u, 1u));
	put32(image, first_code_offset + 0xffcu,
	      encode_op_immediate(0u, 5u, 5u, 1u));
	put32(image, second_code_offset,
	      encode_op_immediate(0u, 5u, 5u, 1u));
	state.pc = code_va + 0xff8u;
	state.satp = 0x80000000u | root_ppn;
	assert(MiniRV32IMAStep(&state, image, 0, 0, 3) == 0);
	assert(state.regs[5] == 3u);
	assert(state.pc == code_va + 0x1004u);
	assert(state.perf.tlb_walks == 2u);

	/* A same-page taken branch can move directly to the final word.  Its
	 * refreshed budget must still force translation on the following fetch. */
	memset(&state, 0, sizeof(state));
	put32(image, first_code_offset + 0x100u,
	      encode_branch(1u, 1u, 0u, 0xefcu));
	state.pc = code_va + 0x100u;
	state.regs[1] = 1u;
	state.satp = 0x80000000u | root_ppn;
	assert(MiniRV32IMAStep(&state, image, 0, 0, 3) == 0);
	assert(state.regs[5] == 2u);
	assert(state.pc == code_va + 0x1004u);
	assert(state.perf.tlb_walks == 2u);
}

int main(void)
{
	uint8_t *image = calloc(1, MINI_RV32_RAM_SIZE);

	assert(image != NULL);
	assert(offsetof(struct MiniRV32IMAState, fast_tlb) <
	       offsetof(struct MiniRV32IMAState, tlb));
	assert((offsetof(struct MiniRV32IMAState, fast_tlb) & 127u) == 0);
	assert((offsetof(struct MiniRV32IMAState, tlb) & 127u) == 0);
	test_supervisor_linear_page_cache(image);
	test_user_sv32_page_cache(image);
	test_linear_mmio_aperture(image);
	test_negative_immediates(image);
	test_instruction_fetch_page_budget(image);
	free(image);
	puts("mini-rv32ima page-cache tests passed");
	return 0;
}
