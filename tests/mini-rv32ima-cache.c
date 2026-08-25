#include <assert.h>
#include <stddef.h>
#include <stdint.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>

#define MINI_RV32_RAM_SIZE (16u * 1024u * 1024u)
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
	free(image);
	puts("mini-rv32ima page-cache tests passed");
	return 0;
}
