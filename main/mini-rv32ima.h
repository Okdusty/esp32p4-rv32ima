// Copyright 2022 Charles Lohr, you may use this file or any portions herein under any of the BSD, MIT, or CC0 licenses.

#ifndef _MINI_RV32IMAH_H
#define _MINI_RV32IMAH_H

/**
    To use mini-rv32ima.h for the bare minimum, the following:

	#define MINI_RV32_RAM_SIZE ram_amt
	#define MINIRV32_IMPLEMENTATION

	#include "mini-rv32ima.h"

	Though, that's not _that_ interesting. You probably want I/O!


	Notes:
		* There is a dedicated CLNT at 0x10000000.
		* There is free MMIO from there to 0x12000000.
		* You can put things like a UART, or whatever there.
		* Feel free to override any of the functionality with macros.
*/

#ifndef MINIRV32WARN
	#define MINIRV32WARN( x... );
#endif

#ifndef MINIRV32_DECORATE
	#define MINIRV32_DECORATE static
#endif

#ifndef MINIRV32_RAM_IMAGE_OFFSET
	#define MINIRV32_RAM_IMAGE_OFFSET  0x48000000
#endif

#ifndef MINIRV32_TLB_ENTRIES
	#define MINIRV32_TLB_ENTRIES 2048
#endif

/*
 * Linux RV32 uses a fixed linear mapping of guest RAM at PAGE_OFFSET.  The
 * ESP32-P4 guest has no memory hotplug and CONFIG_STRICT_KERNEL_RWX is off, so
 * supervisor accesses in this window are exactly VA - PAGE_OFFSET + RAM base.
 * Bypass even the software-TLB lookup for this dominant kernel path.  DMA and
 * MMIO remaps live outside the linear window and still use their PTEs.
 */
#ifndef MINIRV32_KERNEL_LINEAR_BASE
	#define MINIRV32_KERNEL_LINEAR_BASE 0xc0000000u
#endif

/* Linux is linked at PAGE_OFFSET and loaded after the 4 MiB image headroom. */
#ifndef MINIRV32_KERNEL_LINEAR_PHYS_OFFSET
	#define MINIRV32_KERNEL_LINEAR_PHYS_OFFSET 0x00400000u
#endif

#if (MINIRV32_TLB_ENTRIES & (MINIRV32_TLB_ENTRIES - 1)) != 0
	#error "MINIRV32_TLB_ENTRIES must be a power of two"
#endif

#ifndef MINIRV32_POSTEXEC
	#define MINIRV32_POSTEXEC(...);
#endif

#ifndef MINIRV32_HANDLE_MEM_STORE_CONTROL
	#define MINIRV32_HANDLE_MEM_STORE_CONTROL(...);
#endif

#ifndef MINIRV32_HANDLE_MEM_LOAD_CONTROL
	#define MINIRV32_HANDLE_MEM_LOAD_CONTROL(...);
#endif

#ifndef MINIRV32_HANDLE_CACHE_OP
	#define MINIRV32_HANDLE_CACHE_OP(...);
#endif

#ifndef MINIRV32_OTHERCSR_WRITE
	#define MINIRV32_OTHERCSR_WRITE(...);
#endif

#ifndef MINIRV32_OTHERCSR_READ
	#define MINIRV32_OTHERCSR_READ(...);
#endif

#ifndef MINIRV32_CUSTOM_MEMORY_BUS
	#define MINIRV32_STORE4( ofs, val ) *(uint32_t*)(image + ofs) = val
	#define MINIRV32_STORE2( ofs, val ) *(uint16_t*)(image + ofs) = val
	#define MINIRV32_STORE1( ofs, val ) *(uint8_t*)(image + ofs) = val
	#define MINIRV32_LOAD4( ofs ) *(uint32_t*)(image + ofs)
	#define MINIRV32_LOAD2( ofs ) *(uint16_t*)(image + ofs)
	#define MINIRV32_LOAD1( ofs ) *(uint8_t*)(image + ofs)
#endif

#ifndef MINIRV32_LOAD4_UNCACHED
	#define MINIRV32_LOAD4_UNCACHED( ofs ) MINIRV32_LOAD4( ofs )
	#define MINIRV32_LOAD2_UNCACHED( ofs ) MINIRV32_LOAD2( ofs )
	#define MINIRV32_LOAD1_UNCACHED( ofs ) MINIRV32_LOAD1( ofs )
	#define MINIRV32_STORE4_UNCACHED( ofs, val ) MINIRV32_STORE4( ofs, val )
	#define MINIRV32_STORE2_UNCACHED( ofs, val ) MINIRV32_STORE2( ofs, val )
	#define MINIRV32_STORE1_UNCACHED( ofs, val ) MINIRV32_STORE1( ofs, val )
#endif

// As a note: We quouple-ify these, because in HLSL, we will be operating with
// uint4's.  We are going to uint4 data to/from system RAM.
//
struct MiniRV32TLBEntry
{
	uint32_t virtual_page;
	uint32_t physical_page;
	uint32_t pte_pa;
	uint32_t pte;
};

#define MINIRV32_FAST_TLB_SETS 256

struct MiniRV32FastTLBEntry
{
	uint32_t tag;
	/* Physical pages are 4 KiB aligned, so bit 0 carries the uncached flag. */
	uint32_t physical_page_and_flags;
};

#ifdef MINIRV32_PERF_STATS
struct MiniRV32PerfStats
{
	/* Standard 32-bit opcodes are identified by instruction bits [6:2]. */
	uint32_t opcode[32];
	uint32_t load_funct3[8];
	uint32_t store_funct3[8];
	uint32_t opimm_funct3[8];
	uint32_t op_funct3[8];
	uint32_t branch_funct3[8];
	uint32_t muldiv;
	uint32_t fast_tlb_hits;
	uint32_t tlb_hits;
	uint32_t tlb_walks;
	uint32_t tlb_flushes;
	uint32_t satp_writes;
	uint32_t sfence_vma;
	uint32_t sfence_global;
	uint32_t sfence_page;
	uint32_t exec_page_hits;
	uint32_t kernel_linear_hits;
	uint32_t read_page_hits;
	uint32_t read_ram_page_hits;
	uint32_t write_page_hits;
	uint32_t write_ram_page_hits;
};
#endif

struct MiniRV32IMAState
{
	uint32_t regs[32];

	uint32_t pc;
	uint32_t mstatus;
	uint32_t cyclel;
	uint32_t cycleh;

	uint32_t timerl;
	uint32_t timerh;
	uint32_t timermatchl;
	uint32_t timermatchh;

	uint32_t mscratch;
	uint32_t mtvec;
	uint32_t mie;
	uint32_t mip;

	uint32_t mepc;
	uint32_t mtval;
	uint32_t mcause;

  //Supervisor CSRs

  uint32_t satp;
  uint32_t stvec;
  uint32_t sscratch;
  uint32_t sepc;
  uint32_t scause;
  uint32_t stval;
  uint32_t sie;
  uint32_t sip;

	/* Keep per-instruction state close to the register/CSR block.  Previously
	 * this lived after the 41 KiB translation caches, costing the hot loop a
	 * second state-base register. */
	// Note: only a few bits are used.  (Machine = 3, User = 0, Supervisor = 1)
	// Bits 0..1 = privilege.
	// Bit 2 = WFI (Wait for interrupt)
	// Bit 3+ = Load/Store reservation LSBs.
	uint32_t extraflags;
	uint32_t tlb_epoch;

#ifdef MINIRV32_PERF_STATS
	struct MiniRV32PerfStats perf;
#endif

	/* The three small first-level caches are touched by ordinary loads, stores
	 * and instruction fetches. Keep them before the colder full TLB so their
	 * base stays near the register block and their lines do not inherit the
	 * full TLB's 32 KiB displacement. */
	struct MiniRV32FastTLBEntry fast_tlb[3][MINIRV32_FAST_TLB_SETS]
		__attribute__((aligned(128)));
	struct MiniRV32TLBEntry tlb[MINIRV32_TLB_ENTRIES]
		__attribute__((aligned(128)));
};

enum {
	ACCESS_READ  = 0,
	ACCESS_WRITE = 1,
	ACCESS_EXEC  = 2,
};

static uint32_t MiniRV32Translate(
	struct MiniRV32IMAState *state,
	uint8_t *image,
	uint32_t va,
	int access,
	int *fault,
	int *uncached);

static uint32_t MiniRV32TranslateSlow(
	struct MiniRV32IMAState *state,
	uint8_t *image,
	uint32_t va,
	int access,
	int *fault,
	int *uncached,
	uint32_t priv,
	uint32_t virtual_page,
	uint32_t fast_tag,
	struct MiniRV32FastTLBEntry *fast);

static int MiniRV32HandleSBI(struct MiniRV32IMAState *state);

MINIRV32_DECORATE int32_t MiniRV32IMAStep(
	struct MiniRV32IMAState *state,
	uint8_t *image, uint32_t vProcAddress,
	uint32_t elapsedUs, int count );

#ifdef MINIRV32_IMPLEMENTATION

#define CSR( x ) state->x
#define SETCSR( x, val ) { state->x = val; }
#define REG( x ) state->regs[x]
#define REGSET( x, val ) { state->regs[x] = val; }

static int MiniRV32PhysRead32(
	uint8_t *image, uint32_t pa, uint32_t *value)
{
	if (pa < MINIRV32_RAM_IMAGE_OFFSET)
		return -1;

	uint32_t ofs = pa - MINIRV32_RAM_IMAGE_OFFSET;

	if (ofs > MINI_RV32_RAM_SIZE - 4)
		return -1;

	*value = MINIRV32_LOAD4(ofs);
	return 0;
}

static int MiniRV32PhysWrite32(
	uint8_t *image, uint32_t pa, uint32_t value)
{
	if (pa < MINIRV32_RAM_IMAGE_OFFSET)
		return -1;

	uint32_t ofs = pa - MINIRV32_RAM_IMAGE_OFFSET;

	if (ofs > MINI_RV32_RAM_SIZE - 4)
		return -1;

	MINIRV32_STORE4(ofs, value);
	return 0;
}

/* Bit 31 is free because an Sv32 VPN is only 20 bits wide. */
#define MINIRV32_TLB_VALID (1u << 31)
#define MINIRV32_TLB_EPOCH_MASK 0x7fu
#define MINIRV32_TLB_EPOCH_SHIFT 24
#define MINIRV32_FAST_TLB_UNCACHED 1u

#ifdef MINIRV32_PERF_STATS
	#define MINIRV32_PERF_INC(field) (state->perf.field++)
	#define MINIRV32_PERF_FUNCT3_INC(field, instruction) \
		(state->perf.field[((instruction) >> 12) & 7u]++)
#else
	#define MINIRV32_PERF_INC(field) do { } while (0)
	#define MINIRV32_PERF_FUNCT3_INC(field, instruction) do { } while (0)
#endif

static void MiniRV32FlushTLB(struct MiniRV32IMAState *state)
{
	/* Seven unused fast-tag bits form a generation number.  Most invalidations
	 * now cost one increment instead of walking 2,816 entries spread over
	 * roughly 41 KiB.  Clear physically only when the generation wraps. */
	state->tlb_epoch =
		(state->tlb_epoch + 1u) & MINIRV32_TLB_EPOCH_MASK;
	MINIRV32_PERF_INC(tlb_flushes);
	if (state->tlb_epoch != 0u)
		return;

	for (unsigned int i = 0; i < MINIRV32_TLB_ENTRIES; i++)
		state->tlb[i].virtual_page = 0;

	for (unsigned int access = 0; access < 3; access++)
		for (unsigned int i = 0; i < MINIRV32_FAST_TLB_SETS; i++)
			state->fast_tlb[access][i].tag = 0;
}

static __attribute__((noinline)) void MiniRV32FlushTLBPage(
	struct MiniRV32IMAState *state, uint32_t va)
{
	uint32_t virtual_page = va >> 12;
	uint32_t tlb_index =
		(virtual_page ^ (virtual_page >> 8)) &
		(MINIRV32_TLB_ENTRIES - 1);
	uint32_t fast_index =
		virtual_page & (MINIRV32_FAST_TLB_SETS - 1);

	/* Both caches are direct-mapped.  Invalidating the target sets is enough;
	 * evicting an unrelated collision is harmless and avoids a tag load. */
	state->tlb[tlb_index].virtual_page = 0;
	for (unsigned int access = 0; access < 3; access++)
		state->fast_tlb[access][fast_index].tag = 0;
}

static int MiniRV32CheckLeafPermissions(
	struct MiniRV32IMAState *state,
	uint32_t pte,
	uint32_t priv,
	int access)
{
	uint32_t R = (pte >> 1) & 1;
	uint32_t W = (pte >> 2) & 1;
	uint32_t X = (pte >> 3) & 1;
	uint32_t U = (pte >> 4) & 1;

	if (priv == 0) {
		if (!U)
			return -1;
	} else if (priv == 1 && U) {
		/* Supervisor mode may never execute user pages. */
		if (access == ACCESS_EXEC)
			return -1;

		/* SUM permits supervisor loads/stores to user pages. */
		if (!(CSR(mstatus) & (1u << 18)))
			return -1;
	}

	if (access == ACCESS_EXEC)
		return X ? 0 : -1;

	if (access == ACCESS_WRITE)
		return W ? 0 : -1;

	/* MXR allows executable pages to be read. */
	return (R || ((CSR(mstatus) & (1u << 19)) && X)) ? 0 : -1;
}

/*
 * Keep the overwhelmingly common fast-TLB hit in the interpreter itself.
 * Separating the page walk prevents its much larger cold path from making
 * every guest load/store pay a C call and return.
 */
static inline __attribute__((always_inline)) uint32_t MiniRV32Translate(
	struct MiniRV32IMAState *state,
	uint8_t *image,
	uint32_t va,
	int access,
	int *fault,
	int *uncached)
{
	*fault = 0;
	if (uncached)
		*uncached = 0;

	uint32_t priv = CSR(extraflags) & 3;

	/* M-mode ignores satp. MODE=0 is Bare and MODE=1 is Sv32. */
	if (priv == 3 || !(CSR(satp) & 0x80000000u))
		return va;

	uint32_t linear_offset = va - MINIRV32_KERNEL_LINEAR_BASE;
	if (priv == 1 &&
	    linear_offset <
		MINI_RV32_RAM_SIZE - MINIRV32_KERNEL_LINEAR_PHYS_OFFSET) {
		MINIRV32_PERF_INC(kernel_linear_hits);
		return MINIRV32_RAM_IMAGE_OFFSET +
			MINIRV32_KERNEL_LINEAR_PHYS_OFFSET + linear_offset;
	}

	uint32_t virtual_page = va >> 12;
	uint32_t epoch_tag =
		CSR(tlb_epoch) << MINIRV32_TLB_EPOCH_SHIFT;
	/* Include privilege, SUM and MXR in the fast permission context. */
	uint32_t fast_tag = virtual_page | MINIRV32_TLB_VALID |
		(priv << 20) | (((CSR(mstatus) >> 18) & 3u) << 22) |
		epoch_tag;
	struct MiniRV32FastTLBEntry *fast =
		&state->fast_tlb[access]
			[virtual_page & (MINIRV32_FAST_TLB_SETS - 1)];

	if (fast->tag == fast_tag) {
		uint32_t page_and_flags = fast->physical_page_and_flags;
		MINIRV32_PERF_INC(fast_tlb_hits);
		if (uncached)
			*uncached = !!(page_and_flags & MINIRV32_FAST_TLB_UNCACHED);
		return (page_and_flags & ~MINIRV32_FAST_TLB_UNCACHED) |
			(va & 0xfffu);
	}

	return MiniRV32TranslateSlow(state, image, va, access, fault, uncached,
		priv, virtual_page, fast_tag, fast);
}

/* Full TLB lookup and Sv32 page-table walk after a fast-TLB miss. */
static __attribute__((noinline)) uint32_t MiniRV32TranslateSlow(
	struct MiniRV32IMAState *state,
	uint8_t *image,
	uint32_t va,
	int access,
	int *fault,
	int *uncached,
	uint32_t priv,
	uint32_t virtual_page,
	uint32_t fast_tag,
	struct MiniRV32FastTLBEntry *fast)
{

	uint32_t tlb_index =
		(virtual_page ^ (virtual_page >> 8)) &
		(MINIRV32_TLB_ENTRIES - 1);
	struct MiniRV32TLBEntry *cached = &state->tlb[tlb_index];
	uint32_t cached_tag = virtual_page | MINIRV32_TLB_VALID |
		(CSR(tlb_epoch) << MINIRV32_TLB_EPOCH_SHIFT);

	if (cached->virtual_page == cached_tag) {
		MINIRV32_PERF_INC(tlb_hits);
		if (MiniRV32CheckLeafPermissions(
				state, cached->pte, priv, access) < 0) {
			*fault = 1;
			return va;
		}

		/* A is set on insertion; a later store may still need to set D. */
		if (access == ACCESS_WRITE && !(cached->pte & (1u << 7))) {
			uint32_t new_pte = cached->pte | (1u << 7);

			if (MiniRV32PhysWrite32(image, cached->pte_pa, new_pte) < 0) {
				*fault = 1;
				return va;
			}

			cached->pte = new_pte;
		}

		fast->physical_page_and_flags = cached->physical_page |
			((cached->pte & (1u << 9)) ?
			 MINIRV32_FAST_TLB_UNCACHED : 0u);
		fast->tag = fast_tag;
		if (uncached)
			*uncached = !!(cached->pte & (1u << 9));
		return cached->physical_page | (va & 0xfff);
	}
	MINIRV32_PERF_INC(tlb_walks);

	uint32_t vpn[2];
	vpn[0] = (va >> 12) & 0x3ff;
	vpn[1] = (va >> 22) & 0x3ff;

	/* satp[21:0] contains the root page-table PPN. */
	uint64_t table =
		((uint64_t)(CSR(satp) & 0x003fffffu)) << 12;

	for (int level = 1; level >= 0; level--) {
		uint64_t pte_pa64 =
			table + ((uint64_t)vpn[level] * 4);

		if (pte_pa64 > 0xffffffffULL) {
			*fault = 1;
			return va;
		}

		uint32_t pte_pa = (uint32_t)pte_pa64;
		uint32_t pte;

		if (MiniRV32PhysRead32(image, pte_pa, &pte) < 0) {
			*fault = 1;
			return va;
		}

		uint32_t V = (pte >> 0) & 1;
		uint32_t R = (pte >> 1) & 1;
		uint32_t W = (pte >> 2) & 1;
		uint32_t X = (pte >> 3) & 1;
		uint32_t A = (pte >> 6) & 1;
		uint32_t D = (pte >> 7) & 1;

		/* V=0 and the reserved W=1,R=0 combination are invalid. */
		if (!V || (!R && W)) {
			*fault = 1;
			return va;
		}

		/* A PTE with R or X set is a leaf. */
		if (R || X) {
			if (MiniRV32CheckLeafPermissions(
					state, pte, priv, access) < 0) {
				*fault = 1;
				return va;
			}

			/* Hardware-manage the Accessed and Dirty bits for Linux. */
			uint32_t new_pte = pte;

			if (!A)
				new_pte |= (1u << 6);

			if (access == ACCESS_WRITE && !D)
				new_pte |= (1u << 7);

			if (new_pte != pte) {
				if (MiniRV32PhysWrite32(image, pte_pa, new_pte) < 0) {
					*fault = 1;
					return va;
				}

				pte = new_pte;
			}

			uint32_t ppn0 = (pte >> 10) & 0x3ff;
			uint32_t ppn1 = (pte >> 20) & 0xfff;
			uint64_t pa;

			if (level == 1) {
				/* A 4 MiB superpage must have an aligned physical PPN. */
				if (ppn0 != 0) {
					*fault = 1;
					return va;
				}

				pa =
					((uint64_t)ppn1 << 22) |
					((uint64_t)vpn[0] << 12) |
					(va & 0xfff);
			} else {
				pa =
					((uint64_t)ppn1 << 22) |
					((uint64_t)ppn0 << 12) |
					(va & 0xfff);
			}

			if (pa > 0xffffffffULL) {
				*fault = 1;
				return va;
			}

			cached->physical_page = (uint32_t)pa & ~0xfffu;
			cached->pte_pa = pte_pa;
			cached->pte = pte;
			cached->virtual_page = cached_tag;
			fast->physical_page_and_flags = cached->physical_page |
				((pte & (1u << 9)) ?
				 MINIRV32_FAST_TLB_UNCACHED : 0u);
			fast->tag = fast_tag;
			if (uncached)
				*uncached = !!(pte & (1u << 9));

			return (uint32_t)pa;
		}

		/* A non-leaf PTE points to the next-level table. */
		table = ((uint64_t)(pte >> 10)) << 12;

		if (table > 0xffffffffULL) {
			*fault = 1;
			return va;
		}
	}

	*fault = 1;
	return va;
}

static int MiniRV32HandleSBI(struct MiniRV32IMAState *state)
{
	uint32_t ext = state->regs[17]; /* a7 */
	uint32_t fid = state->regs[16]; /* a6 */
	int32_t error = 0;
	uint32_t value = 0;

	if (ext == 0x10) {
		/* SBI Base extension. */
		switch (fid) {
		case 0: /* get_spec_version */
			value = 2; /* SBI 0.2 */
			break;
		case 1: /* get_impl_id */
			value = 0;
			break;
		case 2: /* get_impl_version */
			value = 1;
			break;
		case 3: { /* probe_extension */
			uint32_t wanted = state->regs[10];

			switch (wanted) {
			case 0x10:       /* BASE */
			case 0x54494d45: /* TIME */
				value = 1;
				break;
			default:
				value = 0;
				break;
			}
			break;
		}
		case 4: /* get_mvendorid */
		case 5: /* get_marchid */
		case 6: /* get_mimpid */
			value = 0;
			break;
		default:
			error = -2; /* SBI_ERR_NOT_SUPPORTED */
			break;
		}
	} else if (ext == 0x54494d45 && fid == 0) {
		/* SBI TIME set_timer. */
		uint64_t when =
			((uint64_t)state->regs[11] << 32) |
			(uint64_t)state->regs[10];

		state->timermatchl = (uint32_t)when;
		state->timermatchh = (uint32_t)(when >> 32);
		state->sip &= ~(1u << 5);
	} else if (ext == 0) {
		/* Legacy SBI set_timer. */
		uint64_t when =
			((uint64_t)state->regs[11] << 32) |
			(uint64_t)state->regs[10];

		state->timermatchl = (uint32_t)when;
		state->timermatchh = (uint32_t)(when >> 32);
		state->sip &= ~(1u << 5);
	} else {
		error = -2;
	}

	state->regs[10] = (uint32_t)error; /* a0 */
	state->regs[11] = value;           /* a1 */

	return 0;
}

/* This runs once per instruction batch and when interrupt-related CSRs
 * change, not once per guest opcode.  Keeping it out of the opcode function
 * reduces hot-loop code size without affecting the common dispatch path. */
static __attribute__((noinline)) uint32_t MiniRV32SupervisorInterruptCause(
	struct MiniRV32IMAState *state)
{
	uint32_t pending = state->sip & state->sie &
		((1u << 9) | (1u << 5));
	uint32_t privilege = state->extraflags & 3u;

	if (!pending ||
	    (privilege == 1u && !(state->mstatus & (1u << 1))) ||
	    privilege > 1u)
		return 0;

	/* Supervisor external interrupts have priority over timer interrupts. */
	return (pending & (1u << 9)) ? 9u : 5u;
}

MINIRV32_DECORATE int32_t MiniRV32IMAStep(
	struct MiniRV32IMAState *state,
	uint8_t *image, uint32_t vProcAddress,
	uint32_t elapsedUs, int count )
{
	uint32_t new_timer = CSR( timerl ) + elapsedUs;
	if( new_timer < CSR( timerl ) ) CSR( timerh )++;
	CSR( timerl ) = new_timer;

	/* Supervisor timer interrupt. */
	uint64_t now =
		((uint64_t)CSR(timerh) << 32) |
		CSR(timerl);

	/*
	 * elapsedUs updates the architectural clock between batches.  A guest
	 * may also poll rdtime inside one batch (Linux __delay does this), so
	 * retain a wall-clock anchor for an interpolated, read-only value.
	 */
	#if defined(MINIRV32_HOST_CYCLE_COUNT) && \
	    defined(MINIRV32_HOST_CYCLES_PER_US)
	uint64_t rdtime_timer_anchor = now;
	uint32_t rdtime_cycle_anchor = MINIRV32_HOST_CYCLE_COUNT();
	#elif defined(MINIRV32_HOST_TIME_US)
	uint64_t rdtime_timer_anchor = now;
	uint64_t rdtime_host_anchor = MINIRV32_HOST_TIME_US();
	#endif

	uint64_t match =
		((uint64_t)CSR(timermatchh) << 32) |
		CSR(timermatchl);

	if (match && now >= match) {
		CSR(sip) |= (1u << 5); /* STIP */
		CSR(extraflags) &= ~4u; /* Wake WFI. */
	} else {
		CSR(sip) &= ~(1u << 5);
	}

	// If WFI, don't run processor.
	if( CSR( extraflags ) & 4 )
		return 1;

	uint32_t trap = 0;
	uint32_t rval = 0;
	uint32_t pc = CSR( pc );
	uint32_t cycle = CSR( cyclel );

	uint32_t exec_virtual_page = UINT32_MAX;
	uint32_t exec_offset_bias = 0;
	uint32_t read_virtual_page = UINT32_MAX;
	uint32_t read_physical_page = 0;
	uint32_t read_ram_offset_bias = UINT32_MAX;
	int read_uncached = 0;
	uint32_t write_virtual_page = UINT32_MAX;
	uint32_t write_physical_page = 0;
	uint32_t write_ram_offset_bias = UINT32_MAX;
	int write_uncached = 0;
	uint32_t kernel_linear_limit =
		(((CSR(extraflags) & 3u) == 1u) &&
		 (CSR(satp) & 0x80000000u)) ?
			(MINI_RV32_RAM_SIZE -
			 MINIRV32_KERNEL_LINEAR_PHYS_OFFSET) : 0u;

	uint32_t supervisor_cause =
		MiniRV32SupervisorInterruptCause(state);
	int icount = 0;

	if (supervisor_cause) {
		trap = 0x80000000u | supervisor_cause;
		pc -= 4;
		/* No guest instruction executes in an interrupt-only batch. */
		count = 0;
	}
	else // No timer interrupt?  Execute a bunch of instructions.
	for( ; icount < count; icount++ )
	{
		uint32_t ir = 0;
		rval = 0;
		int fault;
		uint32_t pc_page = pc & ~0xfffu;
		uint32_t ofs_pc;

		if (pc_page == exec_virtual_page) {
			MINIRV32_PERF_INC(exec_page_hits);
			ofs_pc = pc + exec_offset_bias;
		} else {
			uint32_t phys_pc = MiniRV32Translate(
				state, image, pc, ACCESS_EXEC, &fault, NULL);

			if (fault) {
				trap = 12 + 1;
				rval = pc;
				break;
			}

			ofs_pc = phys_pc - MINIRV32_RAM_IMAGE_OFFSET;

			/* A valid physical page remains in-range for all page offsets. */
			if (ofs_pc >= MINI_RV32_RAM_SIZE) {
				trap = 1 + 1;  // Handle access violation on instruction read.
				break;
			}

			exec_virtual_page = pc_page;
			/* Unsigned wraparound represents host-offset minus guest VA. */
			exec_offset_bias = ofs_pc - pc;
		}

		#ifndef MINIRV32_TRUSTED_32BIT_FETCH
		if( ofs_pc & 3 )
		{
			trap = 1 + 0;  //Handle PC-misaligned access
			break;
		}
		else
		#endif
		{
				ir = MINIRV32_LOAD4( ofs_pc );
				uint32_t opcode = (ir >> 2) & 0x1fu;
				#ifdef MINIRV32_PERF_STATS
				state->perf.opcode[opcode]++;
				#endif
				/* Decode rd only for opcode families that can write it.  Keeping
				 * this out of the common fetch path saves two host instructions
				 * for branches, stores, fences and other no-destination opcodes. */
				uint32_t rdid = 0;

				/* All supported instructions are 32-bit encodings ending in 0b11.
				 * Bits [6:2] form a compact switch key, reducing the range tests GCC
				 * emitted for a sparse switch over the full seven bits. */
				#ifndef MINIRV32_TRUSTED_32BIT_FETCH
				if (__builtin_expect((ir & 3u) != 3u, 0)) {
					trap = 2 + 1;
				} else
				#endif
				switch (opcode)
				{
					case 0b01101: // LUI
					rdid = (ir >> 7) & 0x1f;
					rval = ( ir & 0xfffff000 );
					break;
					case 0b00101: // AUIPC
					rdid = (ir >> 7) & 0x1f;
					rval = pc + ( ir & 0xfffff000 );
					break;
					case 0b11011: // JAL
				{
					rdid = (ir >> 7) & 0x1f;
					int32_t reladdy = ((ir & 0x80000000)>>11) | ((ir & 0x7fe00000)>>20) | ((ir & 0x00100000)>>9) | ((ir&0x000ff000));
					if( reladdy & 0x00100000 ) reladdy |= 0xffe00000; // Sign extension.
					rval = pc + 4;
					pc = pc + reladdy - 4;
					break;
				}
					case 0b11001: // JALR
				{
					rdid = (ir >> 7) & 0x1f;
					uint32_t imm = ir >> 20;
					int32_t imm_se = imm | (( imm & 0x800 )?0xfffff000:0);
					rval = pc + 4;
					pc = ( (REG( (ir >> 15) & 0x1f ) + imm_se) & ~1) - 4;
					break;
				}
						case 0b11000: // Branch
					{
						MINIRV32_PERF_FUNCT3_INC(branch_funct3, ir);
						uint32_t funct3 = (ir >> 12) & 0x7;
						uint32_t immm4 = ((ir & 0xf00)>>7) | ((ir & 0x7e000000)>>20) | ((ir & 0x80) << 4) | ((ir >> 31)<<12);
					if( immm4 & 0x1000 ) immm4 |= 0xffffe000;
					int32_t rs1 = REG((ir >> 15) & 0x1f);
					int32_t rs2 = REG((ir >> 20) & 0x1f);
					immm4 = pc + immm4 - 4;
					rdid = 0;
						/* BNE and BEQ make up most Linux branches. Keep them out of
						 * the indirect switch used by the less common comparisons. */
						if (__builtin_expect(funct3 == 0b001, 1)) {
							if (rs1 != rs2)
								pc = immm4;
						} else if (funct3 == 0b000) {
							if (rs1 == rs2)
								pc = immm4;
						} else switch (funct3) {
							// BLT, BGE, BLTU, BGEU
							case 0b100: if( rs1 < rs2 ) pc = immm4; break;
						case 0b101: if( rs1 >= rs2 ) pc = immm4; break; //BGE
						case 0b110: if( (uint32_t)rs1 < (uint32_t)rs2 ) pc = immm4; break;   //BLTU
						case 0b111: if( (uint32_t)rs1 >= (uint32_t)rs2 ) pc = immm4; break;  //BGEU
						default: trap = (2+1);
					}
					break;
				}
					case 0b00000: // Load
					{
						rdid = (ir >> 7) & 0x1f;
						MINIRV32_PERF_FUNCT3_INC(load_funct3, ir);
						uint32_t funct3 = (ir >> 12) & 7;
					uint32_t rs1 = REG((ir >> 15) & 0x1f);
					uint32_t imm = ir >> 20;
					int32_t imm_se =
						imm | ((imm & 0x800) ? 0xfffff000 : 0);
					uint32_t virtual_addr = rs1 + imm_se;
					int uncached = 0;
					uint32_t virtual_page = virtual_addr & ~0xfffu;
					uint32_t phys_addr = 0;
					uint32_t ofs = 0;

					/*
					 * A process normally touches several values on one page.  Check
					 * that page before asking whether this is a kernel-linear
					 * address.  This makes both S-mode and U-mode cached RAM use the
					 * same shortest path; the first linear-map access below seeds the
					 * cache for the rest of that page.
					 */
					if (__builtin_expect(
						virtual_page == read_virtual_page, 1)) {
						MINIRV32_PERF_INC(read_page_hits);
						if (__builtin_expect(
							read_ram_offset_bias != UINT32_MAX, 1)) {
							ofs = virtual_addr + read_ram_offset_bias;
							MINIRV32_PERF_INC(read_ram_page_hits);
							goto load_cached_ram;
						}
						phys_addr = read_physical_page |
							(virtual_addr & 0xfffu);
						uncached = read_uncached;
					} else {
						uint32_t linear_offset =
							virtual_addr - MINIRV32_KERNEL_LINEAR_BASE;

						if (linear_offset < kernel_linear_limit) {
							ofs =
								MINIRV32_KERNEL_LINEAR_PHYS_OFFSET +
								linear_offset;
							read_virtual_page = virtual_page;
							read_physical_page =
								MINIRV32_RAM_IMAGE_OFFSET +
								(ofs & ~0xfffu);
							read_uncached = 0;
							read_ram_offset_bias = ofs - virtual_addr;
							MINIRV32_PERF_INC(kernel_linear_hits);
							goto load_cached_ram;
						}

						phys_addr = MiniRV32Translate(
							state, image, virtual_addr, ACCESS_READ, &fault,
							&uncached);
						if (fault) {
							trap = 13 + 1; /* Load page fault. */
							rval = virtual_addr;
							break;
						}
						read_virtual_page = virtual_page;
						read_physical_page = phys_addr & ~0xfffu;
						read_uncached = uncached;
						if (!uncached &&
						    read_physical_page >= MINIRV32_RAM_IMAGE_OFFSET &&
						    read_physical_page - MINIRV32_RAM_IMAGE_OFFSET <=
							MINI_RV32_RAM_SIZE - 0x1000u)
							read_ram_offset_bias = read_physical_page -
								MINIRV32_RAM_IMAGE_OFFSET - virtual_page;
						else
							read_ram_offset_bias = UINT32_MAX;
					}

					if (phys_addr >= MINIRV32_RAM_IMAGE_OFFSET &&
						phys_addr - MINIRV32_RAM_IMAGE_OFFSET <
							MINI_RV32_RAM_SIZE - 3) {
						ofs = phys_addr - MINIRV32_RAM_IMAGE_OFFSET;
							if (!uncached)
								goto load_cached_ram;

							switch (funct3) {
						case 0b000:
							rval = (int8_t)MINIRV32_LOAD1_UNCACHED(ofs);
							break;
						case 0b001:
							rval = (int16_t)MINIRV32_LOAD2_UNCACHED(ofs);
							break;
						case 0b010:
							rval = MINIRV32_LOAD4_UNCACHED(ofs);
							break;
						case 0b100:
							rval = MINIRV32_LOAD1_UNCACHED(ofs);
							break;
						case 0b101:
							rval = MINIRV32_LOAD2_UNCACHED(ofs);
							break;
						default:
							trap = 2 + 1;
							break;
						}
					} else if ((phys_addr >= 0x10000000 &&
						    phys_addr < 0x12000200) ||
						   (phys_addr >= 0x0c000000 &&
						    phys_addr < 0x0c400000) ||
						   (phys_addr >= 0x50000000 &&
						    phys_addr < 0x50040000)) {
						if (phys_addr == 0x1100bffc)
							rval = CSR(timerh);
						else if (phys_addr == 0x1100bff8)
							rval = CSR(timerl);
						else
							MINIRV32_HANDLE_MEM_LOAD_CONTROL(
								phys_addr, rval);
					} else {
						trap = 5 + 1; /* Load access fault. */
						rval = virtual_addr;
					}
					break;

					load_cached_ram:
						/* Linux uses LW for nearly every cached load. Avoid a jump
						 * table and indirect branch on that measured common case. */
						if (__builtin_expect(funct3 == 0b010, 1)) {
							rval = MINIRV32_LOAD4(ofs);
							break;
						}
						switch (funct3) {
					case 0b000:
						rval = (int8_t)MINIRV32_LOAD1(ofs);
						break;
					case 0b001:
						rval = (int16_t)MINIRV32_LOAD2(ofs);
						break;
						case 0b100:
						rval = MINIRV32_LOAD1(ofs);
						break;
					case 0b101:
						rval = MINIRV32_LOAD2(ofs);
						break;
					default:
						trap = 2 + 1;
						break;
					}
					break;
				}
						case 0b01000: // Store
					{
						MINIRV32_PERF_FUNCT3_INC(store_funct3, ir);
						uint32_t funct3 = (ir >> 12) & 7;
					uint32_t rs1 = REG((ir >> 15) & 0x1f);
					uint32_t rs2 = REG((ir >> 20) & 0x1f);
					uint32_t imm =
						((ir >> 7) & 0x1f) |
						((ir & 0xfe000000) >> 20);

					if (imm & 0x800)
						imm |= 0xfffff000;

					uint32_t virtual_addr = rs1 + imm;
					int uncached = 0;
					rdid = 0;
					uint32_t virtual_page = virtual_addr & ~0xfffu;
					uint32_t phys_addr = 0;
					uint32_t ofs = 0;

					if (__builtin_expect(
						virtual_page == write_virtual_page, 1)) {
						MINIRV32_PERF_INC(write_page_hits);
						if (__builtin_expect(
							write_ram_offset_bias != UINT32_MAX, 1)) {
							ofs = virtual_addr + write_ram_offset_bias;
							MINIRV32_PERF_INC(write_ram_page_hits);
							goto store_cached_ram;
						}
						phys_addr = write_physical_page |
							(virtual_addr & 0xfffu);
						uncached = write_uncached;
					} else {
						uint32_t linear_offset =
							virtual_addr - MINIRV32_KERNEL_LINEAR_BASE;

						if (linear_offset < kernel_linear_limit) {
							ofs =
								MINIRV32_KERNEL_LINEAR_PHYS_OFFSET +
								linear_offset;
							write_virtual_page = virtual_page;
							write_physical_page =
								MINIRV32_RAM_IMAGE_OFFSET +
								(ofs & ~0xfffu);
							write_uncached = 0;
							write_ram_offset_bias = ofs - virtual_addr;
							MINIRV32_PERF_INC(kernel_linear_hits);
							goto store_cached_ram;
						}

						phys_addr = MiniRV32Translate(
							state, image, virtual_addr, ACCESS_WRITE, &fault,
							&uncached);
						if (fault) {
							trap = 15 + 1; /* Store page fault. */
							rval = virtual_addr;
							break;
						}
						write_virtual_page = virtual_page;
						write_physical_page = phys_addr & ~0xfffu;
						write_uncached = uncached;
						if (!uncached &&
						    write_physical_page >= MINIRV32_RAM_IMAGE_OFFSET &&
						    write_physical_page - MINIRV32_RAM_IMAGE_OFFSET <=
							MINI_RV32_RAM_SIZE - 0x1000u)
							write_ram_offset_bias = write_physical_page -
								MINIRV32_RAM_IMAGE_OFFSET - virtual_page;
						else
							write_ram_offset_bias = UINT32_MAX;
					}

					if (phys_addr >= MINIRV32_RAM_IMAGE_OFFSET &&
						phys_addr - MINIRV32_RAM_IMAGE_OFFSET <
							MINI_RV32_RAM_SIZE - 3) {
						ofs = phys_addr - MINIRV32_RAM_IMAGE_OFFSET;
							if (!uncached)
								goto store_cached_ram;

							switch (funct3) {
						case 0b000:
							MINIRV32_STORE1_UNCACHED(ofs, rs2);
							break;
						case 0b001:
							MINIRV32_STORE2_UNCACHED(ofs, rs2);
							break;
						case 0b010:
							MINIRV32_STORE4_UNCACHED(ofs, rs2);
							break;
						default:
							trap = 2 + 1;
							break;
						}
					} else if ((phys_addr >= 0x10000000 &&
						    phys_addr < 0x12000200) ||
						   (phys_addr >= 0x0c000000 &&
						    phys_addr < 0x0c400000) ||
						   (phys_addr >= 0x50000000 &&
						    phys_addr < 0x50040000)) {
						if (phys_addr == 0x11004004)
							CSR(timermatchh) = rs2;
						else if (phys_addr == 0x11004000)
							CSR(timermatchl) = rs2;
						else if (phys_addr == 0x11100000) {
							SETCSR(pc, pc + 4);
							return rs2;
						} else {
							MINIRV32_HANDLE_MEM_STORE_CONTROL(
								phys_addr, rs2);
						}
					} else {
						trap = 7 + 1; /* Store access fault. */
						rval = virtual_addr;
					}
					break;

					store_cached_ram:
						/* SW accounts for almost every cached Linux store. */
						if (__builtin_expect(funct3 == 0b010, 1)) {
							MINIRV32_STORE4(ofs, rs2);
						} else if (funct3 == 0b000) {
							MINIRV32_STORE1(ofs, rs2);
						} else if (funct3 == 0b001) {
							MINIRV32_STORE2(ofs, rs2);
						} else {
							trap = 2 + 1;
						}
						break;
				}
					case 0b00100: // Op-immediate
					{
						rdid = (ir >> 7) & 0x1f;
						MINIRV32_PERF_FUNCT3_INC(opimm_funct3, ir);
						uint32_t funct3 = (ir >> 12) & 7;
						uint32_t imm = ir >> 20;
					imm = imm | (( imm & 0x800 )?0xfffff000:0);
					uint32_t rs1 = REG((ir >> 15) & 0x1f);

						/* ADDI is the dominant integer opcode during boot. */
						if (__builtin_expect(funct3 == 0b000, 1)) {
							rval = rs1 + imm;
							break;
						}
						switch (funct3) {
							case 0b001: rval = rs1 << (imm & 0x1f); break; // SLLI
						case 0b010: rval = (int32_t)rs1 < (int32_t)imm; break; // SLTI
						case 0b011: rval = rs1 < imm; break; // SLTIU
						case 0b100: rval = rs1 ^ imm; break; // XORI
						case 0b101: rval = (ir & 0x40000000) ?
							((int32_t)rs1 >> (imm & 0x1f)) :
							(rs1 >> (imm & 0x1f)); break; // SRAI/SRLI
						case 0b110: rval = rs1 | imm; break; // ORI
							case 0b111: rval = rs1 & imm; break; // ANDI
					}
					break;
				}
					case 0b01100: // Op
				{
					rdid = (ir >> 7) & 0x1f;
					MINIRV32_PERF_FUNCT3_INC(op_funct3, ir);
					uint32_t rs1 = REG((ir >> 15) & 0x1f);
					uint32_t rs2 = REG((ir >> 20) & 0x1f);
					uint32_t funct3 = (ir >> 12) & 7u;

					if (__builtin_expect(ir & 0x02000000, 0))
					{
						MINIRV32_PERF_INC(muldiv);
						switch (funct3) //0x02000000 = RV32M
						{
							case 0b000: rval = rs1 * rs2; break; // MUL
							case 0b001: rval = ((int64_t)((int32_t)rs1) * (int64_t)((int32_t)rs2)) >> 32; break; // MULH
							case 0b010: rval = ((int64_t)((int32_t)rs1) * (uint64_t)rs2) >> 32; break; // MULHSU
							case 0b011: rval = ((uint64_t)rs1 * (uint64_t)rs2) >> 32; break; // MULHU
							case 0b100: if( rs2 == 0 ) rval = -1; else rval = ((int32_t)rs1 == INT32_MIN && (int32_t)rs2 == -1) ? rs1 : ((int32_t)rs1 / (int32_t)rs2); break; // DIV
							case 0b101: if( rs2 == 0 ) rval = 0xffffffff; else rval = rs1 / rs2; break; // DIVU
							case 0b110: if( rs2 == 0 ) rval = rs1; else rval = ((int32_t)rs1 == INT32_MIN && (int32_t)rs2 == -1) ? 0 : ((uint32_t)((int32_t)rs1 % (int32_t)rs2)); break; // REM
							case 0b111: if( rs2 == 0 ) rval = rs1; else rval = rs1 % rs2; break; // REMU
						}
					}
					else if (__builtin_expect(funct3 == 0b000, 1))
					{
						/* ADD/SUB is the dominant base-register operation. */
						rval = (ir & 0x40000000) ?
							(rs1 - rs2) : (rs1 + rs2);
					}
					else
					{
						switch (funct3)
						{
							case 0b001: rval = rs1 << (rs2 & 0x1f); break;
							case 0b010: rval = (int32_t)rs1 < (int32_t)rs2; break;
							case 0b011: rval = rs1 < rs2; break;
							case 0b100: rval = rs1 ^ rs2; break;
							case 0b101: rval = (ir & 0x40000000) ? ((int32_t)rs1 >> (rs2 & 0x1f)) : (rs1 >> (rs2 & 0x1f)); break;
							case 0b110: rval = rs1 | rs2; break;
							case 0b111: rval = rs1 & rs2; break;
						}
					}
					break;
				}
					case 0b00011:
					rdid = 0;
					if (((ir >> 12) & 7) == 2 &&
					    ((ir >> 7) & 0x1f) == 0) {
						/* Zicbom CBOs use rs1 as the cache-block address. */
						uint32_t operation = ir >> 20;
						uint32_t virtual_addr =
							REG((ir >> 15) & 0x1f);
						uint32_t phys_addr;

						if (operation > 2) {
							trap = 2 + 1;
							break;
						}

						phys_addr = MiniRV32Translate(
							state, image, virtual_addr, ACCESS_READ, &fault,
							NULL);
						if (fault) {
							trap = 13 + 1;
							rval = virtual_addr;
						} else if (phys_addr >= MINIRV32_RAM_IMAGE_OFFSET &&
							   phys_addr - MINIRV32_RAM_IMAGE_OFFSET <
								MINI_RV32_RAM_SIZE) {
							MINIRV32_HANDLE_CACHE_OP(
								phys_addr, operation);
						} else {
							trap = 5 + 1;
							rval = virtual_addr;
						}
					}
					/* Plain FENCE/FENCE.I need no host action. */
					break;
					case 0b11100: // Zifencei+Zicsr
				{
					rdid = (ir >> 7) & 0x1f;
					uint32_t csrno = ir >> 20;
					int microop = ( ir >> 12 ) & 0b111;
					if( (microop & 3) ) // It's a Zicsr function.
					{
						int rs1imm = (ir >> 15) & 0x1f;
						uint32_t rs1 = REG(rs1imm);
						uint32_t writeval = rs1;

						// https://raw.githubusercontent.com/riscv/virtual-memory/main/specs/663-Svpbmt.pdf
							// Generally, support for Zicsr
							switch( csrno )
							{
							case 0x100: rval = CSR(mstatus); break; /* sstatus */
							case 0x104: rval = CSR(sie); break;
							case 0x105: rval = CSR(stvec); break;
							case 0x140: rval = CSR(sscratch); break;
							case 0x141: rval = CSR(sepc); break;
							case 0x142: rval = CSR(scause); break;
							case 0x143: rval = CSR(stval); break;
							case 0x144: rval = CSR(sip); break;
							case 0x180: rval = CSR(satp); break;
							case 0x340: rval = CSR( mscratch ); break;
							case 0x305: rval = CSR( mtvec ); break;
							case 0x304: rval = CSR( mie ); break;
							case 0xC00:
								rval = cycle + (uint32_t)icount + 1u;
								break;
							case 0xC80: rval = CSR( cycleh ); break;
							case 0xC01:
								#if defined(MINIRV32_HOST_CYCLE_COUNT) && \
								    defined(MINIRV32_HOST_CYCLES_PER_US)
								rval = (uint32_t)(rdtime_timer_anchor +
									(uint32_t)(MINIRV32_HOST_CYCLE_COUNT() -
									 rdtime_cycle_anchor) /
									 MINIRV32_HOST_CYCLES_PER_US);
								#elif defined(MINIRV32_HOST_TIME_US)
								rval = (uint32_t)(rdtime_timer_anchor +
									(MINIRV32_HOST_TIME_US() -
									 rdtime_host_anchor));
								#else
								rval = CSR(timerl);
								#endif
								break;
							case 0xC81:
								#if defined(MINIRV32_HOST_CYCLE_COUNT) && \
								    defined(MINIRV32_HOST_CYCLES_PER_US)
								rval = (uint32_t)((rdtime_timer_anchor +
									(uint32_t)(MINIRV32_HOST_CYCLE_COUNT() -
									 rdtime_cycle_anchor) /
									 MINIRV32_HOST_CYCLES_PER_US) >> 32);
								#elif defined(MINIRV32_HOST_TIME_US)
								rval = (uint32_t)((rdtime_timer_anchor +
									(MINIRV32_HOST_TIME_US() -
									 rdtime_host_anchor)) >> 32);
								#else
								rval = CSR(timerh);
								#endif
								break;
							case 0x344: rval = CSR( mip ); break;
						case 0x341: rval = CSR( mepc ); break;
						case 0x300: rval = CSR( mstatus ); break; //mstatus
						case 0x342: rval = CSR( mcause ); break;
						case 0x343: rval = CSR( mtval ); break;
						case 0xf11: rval = 0xff0ff0ff; break; //mvendorid
						case 0x301: rval = 0x40401101; break; //misa (XLEN=32, IMA+X)
						//case 0x3B0: rval = 0; break; //pmpaddr0
						//case 0x3a0: rval = 0; break; //pmpcfg0
						//case 0xf12: rval = 0x00000000; break; //marchid
						//case 0xf13: rval = 0x00000000; break; //mimpid
						//case 0xf14: rval = 0x00000000; break; //mhartid
						default:
							MINIRV32_OTHERCSR_READ( csrno, rval );
							break;
						}

						switch( microop )
						{
							case 0b001: writeval = rs1; break;  			//CSRRW
							case 0b010: writeval = rval | rs1; break;		//CSRRS
							case 0b011: writeval = rval & ~rs1; break;		//CSRRC
							case 0b101: writeval = rs1imm; break;			//CSRRWI
							case 0b110: writeval = rval | rs1imm; break;	//CSRRSI
							case 0b111: writeval = rval & ~rs1imm; break;	//CSRRCI
						}

							switch( csrno )
							{
							case 0x100:
								SETCSR(mstatus, writeval);
								read_virtual_page = UINT32_MAX;
								write_virtual_page = UINT32_MAX;
								if (MiniRV32SupervisorInterruptCause(state))
									count = icount + 1;
								break;
							case 0x104:
								SETCSR(sie, writeval);
								if (MiniRV32SupervisorInterruptCause(state))
									count = icount + 1;
								break;
							case 0x105: SETCSR(stvec, writeval); break;
							case 0x140: SETCSR(sscratch, writeval); break;
							case 0x141: SETCSR(sepc, writeval); break;
							case 0x142: SETCSR(scause, writeval); break;
							case 0x143: SETCSR(stval, writeval); break;
							case 0x144:
								SETCSR(sip, writeval);
								if (MiniRV32SupervisorInterruptCause(state))
									count = icount + 1;
								break;
							case 0x180:
								MINIRV32_PERF_INC(satp_writes);
								if (writeval != CSR(satp)) {
									SETCSR(satp, writeval);
									kernel_linear_limit =
										(((CSR(extraflags) & 3u) == 1u) &&
										 (writeval & 0x80000000u)) ?
											(MINI_RV32_RAM_SIZE -
											 MINIRV32_KERNEL_LINEAR_PHYS_OFFSET) : 0u;
									MiniRV32FlushTLB(state);
									exec_virtual_page = UINT32_MAX;
									read_virtual_page = UINT32_MAX;
									write_virtual_page = UINT32_MAX;
								}
								break;
							case 0x340: SETCSR( mscratch, writeval ); break;
						case 0x305: SETCSR( mtvec, writeval ); break;
						case 0x304: SETCSR( mie, writeval ); break;
						case 0x344: SETCSR( mip, writeval ); break;
						case 0x341: SETCSR( mepc, writeval ); break;
						case 0x300:
							SETCSR(mstatus, writeval);
							read_virtual_page = UINT32_MAX;
							write_virtual_page = UINT32_MAX;
							break; //mstatus
						case 0x342: SETCSR( mcause, writeval ); break;
						case 0x343: SETCSR( mtval, writeval ); break;
						//case 0x3a0: break; //pmpcfg0
						//case 0x3B0: break; //pmpaddr0
						//case 0xf11: break; //mvendorid
						//case 0xf12: break; //marchid
						//case 0xf13: break; //mimpid
						//case 0xf14: break; //mhartid
						//case 0x301: break; //misa
						default:
							MINIRV32_OTHERCSR_WRITE( csrno, writeval );
							break;
						}
					}
					else if( microop == 0b000 ) // "SYSTEM"
					{
						rdid = 0;

						if ((ir & 0xfe007fffu) == 0x12000073u) {
							uint32_t rs1id = (ir >> 15) & 0x1f;
							uint32_t rs2id = (ir >> 20) & 0x1f;
							uint32_t current_asid =
								(CSR(satp) >> 22) & 0x1ffu;
							uint32_t requested_asid =
								REG(rs2id) & 0x1ffu;

							MINIRV32_PERF_INC(sfence_vma);
							if (rs1id == 0) {
								/* x0 selects every virtual address.  With the
								 * current single-address-space cache, a fence for
								 * another ASID cannot match any resident entry. */
								if (rs2id == 0 || requested_asid == current_asid) {
									MINIRV32_PERF_INC(sfence_global);
									MiniRV32FlushTLB(state);
									exec_virtual_page = UINT32_MAX;
									read_virtual_page = UINT32_MAX;
									write_virtual_page = UINT32_MAX;
								}
							} else if (rs2id == 0 ||
								   requested_asid == current_asid) {
								uint32_t page = REG(rs1id) & ~0xfffu;

								MINIRV32_PERF_INC(sfence_page);
								MiniRV32FlushTLBPage(state, page);
								if (exec_virtual_page == page)
									exec_virtual_page = UINT32_MAX;
								if (read_virtual_page == page)
									read_virtual_page = UINT32_MAX;
								if (write_virtual_page == page)
									write_virtual_page = UINT32_MAX;
							}
						} else if (csrno == 0x105) { /* WFI */
							CSR(extraflags) |= 4;
							SETCSR(pc, pc + 4);
							return 1;
						} else if (csrno == 0x102) { /* SRET */
							uint32_t status = CSR(mstatus);
							uint32_t new_priv =
								(status & (1u << 8)) ? 1u : 0u;

							/* SIE <- SPIE. */
							if (status & (1u << 5))
								status |= (1u << 1);
							else
								status &= ~(1u << 1);

							status |= (1u << 5);  /* SPIE <- 1. */
							status &= ~(1u << 8); /* SPP <- U. */
							SETCSR(mstatus, status);

							CSR(extraflags) =
								(CSR(extraflags) & ~3u) | new_priv;
							kernel_linear_limit =
								(new_priv == 1u &&
								 (CSR(satp) & 0x80000000u)) ?
									(MINI_RV32_RAM_SIZE -
									 MINIRV32_KERNEL_LINEAR_PHYS_OFFSET) : 0u;
							exec_virtual_page = UINT32_MAX;
							read_virtual_page = UINT32_MAX;
							write_virtual_page = UINT32_MAX;
							pc = CSR(sepc) - 4;
							if (MiniRV32SupervisorInterruptCause(state))
								count = icount + 1;
						} else if ((csrno & 0xff) == 0x02) { /* MRET */
							uint32_t startmstatus = CSR(mstatus);
							uint32_t startextraflags = CSR(extraflags);

							SETCSR(mstatus,
								((startmstatus & 0x80) >> 4) |
								((startextraflags & 3) << 11) |
								0x80);
							SETCSR(extraflags,
								(startextraflags & ~3) |
								((startmstatus >> 11) & 3));
							kernel_linear_limit =
								((((startmstatus >> 11) & 3u) == 1u) &&
								 (CSR(satp) & 0x80000000u)) ?
									(MINI_RV32_RAM_SIZE -
									 MINIRV32_KERNEL_LINEAR_PHYS_OFFSET) : 0u;
							exec_virtual_page = UINT32_MAX;
							read_virtual_page = UINT32_MAX;
							write_virtual_page = UINT32_MAX;
							pc = CSR(mepc) - 4;
						} else {
							switch (csrno) {
							case 0:
								switch (CSR(extraflags) & 3) {
								case 0:
									trap = 8 + 1;  /* U-mode ECALL. */
									break;
								case 1:
									trap = 9 + 1;  /* S-mode ECALL/SBI. */
									break;
								case 3:
									trap = 11 + 1; /* M-mode ECALL. */
									break;
								default:
									trap = 2 + 1;
									break;
								}
								break;
							case 1:
								trap = 3 + 1; /* EBREAK. */
								break;
							default:
								trap = 2 + 1;
								break;
							}
						}
					}
					else
						trap = (2+1); 				// Note micrrop 0b100 == undefined.
					break;
				}
					case 0b01011: // RV32A
				{
					rdid = (ir >> 7) & 0x1f;
					uint32_t virtual_addr = REG((ir >> 15) & 0x1f);
					uint32_t rs2 = REG((ir >> 20) & 0x1f);
					uint32_t irmid = (ir >> 27) & 0x1f;
					int access =
						(irmid == 0b00010) ? ACCESS_READ : ACCESS_WRITE;
					int uncached = 0;
					uint32_t phys_addr;
					uint32_t linear_offset =
						virtual_addr - MINIRV32_KERNEL_LINEAR_BASE;

					/*
					 * Kernel locks and refcounts make AMOs a hot path during
					 * boot.  They use the same fixed linear mapping as ordinary
					 * kernel loads/stores, so avoid a TLB lookup for that case.
					 */
					if (__builtin_expect(
						linear_offset < kernel_linear_limit,
						1)) {
						phys_addr = MINIRV32_RAM_IMAGE_OFFSET +
							MINIRV32_KERNEL_LINEAR_PHYS_OFFSET +
							linear_offset;
					} else {
						phys_addr = MiniRV32Translate(
							state, image, virtual_addr, access, &fault,
							&uncached);
						if (fault) {
							trap =
								((access == ACCESS_READ) ? 13 : 15) + 1;
							rval = virtual_addr;
							break;
						}
					}

					if (phys_addr < MINIRV32_RAM_IMAGE_OFFSET ||
						phys_addr - MINIRV32_RAM_IMAGE_OFFSET >=
							MINI_RV32_RAM_SIZE - 3) {
						trap =
							((access == ACCESS_READ) ? 5 : 7) + 1;
						rval = virtual_addr;
						break;
					}

					uint32_t ofs =
						phys_addr - MINIRV32_RAM_IMAGE_OFFSET;

					rval = uncached ? MINIRV32_LOAD4_UNCACHED(ofs) :
						MINIRV32_LOAD4(ofs);

					uint32_t dowrite = 1;

					switch (irmid) {
					case 0b00010: /* LR.W */
						dowrite = 0;
						CSR(extraflags) =
							(CSR(extraflags) & 0b111) | (ofs << 3);
						break;
					case 0b00011: /* SC.W */
						rval =
							(CSR(extraflags) >> 3 !=
							 (ofs & 0x1fffffff));
						dowrite = !rval;
						break;
					case 0b00001: /* AMOSWAP.W */
						break;
					case 0b00000: /* AMOADD.W */
						rs2 += rval;
						break;
					case 0b00100: /* AMOXOR.W */
						rs2 ^= rval;
						break;
					case 0b01100: /* AMOAND.W */
						rs2 &= rval;
						break;
					case 0b01000: /* AMOOR.W */
						rs2 |= rval;
						break;
					case 0b10000: /* AMOMIN.W */
						rs2 =
							((int32_t)rs2 < (int32_t)rval) ? rs2 : rval;
						break;
					case 0b10100: /* AMOMAX.W */
						rs2 =
							((int32_t)rs2 > (int32_t)rval) ? rs2 : rval;
						break;
					case 0b11000: /* AMOMINU.W */
						rs2 = (rs2 < rval) ? rs2 : rval;
						break;
					case 0b11100: /* AMOMAXU.W */
						rs2 = (rs2 > rval) ? rs2 : rval;
						break;
					default:
						trap = 2 + 1;
						dowrite = 0;
						break;
					}

					if (dowrite) {
						if (uncached)
							MINIRV32_STORE4_UNCACHED(ofs, rs2);
						else
							MINIRV32_STORE4(ofs, rs2);
					}
					break;
				}
					/* The switch key is instruction bits [6:2], so these are all
					 * remaining entries in its 0..31 range.  Listing them makes the
					 * dispatch exhaustive and lets the compiler omit a redundant
					 * bounds branch before the jump table. */
					case 0b00001:
					case 0b00010:
					case 0b00110:
					case 0b00111:
					case 0b01001:
					case 0b01010:
					case 0b01110:
					case 0b01111:
					case 0b10000:
					case 0b10001:
					case 0b10010:
					case 0b10011:
					case 0b10100:
					case 0b10101:
					case 0b10110:
					case 0b10111:
					case 0b11010:
					case 0b11101:
					case 0b11110:
					case 0b11111:
						trap = (2+1); // Fault: invalid or unsupported opcode.
						break;
			}

			// If there was a trap, do NOT allow register writeback.
			if( trap )
				break;

			if( rdid )
			{
				REGSET( rdid, rval ); // Write back register.
			}
		}

		MINIRV32_POSTEXEC( pc, ir, trap );

		pc += 4;
	}

	/* Handle traps and interrupts. */
	if (trap)
	{
		uint32_t old_priv = CSR(extraflags) & 3;
		uint32_t is_interrupt = !!(trap & 0x80000000u);
		uint32_t cause =
			is_interrupt ? (trap & 0x7fffffffu) : (trap - 1);
		uint32_t encoded_cause =
			is_interrupt ? (0x80000000u | cause) : cause;

		/* Interrupt setup backed pc up by four; recover the interrupted PC. */
		uint32_t trap_pc = is_interrupt ? pc + 4 : pc;

		/* Emulate S-mode SBI calls directly instead of entering OpenSBI. */
		if (!is_interrupt && cause == 9 && old_priv == 1) {
			MiniRV32HandleSBI(state);
			pc = trap_pc + 4;
			trap = 0;
		} else if (old_priv <= 1) {
			/* Model the delegation Linux expects and enter S-mode. */
			SETCSR(scause, encoded_cause);
			SETCSR(sepc, trap_pc);

			if (is_interrupt) {
				SETCSR(stval, 0);
			} else {
				switch (cause) {
				case 0:
				case 1:
				case 12:
					SETCSR(stval, rval ? rval : trap_pc);
					break;
				case 4:
				case 5:
				case 6:
				case 7:
				case 13:
				case 15:
					SETCSR(stval, rval);
					break;
				default:
					SETCSR(stval, 0);
					break;
				}
			}

			/* SPIE <- SIE, SIE <- 0, SPP <- previous privilege. */
			uint32_t status = CSR(mstatus);

			if (status & (1u << 1))
				status |= (1u << 5);
			else
				status &= ~(1u << 5);

			status &= ~(1u << 1);

			if (old_priv == 1)
				status |= (1u << 8);
			else
				status &= ~(1u << 8);

			SETCSR(mstatus, status);
			CSR(extraflags) =
				(CSR(extraflags) & ~3u) | 1u;

			uint32_t stvec = CSR(stvec);
			uint32_t base = stvec & ~3u;
			uint32_t mode = stvec & 3u;

			if (is_interrupt && mode == 1)
				pc = base + (cause * 4);
			else
				pc = base;

			trap = 0;
		} else {
			/* Retain M-mode trap behavior for future M-mode use. */
			SETCSR(mcause, encoded_cause);
			SETCSR(mepc, trap_pc);

			if (is_interrupt) {
				SETCSR(mtval, 0);
			} else {
				SETCSR(mtval, rval);
			}

			/* MPIE <- MIE, MIE <- 0, MPP <- previous privilege. */
			uint32_t status = CSR(mstatus);

			if (status & (1u << 3))
				status |= (1u << 7);
			else
				status &= ~(1u << 7);

			status &= ~(1u << 3);
			status &= ~(3u << 11);
			status |= (old_priv & 3u) << 11;

			SETCSR(mstatus, status);
			CSR(extraflags) =
				(CSR(extraflags) & ~3u) | 3u;

			uint32_t mtvec = CSR(mtvec);
			uint32_t base = mtvec & ~3u;
			uint32_t mode = mtvec & 3u;

			if (is_interrupt && mode == 1)
				pc = base + (cause * 4);
			else
				pc = base;

			trap = 0;
		}
	}

	/* The loop index already advances once per instruction.  Reuse it for the
	 * architectural cycle count instead of performing a second hot-loop add.
	 * A trap breaks before the for-loop increment, so count that attempt here. */
	uint32_t executed = (uint32_t)icount;
	if (icount < count)
		executed++;
	uint32_t new_cycle = cycle + executed;
	if (new_cycle < cycle)
		CSR(cycleh)++;
	SETCSR(cyclel, new_cycle);
	SETCSR( pc, pc );
	return 0;
}

#endif

#endif
