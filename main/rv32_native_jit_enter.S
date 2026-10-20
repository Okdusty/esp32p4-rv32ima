/*
 * One ABI boundary surrounds a whole chain of generated regions, which use
 * every callee-saved register for guest values and page translations.
 */

    .section .iram1,"ax"
    .align  2
    .global rv32_native_jit_enter
    .type   rv32_native_jit_enter, @function

/* uint32_t rv32_native_jit_enter(void *code, struct runtime *runtime); */
rv32_native_jit_enter:
    addi    sp, sp, -64
    sw      ra,  0(sp)
    sw      s0,  4(sp)
    sw      s1,  8(sp)
    sw      s2, 12(sp)
    sw      s3, 16(sp)
    sw      s4, 20(sp)
    sw      s5, 24(sp)
    sw      s6, 28(sp)
    sw      s7, 32(sp)
    sw      s8, 36(sp)
    sw      s9, 40(sp)
    sw      s10, 44(sp)
    sw      s11, 48(sp)

    mv      t0, a0
    mv      a0, a1
    jalr    ra, t0, 0

    lw      ra,  0(sp)
    lw      s0,  4(sp)
    lw      s1,  8(sp)
    lw      s2, 12(sp)
    lw      s3, 16(sp)
    lw      s4, 20(sp)
    lw      s5, 24(sp)
    lw      s6, 28(sp)
    lw      s7, 32(sp)
    lw      s8, 36(sp)
    lw      s9, 40(sp)
    lw      s10, 44(sp)
    lw      s11, 48(sp)
    addi    sp, sp, 64
    ret

    .size   rv32_native_jit_enter, .-rv32_native_jit_enter

/* Native dispatch for a region's normal exit.
 *   a1 = next guest pc, a5 = this region's slot, t1 = its link's expected pc
 *   (UINT32_MAX when empty), t3 = miss return address.
 * The jump cache answers targets seen before under the current translations;
 * otherwise, once a fresh region's first exits have gone to C (the learning
 * budget), same-page targets use the region's page, other pages the
 * validated exec-page cache, and a hit is added to the jump cache.  A target
 * found for a region with an empty link becomes its link (never the region
 * itself, as in jit_observe_link), so links return after the translation
 * flushes that clear them.  Targets check the
 * instruction budget at their own chain entry.
 * Keeps a1, a6, a7, t5, t6, s10, s11; s9 is free between regions.  On a hit
 * it tail-jumps to the target's native chain entry exactly like a confirmed
 * link; on a miss it returns to the region, which then publishes its result
 * to the C dispatcher.  The lookup, wake, hot-code and promotion-counter rules mirror
 * rv32_native_jit_try and jit_dispatch_code; the table layout is defined in
 * rv32_native_jit.c next to struct rv32_native_jit_dispatch_table. */
    .align  2
    .global rv32_native_jit_dispatch
    .type   rv32_native_jit_dispatch, @function
rv32_native_jit_dispatch:
    la      a3, rv32_native_jit_dispatch_table
    srli    t0, a1, 2                   /* jump cache index */
    srli    a2, a1, 10
    xor     t0, t0, a2
    andi    t0, t0, 255
    slli    t0, t0, 3
    lw      a4, 64(a3)                  /* this privilege's jump cache */
    add     a4, a4, t0
    lw      a2, 0(a4)
    bne     a2, a1, 13f
    lw      t2, 4(a4)                   /* target code */
    lw      a2, 16(a3)                  /* host wake word */
    lw      a2, 0(a2)
    bnez    a2, 9f
    addi    a2, t1, 1
    bnez    a2, 18f                     /* this region's link is in use */
    lw      a2, 0(a3)                   /* entries */
    slli    a4, a5, 4
    add     a2, a2, a4
    lw      a2, 4(a2)                   /* this region's guest pc */
    beq     a2, a1, 18f                 /* a loop keeps its wake checks */
    lw      a2, 68(a3)                  /* links */
    slli    a4, a5, 3
    add     a2, a2, a4
    sw      t2, 0(a2)
    sw      a1, 4(a2)
18:
    jalr    x0, 20(t2)                  /* native chain entry */
13:
    lw      a2, 52(a3)                  /* link learning budget */
    add     a2, a2, a5
    lbu     a4, 0(a2)
    beqz    a4, 10f
    addi    a4, a4, -1                  /* let C observe this exit */
    sb      a4, 0(a2)
    j       9f
10:
    mv      a0, a5                      /* this region's slot */
    lw      a4, 0(a3)                   /* entries */
    slli    a2, a5, 4
    add     a4, a4, a2                  /* this region's entry */
    lw      s9, 0(a4)                   /* physical offset */
    lw      t4, 4(a4)                   /* guest pc */
    srli    a2, t4, 12
    slli    a2, a2, 12
    sub     t0, t4, a2                  /* page offset */
    sub     s9, s9, t0                  /* physical page offset */
    mv      t4, a2                      /* virtual page */
    srli    t0, a1, 12
    slli    t0, t0, 12
    beq     t0, t4, 11f                 /* same page as this region */
    /* Another page: only a mapping the Sv32 execute path already validated
     * in this memory context may be chained, exactly as jit_resolve_exec_offset. */
    srli    t0, a1, 12                  /* virtual page */
    srli    a2, t0, 8
    xor     a2, a2, t0
    andi    a2, a2, 63
    slli    a2, a2, 3
    lw      a4, 56(a3)                  /* exec pages */
    add     a4, a4, a2
    lw      a2, 60(a3)                  /* privilege tag */
    lw      a2, 0(a2)
    or      a2, a2, t0
    lw      t2, 0(a4)
    bne     t2, a2, 9f                  /* not validated for this context */
    lw      t2, 4(a4)                   /* physical page offset */
    slli    t0, t0, 12
    xor     t0, a1, t0                  /* page offset */
    add     t2, t2, t0                  /* physical offset */
    j       12f
11:
    xor     t0, a1, t4                  /* page offset */
    add     t2, s9, t0                  /* physical offset */
12:
    srli    t0, t2, 2
    srli    a2, t2, 9
    xor     t0, t0, a2
    srli    a2, a1, 12
    xor     t0, t0, a2
    srli    a2, a1, 20
    xor     t0, t0, a2
    lw      a2, 28(a3)                  /* set mask */
    and     t0, t0, a2
    lw      a4, 0(a3)                   /* entries */
    slli    t0, t0, 6                   /* set * 4 ways * 16 bytes */
    add     a4, a4, t0
    addi    t0, a4, 64                  /* end of the set */
1:
    lw      a2, 0(a4)
    bne     a2, t2, 3f
    lw      a2, 4(a4)
    bne     a2, a1, 3f
    lbu     a2, 9(a4)
    addi    a2, a2, -1                  /* JIT_ENTRY_READY */
    beqz    a2, 2f
3:
    addi    a4, a4, 16
    bne     a4, t0, 1b
    sw      a1, 92(t6)                  /* runtime->dispatch_miss_pc */
    jr      t3
2:
    lw      a2, 16(a3)                  /* host wake word */
    lw      a2, 0(a2)
    bnez    a2, 9f
    lbu     a5, 8(a4)                   /* guest_instruction_count */
    sub     t0, s11, s10                /* remaining budget */
    bltu    t0, a5, 9f
    lw      a2, 0(a3)
    sub     a5, a4, a2
    srli    a5, a5, 4                   /* slot */
    lw      a2, 72(a3)                  /* link targets: below may publish it */
    add     a2, a2, a5
    li      t0, 1
    sb      t0, 0(a2)
    lw      a2, 4(a3)                   /* hot entries */
    slli    t0, a5, 2
    add     a2, a2, t0
    lhu     t0, 0(a2)                   /* location */
    beqz    t0, 5f
    andi    t0, t0, 0x3ff
    addi    t0, t0, -1
    slli    t0, t0, 7
    lw      a5, 12(a3)
    add     t0, t0, a5
    j       8f
5:
    lw      a5, 12(a3)
    beqz    a5, 6f                      /* no hot arena: never count */
    lhu     t0, 2(a2)                   /* calls */
    li      a5, 0xffff
    beq     t0, a5, 6f                  /* saturated */
    addi    t0, t0, 1
    lbu     a5, 11(a4)
    beqz    a5, 7f
    lw      a5, 36(a3)                  /* loop admit */
    beq     t0, a5, 9f                  /* C promotes at the threshold */
    lw      a5, 44(a3)                  /* loop replace */
    beq     t0, a5, 9f
    j       71f
7:
    lw      a5, 40(a3)                  /* general admit */
    beq     t0, a5, 9f
    lw      a5, 48(a3)                  /* general replace */
    beq     t0, a5, 9f
71:
    sh      t0, 2(a2)
    lhu     t0, 12(a4)                  /* code_unit */
    slli    t0, t0, 7
    lw      a5, 8(a3)
    add     t0, t0, a5
    j       17f                         /* counting: keep coming here */
6:
    lhu     t0, 12(a4)                  /* code_unit */
    slli    t0, t0, 7
    lw      a5, 8(a3)
    add     t0, t0, a5
8:
    srli    a5, a1, 2                   /* remember the target */
    srli    a2, a1, 10
    xor     a5, a5, a2
    andi    a5, a5, 255
    slli    a5, a5, 3
    lw      a2, 64(a3)
    add     a2, a2, a5
    sw      a1, 0(a2)
    sw      t0, 4(a2)
    addi    a2, t1, 1
    bnez    a2, 17f                     /* this region's link is in use */
    lw      a2, 0(a3)                   /* entries */
    slli    a0, a0, 4
    add     a2, a2, a0
    lw      a4, 4(a2)                   /* this region's guest pc */
    beq     a4, a1, 17f                 /* a loop keeps its wake checks */
    srli    a0, a0, 1                   /* slot * 8 */
    lw      a2, 68(a3)                  /* links */
    add     a2, a2, a0
    sw      t0, 0(a2)
    sw      a1, 4(a2)
17:
    lw      a2, 20(a3)
    addi    a2, a2, 1
    sw      a2, 20(a3)                  /* fast dispatch counter */
    jalr    x0, 20(t0)                  /* native chain entry */
9:
    jr      t3
    .size   rv32_native_jit_dispatch, .-rv32_native_jit_dispatch

/* Cold RAM mapping refill: t4 = link, t6 = runtime, a1 = guest address,
 * a0 = access.  Preserves every caller-saved register except t2, which
 * returns success; the C callee preserves s0..s11. */
    .align 2
    .global rv32_native_jit_refill_trampoline
    .type rv32_native_jit_refill_trampoline, @function
rv32_native_jit_refill_trampoline:
    addi sp, sp, -64
    sw ra, 0(sp)
    sw t0, 4(sp)
    sw t1, 8(sp)
    sw t3, 12(sp)
    sw t4, 16(sp)
    sw t5, 20(sp)
    sw t6, 24(sp)
    sw a0, 28(sp)
    sw a1, 32(sp)
    sw a2, 36(sp)
    sw a3, 40(sp)
    sw a4, 44(sp)
    sw a5, 48(sp)
    sw a6, 52(sp)
    sw a7, 56(sp)
    mv a2, a0
    mv a0, t6
    call rv32_native_jit_refill_ram_page
    mv t2, a0
    lw ra, 0(sp)
    lw t0, 4(sp)
    lw t1, 8(sp)
    lw t3, 12(sp)
    lw t4, 16(sp)
    lw t5, 20(sp)
    lw t6, 24(sp)
    lw a0, 28(sp)
    lw a1, 32(sp)
    lw a2, 36(sp)
    lw a3, 40(sp)
    lw a4, 44(sp)
    lw a5, 48(sp)
    lw a6, 52(sp)
    lw a7, 56(sp)
    addi sp, sp, 64
    jr t4
    .size rv32_native_jit_refill_trampoline, .-rv32_native_jit_refill_trampoline
    .section .note.GNU-stack,"",@progbits
