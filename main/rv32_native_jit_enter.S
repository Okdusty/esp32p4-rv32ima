/*
 * One ABI boundary surrounds an entire chain of generated RV32 regions.
 * Generated code is a leaf with respect to host C, but it deliberately uses
 * every saved integer register for guest values and page translations.  Save
 * those registers once here instead of in every short generated region.
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
    .section .note.GNU-stack,"",@progbits
