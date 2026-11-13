/*
 * Copyright (c) 2026 Tyler Hong
 * SPDX-License-Identifier: Apache-2.0
 *
 * Startup code. The host loads the whole image into SRAM, so there is no
 * .data copy; only .bss is cleared.
 *
 * All registers start from zero (22 bytes): the register file has no reset,
 * so otherwise a register the program never writes holds a random value.
 * That is harmless on silicon, but a gate-level simulation sees X there,
 * and a speculative read of it can spread X into the pipeline.
 */

    .section .text.start
    .global _start
_start:
    li    gp, 0
    li    tp, 0
    li    t2, 0
    li    s0, 0
    li    s1, 0
    li    a0, 0
    li    a1, 0
    li    a2, 0
    li    a3, 0
    li    a4, 0
    li    a5, 0
    la    sp, __stack_top
    la    t0, __bss_start
    la    t1, __bss_end
1:  bgeu  t0, t1, 2f
    sw    zero, 0(t0)
    addi  t0, t0, 4
    j     1b
2:  call  main
    /* main returned: park the CPU */
3:  j     3b
