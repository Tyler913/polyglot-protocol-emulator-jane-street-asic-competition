/*
 * Copyright (c) 2026 Tyler Hong
 * SPDX-License-Identifier: Apache-2.0
 *
 * Startup code. The host loads the whole image into SRAM, so there is no
 * .data copy; only .bss is cleared.
 */

    .section .text.start
    .global _start
_start:
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
