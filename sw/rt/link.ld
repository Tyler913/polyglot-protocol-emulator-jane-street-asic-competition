/*
 * Copyright (c) 2026 Tyler Hong
 * SPDX-License-Identifier: Apache-2.0
 *
 * Linker script: the 12 KB SRAM holds code, data, bss and the stack.
 * The last 256 bytes (0x2F00-0x2FFF) are shared with the host (PG_SHARED in
 * protocol.h); the stack grows down from 0x2F00.
 */

OUTPUT_ARCH(riscv)
ENTRY(_start)

MEMORY
{
    RAM (rwx) : ORIGIN = 0x00000000, LENGTH = 12K
}

SECTIONS
{
    .text : {
        KEEP(*(.text.start))
        *(.text .text.*)
    } > RAM

    .rodata : ALIGN(4) {
        *(.rodata .rodata.* .srodata .srodata.*)
    } > RAM

    .data : ALIGN(4) {
        *(.data .data.* .sdata .sdata.*)
    } > RAM

    .bss (NOLOAD) : ALIGN(4) {
        __bss_start = .;
        *(.bss .bss.* .sbss .sbss.* COMMON)
        . = ALIGN(4);
        __bss_end = .;
    } > RAM

    __stack_top = ORIGIN(RAM) + LENGTH(RAM) - 256;
    ASSERT(__bss_end <= __stack_top - 256, "program too large: less than 256 bytes left for the stack")

    /DISCARD/ : { *(.comment) *(.riscv.attributes) }
}
