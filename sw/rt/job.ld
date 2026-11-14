/*
 * Copyright (c) 2026 Tyler Hong
 * SPDX-License-Identifier: Apache-2.0
 *
 * Linker script for a queue job (sw/jobs/<name>): one image per slot, linked
 * at SLOT_BASE (passed with --defsym). The entry point job_entry comes first.
 * Zero-initialized data is placed in the image as zeros, so loading the image
 * over the host port also clears it.
 */

OUTPUT_ARCH(riscv)
ENTRY(job_entry)

SECTIONS
{
    . = SLOT_BASE;
    __global_pointer$ = SLOT_BASE + 0x400;   /* gp-relative reach covers the whole slot */
    .text : {
        KEEP(*(.text.job_entry))
        *(.text .text.*)
    }
    .rodata : ALIGN(4) {
        *(.rodata .rodata.* .srodata .srodata.*)
    }
    .data : ALIGN(4) {
        *(.data .data.* .sdata .sdata.*)
    }
    .bss : ALIGN(4) {
        *(.bss .bss.* .sbss .sbss.* COMMON)
        . = ALIGN(4);
    }
    /* A loaded word after .bss: objcopy -O binary fills the gap up to it with
     * zeros, so the image carries the cleared .bss. */
    .jobend : {
        LONG(0x214A4F42)   /* "BOJ!" */
    }
    __job_end = .;
    ASSERT(__job_end <= SLOT_BASE + SLOT_SIZE, "job too large for its slot")

    /DISCARD/ : { *(.comment) *(.riscv.attributes) }
}
