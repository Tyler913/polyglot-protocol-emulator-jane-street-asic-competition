/*
 * Copyright (c) 2026 Tyler Hong
 * SPDX-License-Identifier: Apache-2.0
 *
 * busy: keeps the single SRAM port 100 % busy for long stretches (256 loads in
 * a row: every instruction needs a fetch slot and a load slot), so
 * test/test_boot.py can check that host accesses still get through while the
 * CPU runs. PG_SHARED[0] counts loop iterations; the host uses 0x800-0x8FF,
 * which this program never touches.
 */

#include "protocol.h"

static volatile uint32_t buf[4];

int main(void)
{
    uint32_t i = 0;
    for (;;) {
        __asm__ volatile(".rept 256\n lw a5, 0(%0)\n .endr" : : "r"(buf) : "a5", "memory");
        PG_SHARED[0] = ++i;
    }
}
