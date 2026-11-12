/*
 * Copyright (c) 2026 Tyler Hong
 * SPDX-License-Identifier: Apache-2.0
 *
 * memtest: exercise byte, halfword and word stores and loads (including
 * store-then-load back to back) and report the result in PG_SHARED (0x0F00).
 * Used by test/test_boot.py. The host halts the CPU and reads PG_SHARED.
 */

#include "protocol.h"

#define RESULT PG_SHARED

static uint8_t buf[64];

int main(void)
{
    volatile uint8_t  *b = buf;
    volatile uint16_t *h = (volatile uint16_t *)buf;
    volatile uint32_t *w = (volatile uint32_t *)buf;
    uint32_t errors = 0;

    for (int i = 0; i < 64; i++) b[i] = (uint8_t)(i * 7 + 3);
    for (int i = 0; i < 64; i++) if (b[i] != (uint8_t)(i * 7 + 3)) errors++;

    for (int i = 0; i < 32; i++) h[i] = (uint16_t)(0xA500 + i);
    for (int i = 0; i < 64; i++) {
        uint8_t expect = (i & 1) ? 0xA5 : (uint8_t)(i / 2);
        if (b[i] != expect) errors++;
    }

    for (int i = 0; i < 16; i++) { w[i] = 0x01020304u * (uint32_t)i; if (w[i] != 0x01020304u * (uint32_t)i) errors++; }
    for (int i = 0; i < 16; i++) { b[4 * i + 1] = 0xEE; if (w[i] != ((0x01020304u * (uint32_t)i) & ~0x0000FF00u) + 0xEE00u) errors++; }

    RESULT[1] = errors;
    RESULT[0] = 0x600D0000u | (errors ? 0xBAD : 0x0);
    for (;;) { }
}
