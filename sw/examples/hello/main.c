/*
 * Copyright (c) 2026 Tyler Hong
 * SPDX-License-Identifier: Apache-2.0
 *
 * hello: the first program for a new board. Toggles HOST_IRQ (uo[6]; LED2
 * on the FPGA prototype) and counts the toggles in PG_SHARED[1], which the
 * host can read while the program runs.
 *
 * Host parameters (PG_SHARED): [0] half period in microseconds (0 = 500 ms).
 */

#include "protocol.h"

int main(void)
{
    uint32_t half = PG_SHARED[0] ? PG_SHARED[0] : 500000u;
    uint32_t ticks = PG_US_TO_TICKS(half);
    uint32_t t = now();

    for (uint32_t n = 1;; n++) {
        t += ticks;
        wait_until(t);
        host_irq(n & 1u);
        PG_SHARED[1] = n;
    }
}
