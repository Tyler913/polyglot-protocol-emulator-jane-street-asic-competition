/*
 * Copyright (c) 2026 Tyler Hong
 * SPDX-License-Identifier: Apache-2.0
 *
 * quadrature: an incremental (A/B quadrature) encoder counter with 4 counts
 * per cycle, polled in C. Each pass reads both pins at once and looks up the
 * step for the (previous, current) state pair; a pair where both lines
 * changed is a missed step and counts as an error. Edges may come every
 * 0.5 us at 50 MHz (RTL sim): 2 million counts per second.
 *
 * Pins: A ui[0] (P16), B ui[1] (P17).
 * Results (PG_SHARED, updated continuously): [0] position (signed),
 * [1] errors, [2] 1 once running.
 */

#include "protocol.h"

int main(void)
{
    static const int8_t step[16] = {  /* index = previous AB << 2 | current AB; 2 = invalid */
         0, +1, -1,  2,
        -1,  0,  2, +1,
        +1,  2,  0, -1,
         2, -1, +1,  0 };
    volatile uint32_t *const in = &PG_GPIO_IN;
    uint32_t prev = (*in >> PIN_UI(0)) & 3u;
    int32_t pos = 0;
    uint32_t err = 0;
    PG_SHARED[2] = 1;                   /* running */
    for (;;) {
        uint32_t cur = (*in >> PIN_UI(0)) & 3u;
        if (cur == prev) continue;      /* the common case: nothing moved */
        int s = step[(prev << 2) | cur];
        if (s == 2) PG_SHARED[1] = ++err;
        else PG_SHARED[0] = (uint32_t)(pos += s);
        prev = cur;
    }
}
