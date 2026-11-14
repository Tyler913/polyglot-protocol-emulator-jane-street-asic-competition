/*
 * Copyright (c) 2026 Tyler Hong
 * SPDX-License-Identifier: Apache-2.0
 *
 * Job ws2812: a WS2812 LED chain from a plain NRZ engine stream at 2.4 Mbit/s
 * (3 line bits per LED bit, as in sw/examples/ws2812), then the latch time.
 * Mailbox: [1] DIN pin, [2] number of LEDs (1..6), [3..8] GRB colour of each LED
 * (0x00GGRRBB). Results: [14] start, [15] end.
 */

#include "job.h"

static uint32_t n, t;

static uint32_t expand(uint32_t b)                      /* 1 -> 110, 0 -> 100 */
{
    uint32_t w = 0;
    for (int i = 7; i >= 0; i--) w = (w << 3) | (((b >> i) & 1u) ? 6u : 4u);
    return w;
}

JOB_ENTRY
{
    PT_BEGIN(pt);
    mb[14] = now();
    wire_config(JOB_W, (wire_cfg_t){
        WIRE_EN | WIRE_TX | WIRE_MSB_FIRST | WIRE_LEN(24),
        PG_BIT_FIX8(2400000) >> 8, WIRE_PINS(mb[1], 0, 0), WIRE_FRAC(PG_BIT_FIX8(2400000)) });
    for (n = 0; n < 3 * mb[2]; n++) {
        PT_WAIT(pt, WIRE_TX_LEVEL(JOB_W->STATUS) < 4);
        wire_putn(JOB_W, 24, expand(mb[3 + n / 3] >> (16 - 8 * (n % 3))));
    }
    PT_WAIT(pt, tx_idle(JOB_W));
    t = now() + PG_US_TO_TICKS(60);                     /* latch */
    PT_WAIT(pt, time_reached(t));
    mb[15] = now();
    PT_END(pt);
}
