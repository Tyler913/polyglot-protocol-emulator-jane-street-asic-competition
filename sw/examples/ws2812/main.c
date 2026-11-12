/*
 * Copyright (c) 2026 Tyler Hong
 * SPDX-License-Identifier: Apache-2.0
 *
 * ws2812: drives a chain of WS2812 ("NeoPixel") LEDs, a protocol the chip
 * has no hardware for. Each LED bit is three bits of a plain NRZ Wire Engine
 * stream at 2.4 Mbit/s: 100 for a 0 (0.42 us high), 110 for a 1 (0.83 us
 * high). The fractional divider gives the 20.83-cycle bit at 50 MHz. After
 * the data the line stays low for the >50 us reset/latch time.
 *
 * Pin: DIN of the first LED on uo[3] (P11).
 * Host parameters (PG_SHARED): [0] number of LEDs (0 = 4).
 */

#include "protocol.h"

#define LEDS W0

/* 8 data bits -> 24 line bits, MSB first: 1 -> 110, 0 -> 100 */
static uint32_t expand(uint8_t b)
{
    uint32_t w = 0;
    for (int i = 7; i >= 0; i--) w = (w << 3) | (((b >> i) & 1u) ? 6u : 4u);
    return w;
}

static void ws2812_show(const uint8_t (*grb)[3], uint32_t n)
{
    for (uint32_t i = 0; i < n; i++)
        for (int c = 0; c < 3; c++)
            wire_putn(LEDS, 24, expand(grb[i][c]));
    wire_flush(LEDS);
    delay_ticks(PG_US_TO_TICKS(60));       /* latch */
}

int main(void)
{
    static const uint8_t colours[4][3] = {     /* G, R, B */
        {0x00, 0xFF, 0x00}, {0xFF, 0x00, 0x00}, {0x00, 0x00, 0xFF}, {0x12, 0x34, 0x56},
    };
    uint32_t n = PG_SHARED[0] ? PG_SHARED[0] : 4u;

    wire_config(LEDS, (wire_cfg_t){
        WIRE_EN | WIRE_TX | WIRE_MSB_FIRST | WIRE_LEN(24),
        PG_BIT_FIX8(2400000) >> 8, WIRE_PINS(PIN_UO(3), 0, 0), WIRE_FRAC(PG_BIT_FIX8(2400000)) });
    ws2812_show(colours, n);
    host_irq(1);
    pg_park();
}
