/*
 * Copyright (c) 2026 Tyler Hong
 * SPDX-License-Identifier: Apache-2.0
 *
 * i2s: an I2S audio transmitter (Philips I2S: BCLK, WS, SD; 16-bit samples,
 * MSB first, one BCLK after each WS change; left channel while WS is low)
 * from two lite Wire Engines running in lockstep. Engine 2 sends the
 * samples and drives BCLK (SPI mode 0: data changes on the falling edge,
 * the receiver samples on the rising edge). Engine 3 sends the WS pattern
 * with the same bit period; both are started with back-to-back register
 * writes, so their bit grids stay a fixed 1-2 cycles apart. 48 kHz stereo:
 * BCLK 1.536 MHz (32.55 cycles per bit at 50 MHz, fractional divider).
 *
 * Pins: SD uo[2] (P10), BCLK uo[3] (P11), WS uo[4] (P12).
 * Host parameters: [0] number of stereo frames (0 = 16).
 * Left sample k = 1000 * k - 8000, right = 0x5A5A ^ k.
 * Results (PG_SHARED): [1] 0x600D0125 when done.
 */

#include "protocol.h"

#define SD W2
#define WS W3
#define FS 48000u

static int16_t left(uint32_t k)  { return (int16_t)(1000 * (int32_t)k - 8000); }
static int16_t right(uint32_t k) { return (int16_t)(0x5A5A ^ k); }

int main(void)
{
    uint32_t frames = PG_SHARED[0] ? PG_SHARED[0] : 16u;
    uint32_t fix8 = PG_BIT_FIX8(FS * 32u);
    uint32_t base = WIRE_EN | WIRE_MSB_FIRST | WIRE_LEN(32);

    SD->CTRL = 0;  WS->CTRL = 0;
    SD->DIV = fix8 >> 8;  SD->CTRL2 = WIRE_FRAC(fix8);
    WS->DIV = fix8 >> 8;  WS->CTRL2 = WIRE_FRAC(fix8);
    SD->PINS = WIRE_PINS(PIN_UO(2), 0, PIN_UO(3));
    WS->PINS = WIRE_PINS(PIN_UO(4), 0, 0);
    SD->CTRL = base | WIRE_CLK_OUT;               /* enabled, not sending yet */
    WS->CTRL = base | WIRE_IDLE_HIGH;             /* WS high before the first left word */
    wire_putn(SD, 1, 0);                          /* one bit first, so WS falls one BCLK */
    wire_putn(WS, 1, 0);                          /* before the first left MSB */
    for (uint32_t k = 0; k < 2u && k < frames; k++) {   /* prefill: two frames each */
        wire_put(SD, ((uint32_t)(uint16_t)left(k) << 16) | (uint16_t)right(k));
        wire_put(WS, 0x0001FFFEu);                /* low for the left word, high one bit early for the right */
    }
    SD->CTRL = base | WIRE_CLK_OUT | WIRE_TX;     /* start both, two stores apart */
    WS->CTRL = base | WIRE_IDLE_HIGH | WIRE_TX;
    for (uint32_t k = 2; k < frames; k++) {
        wire_put(SD, ((uint32_t)(uint16_t)left(k) << 16) | (uint16_t)right(k));
        wire_put(WS, 0x0001FFFEu);
    }
    wire_flush(SD);
    PG_SHARED[1] = 0x600D0125u;
    host_irq(1);
    pg_park();
}
