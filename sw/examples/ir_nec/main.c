/*
 * Copyright (c) 2026 Tyler Hong
 * SPDX-License-Identifier: Apache-2.0
 *
 * ir_nec: an NEC infrared remote control transmitter. The 38 kHz carrier is
 * made by the engine itself: at 76 kbit/s a mark is the bit pattern 1010...
 * (one carrier period per two bits) and a space is all zeros, so the whole
 * modulated waveform is plain pushes. NEC frame: 9 ms mark, 4.5 ms space,
 * 32 bits LSB first (address, ~address, command, ~command), each a 562.5 us
 * mark then a 562.5 us (0) or 1687.5 us (1) space, and a final 562.5 us mark.
 * A held key sends repeat codes (9 ms mark, 2.25 ms space, 562.5 us mark)
 * every 108 ms.
 *
 * Pin: uo[2] (P10), to an IR LED driver (high = LED on).
 * Host parameters (PG_SHARED): [0] address, [1] command, [2] repeat codes.
 * Results: [3] 0x600D0007 when done.
 */

#include "protocol.h"

#define IR W3
#define RATE 76000u                       /* two line bits per 38 kHz carrier period */

/* n line bits of the pattern p (0x55555555: carrier on, 0: off) */
static void ir_bits(uint32_t p, uint32_t n)
{
    for (; n >= 32u; n -= 32u) wire_putn(IR, 32, p);
    if (n) wire_putn(IR, (int)n, p);
}

/* Durations in units of 562.5 us (= 42.75 line bits; the error stays below one bit) */
static uint32_t acc;                      /* line bits sent so far, x4 */
static void ir_seg(uint32_t pattern, uint32_t units)
{
    uint32_t start = acc / 4u;
    acc += 171u * units;                  /* 42.75 * 4 per unit */
    ir_bits(pattern, acc / 4u - start);
}
#define MARK(u)  ir_seg(0x55555555u, (u))
#define SPACE(u) ir_seg(0u, (u))

static void nec_frame(uint8_t addr, uint8_t cmd)
{
    uint32_t data = addr | ((uint32_t)(uint8_t)~addr << 8) | ((uint32_t)cmd << 16) |
                    ((uint32_t)(uint8_t)~cmd << 24);
    MARK(16);                             /* 9 ms */
    SPACE(8);                             /* 4.5 ms */
    for (int i = 0; i < 32; i++) {
        MARK(1);
        SPACE((data >> i) & 1u ? 3u : 1u);
    }
    MARK(1);
}

int main(void)
{
    uint8_t addr = (uint8_t)PG_SHARED[0], cmd = (uint8_t)PG_SHARED[1];
    uint32_t repeats = PG_SHARED[2];

    wire_config(IR, (wire_cfg_t){ WIRE_EN | WIRE_TX, PG_BIT_FIX8(RATE) >> 8,
                                  WIRE_PINS(PIN_UO(2), 0, 0), WIRE_FRAC(PG_BIT_FIX8(RATE)) });
    uint32_t t0 = now();
    nec_frame(addr, cmd);
    for (uint32_t r = 1; r <= repeats; r++) {
        wire_flush(IR);
        wait_until(t0 + r * PG_US_TO_TICKS(108000));    /* repeat period from frame start */
        acc = 0;
        MARK(16);
        SPACE(4);                         /* 2.25 ms */
        MARK(1);
    }
    wire_flush(IR);
    PG_SHARED[3] = 0x600D0007u;
    host_irq(1);
    pg_park();
}
