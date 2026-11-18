/*
 * Copyright (c) 2026 Tyler Hong
 * SPDX-License-Identifier: Apache-2.0
 *
 * ir_rx: an infrared remote-control receiver for two different line codes,
 * NEC (pulse distance) and Philips RC-5 (Manchester), decoded in C from
 * edge timestamps. The input is an IR receiver module's output (38/36 kHz
 * demodulated, low while a burst is seen). The capture unit stamps every
 * edge with the cycle it happened in; the code turns the stamps into
 * low/high durations and classifies them, so timing tolerance is a
 * software choice (here +/- 25 %).
 *
 * Pin: ui[3] (P19).
 * Results (PG_SHARED): for each frame i (up to 8), [1 + i] =
 * 0x01AACC00 NEC (address AA, command CC), 0x01AACCFF NEC repeat code of
 * that, 0x05TAACC RC-5 (toggle T, address AA, command CC), 0xEE.. error;
 * [0] number of frames. HOST_IRQ pulses after each frame.
 */

#include "protocol.h"

#define IR_PIN PIN_UI(3)
#define US(x)  PG_US_TO_TICKS(x)

static uint32_t dur[80];   /* alternating low, high durations (ticks), starting with a low */

/* Collect one burst train: durations until the line stays high for 20 ms */
static int collect(void)
{
    int n = 0;
    capture_arm(IR_PIN, EDGE_BOTH);
    block_timeout(0);
    uint32_t t = capture_wait();              /* first falling edge */
    block_timeout(US(20000));
    for (;;) {
        uint32_t u = capture_wait();
        if (block_timed_out()) break;
        if (n < 80) dur[n++] = u - t;
        t = u;
    }
    block_timeout(0);
    return n;                                 /* the last high (to idle) has no end */
}

static int near(uint32_t d, uint32_t us) { uint32_t x = US(us); return d > x - x / 4u && d < x + x / 4u; }

static uint32_t decode_nec(int n)
{
    if (n >= 3 && near(dur[0], 9000) && near(dur[1], 2250)) return 0xFFu;    /* repeat */
    if (n < 67 || !near(dur[1], 4500)) return 0xEE000001u;
    uint32_t v = 0;
    for (int i = 0; i < 32; i++) {
        uint32_t lo = dur[2 + 2 * i], hi = dur[3 + 2 * i];
        if (!near(lo, 563)) return 0xEE000002u;
        if (near(hi, 1688)) v |= 1u << i;
        else if (!near(hi, 563)) return 0xEE000003u;
    }
    uint32_t a = v & 0xFFu, na = (v >> 8) & 0xFFu, c = (v >> 16) & 0xFFu, nc = v >> 24;
    if ((a ^ na) != 0xFFu || (c ^ nc) != 0xFFu) return 0xEE000004u;
    return (a << 16) | (c << 8);
}

/* RC-5: 14 bits of 1.778 ms; a 1 is high-then-low at the receiver output.
 * Expand the durations into half bits, starting with the idle-high first
 * half of the first start bit. */
static uint32_t decode_rc5(int n)
{
    uint8_t half[28];
    int h = 0;
    half[h++] = 1;
    for (int i = 0; i < n && h < 28; i++) {
        int k = near(dur[i], 889) ? 1 : near(dur[i], 1778) ? 2 : 0;
        if (!k) return 0xEE000005u;
        while (k-- && h < 28) half[h++] = (i & 1) ? 1u : 0u;
    }
    while (h < 28) half[h++] = 1;             /* trailing high merges with idle */
    uint32_t v = 0;
    for (int b = 0; b < 14; b++) {
        if (half[2 * b] == half[2 * b + 1]) return 0xEE000006u;
        v = (v << 1) | half[2 * b];
    }
    if ((v >> 12) != 3u) return 0xEE000007u;  /* two start bits (RC-5, not RC-5X) */
    return 0x05000000u | (((v >> 11) & 1u) << 16) | (((v >> 6) & 31u) << 8) | (v & 63u);
}

int main(void)
{
    uint32_t last_nec = 0;
    host_irq(0);
    for (uint32_t f = 0; f < 8; f++) {
        int n = collect();
        uint32_t r;
        if (dur[0] > US(6000)) {
            r = decode_nec(n);
            if (r == 0xFFu) r = last_nec | 0xFFu;
            else if (r < 0xEE000000u) last_nec = r |= 0x01000000u;
        } else {
            r = decode_rc5(n);
        }
        PG_SHARED[1 + f] = r;
        PG_SHARED[0] = f + 1u;
        host_irq(1);
        host_irq(0);
    }
    pg_park();
}
