/*
 * Copyright (c) 2026 Tyler Hong
 * SPDX-License-Identifier: Apache-2.0
 *
 * eth_autoneg: 10BASE-T auto-negotiation (IEEE 802.3 clause 28) with fast
 * link pulses (FLP), the step before a modern switch port links up.
 * An FLP burst is 33 pulse positions 62.5 us apart: 17 clock pulses and, in
 * between, a data pulse for each 1 bit of the 16-bit Link Code Word (selector
 * 00001 = IEEE 802.3, technology ability: bit 5 10BASE-T, bit 6 10BASE-T
 * full duplex, ..., bit 14 Acknowledge). Bursts repeat every 16 ms.
 * Each pulse is the 100 ns link pulse of eth_link_pulse() at its time.
 * Between its own bursts the chip listens: the capture unit stamps every
 * rising edge on RD, and C decodes the partner's bursts (a pulse less than
 * 94 us after a clock pulse is a 1 bit).
 * Arbitration (simplified clause 28): after three identical Link Code Words
 * (Ack ignored) the partner's abilities are known and ours go out with Ack;
 * after three received words with Ack and at least six sent, negotiation is
 * complete and the highest common mode is chosen (full before half duplex).
 *
 * 60 MHz clock (as eth_ping). Pins: TD+ uio[6] (P6), TD- uio[7] (P7), RD
 * ui[0] (P16).
 * Host parameters (PG_SHARED): [0] our abilities (bits 5..12; 0 = 10BASE-T
 * half and full duplex).
 * Results: [2] 0x600D0016 when done (0xBAD00016: no agreement), [3] the
 * partner's Link Code Word, [4] mode: 2 = 10BASE-T full duplex, 1 = half,
 * 0 = none in common, [5] bursts sent.
 */

#ifndef PG_F_CPU
#define PG_F_CPU 60000000u
#endif
#include "protocol.h"

#define TX  W0
#define RD  PIN_UI(0)
#define US  PG_US_TO_TICKS(1)
#define ACK (1u << 14)

static uint32_t tm[40];

static void flp_burst(uint32_t lcw)
{
    uint32_t t = now() + 20u * US;
    for (uint32_t k = 0; k < 33u; k++) {
        if (!(k & 1u) || ((lcw >> (k >> 1)) & 1u)) {
            wait_until(t);
            eth_link_pulse(TX);
        }
        t += 125u * US / 2u;                       /* 62.5 us */
    }
}

/* Decode one burst from its pulse times; returns the word or -1 */
static int flp_decode(uint32_t n)
{
    uint32_t w = 0, bit = 0, i = 1;
    if (n < 17u) return -1;
    while (i < n && bit < 16u) {
        uint32_t d = tm[i] - tm[i - 1];
        if (d < 94u * US) {                        /* a data pulse: 1, then the next clock */
            w |= 1u << bit;
            i += 2;
        } else {
            i += 1;                                /* this pulse is already the next clock */
        }
        bit++;
    }
    return bit == 16u ? (int)w : -1;
}

/* Listen for window ticks; decode complete bursts; returns how many words were found */
static uint32_t flp_listen(uint32_t window, uint32_t *words, uint32_t max)
{
    uint32_t end = now() + window, n = 0, found = 0;
    for (;;) {
        int32_t left = (int32_t)(end - now());
        if (left <= 0) break;
        capture_arm(RD, EDGE_RISE);
        block_timeout((uint32_t)left);
        uint32_t t = capture_wait();
        int to = block_timed_out();
        block_timeout(0);
        if (to) break;
        if (n && t - tm[n - 1] > 200u * US) {      /* a gap: the previous burst is over */
            int w = flp_decode(n);
            if (w >= 0 && found < max) words[found++] = (uint32_t)w;
            n = 0;
        }
        if (n < 40u) tm[n++] = t;
    }
    int w = flp_decode(n);
    if (w >= 0 && found < max) words[found++] = (uint32_t)w;
    return found;
}

int main(void)
{
    uint32_t ours = 0x0001u | (PG_SHARED[0] ? (PG_SHARED[0] & 0x1FE0u) : 0x0060u);
    uint32_t last = 0xFFFFFFFFu, same = 0, match = 0, acks = 0, sent_ack = 0, partner = 0, bursts = 0;
    uint32_t w[4];
    PG_SHARED[2] = 0xBAD00016u;
    eth_init(TX, PIN_UIO(6), PIN_UIO(7));
    host_irq(1);                                   /* ready */
    for (uint32_t it = 0; it < 40u; it++) {
        flp_burst(ours | (match ? ACK : 0u));
        bursts++;
        if (match) sent_ack++;
        uint32_t k = flp_listen(14000u * US, w, 4);
        for (uint32_t i = 0; i < k; i++) {
            uint32_t v = w[i] & ~ACK;
            same = (v == last) ? same + 1u : 1u;
            last = v;
            if (same >= 3u && !match) { match = 1; partner = w[i]; }
            if (match && (w[i] & ACK)) { acks++; partner = w[i]; }
        }
        if (match && acks >= 3u && sent_ack >= 6u) break;
    }
    PG_SHARED[5] = bursts;
    if (!(match && acks >= 3u)) goto done;
    PG_SHARED[3] = partner;
    uint32_t common = ours & partner;
    PG_SHARED[4] = (common & 0x40u) ? 2u : (common & 0x20u) ? 1u : 0u;
    PG_SHARED[2] = 0x600D0016u;
done:
    host_irq(0);
    pg_park();
}
