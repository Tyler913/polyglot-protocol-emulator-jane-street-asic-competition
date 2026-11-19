/*
 * Copyright (c) 2026 Tyler Hong
 * SPDX-License-Identifier: Apache-2.0
 *
 * arinc429: ARINC 429 (avionics data bus) transmitter and receiver.
 * A word is 32 bits at 100 kbit/s, bipolar return-to-zero: a 1 is HI for
 * half a bit then NULL, a 0 is LO for half a bit then NULL; at least four
 * bit times of NULL between words. Bit 1 goes first; bits 1..8 are the
 * label, sent MSB first (so the label byte is bit-reversed), 9..10 SDI,
 * 11..29 data, 30..31 SSM, 32 odd parity.
 *  - TX, to a line driver with DATA A / DATA B inputs: two lite engines in
 *    lockstep send half-bit "chips" (200 kchip/s): A = 1 for a HI half,
 *    B = 1 for a LO half, both 0 = NULL.
 *  - RX, from a line receiver with two logic outputs (high while the line
 *    is HI, resp. LO): C polls both and takes a bit at each rising edge; a
 *    word ends after its 32nd bit, and a gap of over 2 bit times restarts.
 *
 * Pins: TX A uo[2] (P10), TX B uo[3] (P11); RX HI ui[1] (P17), RX LO ui[2] (P18).
 * Host parameters (PG_SHARED): [0] words to send (0..4), [1] words to
 * receive (0..4), [8..11] words to send (bit 1 = LSB, label as on the
 * wire; parity is set here).
 * Results: [2] 0x600D0014 when done; [16..19] received words (bit 1 =
 * LSB), [20] words with a parity error.
 */

#include "protocol.h"

#define TA W2
#define TB W3
#define CHIP (PG_F_CPU / 200000u)
#define BIT  (2u * CHIP)

static uint32_t par_odd(uint32_t w)              /* set bit 32 for odd parity */
{
    w &= 0x7FFFFFFFu;
    return w | ((ser_par(w) ^ 1u) << 31);
}

static void a429_send(const volatile uint32_t *w, uint32_t n)
{
    uint32_t base = WIRE_EN | WIRE_LEN(32);
    TA->CTRL = base; TB->CTRL = base;            /* TX off; queue first, then start both */
    for (uint32_t k = 0; k < n; k++) {
        uint32_t x = par_odd(w[k]);
        for (uint32_t h = 0; h < 2u; h++) {      /* 16 bits per engine word, LSB = first chip */
            uint32_t a = 0, b = 0;
            for (uint32_t i = 0; i < 16u; i++) {
                uint32_t bit = (x >> (16u * h + i)) & 1u;
                if (bit) a |= 1u << (2u * i); else b |= 1u << (2u * i);
            }
            wire_put(TA, a); wire_put(TB, b);
            if (k == 0 && h == 1) { TA->CTRL = base | WIRE_TX; TB->CTRL = base | WIRE_TX; }
        }
        wire_putn(TA, 8, 0); wire_putn(TB, 8, 0); /* 4 bit times of NULL */
    }
    wire_flush(TA);
}

static int a429_recv(uint32_t *out)
{
    uint32_t w = 0, n = 0, prev = 0, last = now();
    for (;;) {
        uint32_t in = (PG_GPIO_IN >> PIN_UI(1)) & 3u;   /* bit 0 = HI, bit 1 = LO */
        uint32_t rise = in & ~prev;
        prev = in;
        uint32_t t = now();
        if (n && t - last > 2u * BIT + BIT / 2u) { w = 0; n = 0; }   /* gap: start over */
        if (rise == 1u || rise == 2u) {
            if (rise == 1u) w |= 1u << n;
            n++;
            last = t;
            if (n == 32u) { *out = w; return ser_par(w) ? 0 : -1; }   /* odd parity */
        }
    }
}

int main(void)
{
    uint32_t ntx = PG_SHARED[0], nrx = PG_SHARED[1], bad = 0;
    if (ntx > 4u) ntx = 4u;
    if (nrx > 4u) nrx = 4u;
    wire_config(TA, (wire_cfg_t){ WIRE_EN | WIRE_LEN(32), CHIP, WIRE_PINS(PIN_UO(2), 0, 0), 0u });
    wire_config(TB, (wire_cfg_t){ WIRE_EN | WIRE_LEN(32), CHIP, WIRE_PINS(PIN_UO(3), 0, 0), 0u });
    host_irq(1);                                 /* ready */
    if (ntx) a429_send(&PG_SHARED[8], ntx);
    for (uint32_t k = 0; k < nrx; k++) {
        uint32_t w;
        if (a429_recv(&w) < 0) bad++;
        PG_SHARED[16 + k] = w;
    }
    PG_SHARED[20] = bad;
    PG_SHARED[2] = 0x600D0014u;
    host_irq(0);
    pg_park();
}
