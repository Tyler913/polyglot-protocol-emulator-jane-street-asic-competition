/*
 * Copyright (c) 2026 Tyler Hong
 * SPDX-License-Identifier: Apache-2.0
 *
 * biss: a BiSS-C master (unidirectional, point to point) reading absolute
 * position encoders, as used on servo motors; SSI encoders use the same
 * wiring. The master clocks MA (idle high); the encoder answers on SL:
 * Ack (low, as long as it needs), Start (1), CDS (0), the position MSB
 * first, nE and nW (error and warning, active low), CRC-6 (polynomial
 * x^6 + x + 1, over position, nE and nW, sent inverted), then holds SL low
 * for its timeout and releases it. Cable and converter delays move the
 * whole answer by some clocks; the master just looks for the Start bit, so
 * no delay compensation is needed in hardware.
 *
 * Here: SPI engine, mode 3 (MA idles high, SL sampled on rising MA edges),
 * 5 MHz; 48 clocks per frame as two 24-bit words queued back to back.
 *
 * Pins: MA uo[0] (P8), SL ui[3] (P19); uo[1] is the (unused) data output.
 * Host parameters (PG_SHARED): [0] frames (1..6), [1] position bits (8..26).
 * Results: [2] 0x600D000E when done; frame k: [8 + 2k] position,
 * [9 + 2k] status: bit 0 CRC ok, bit 1 nE, bit 2 nW, bit 3 start found,
 * bits 15..8 clocks from the first MA edge to the Start bit.
 */

#include "protocol.h"

#define BI W0

static uint32_t crc6(uint32_t v, uint32_t n)       /* MSB first, x^6 + x + 1 */
{
    uint32_t c = 0;
    while (n--) {
        uint32_t fb = ((v >> n) ^ (c >> 5)) & 1u;
        c = (c << 1) & 0x3Fu;
        if (fb) c ^= 0x03u;
    }
    return c;
}

int main(void)
{
    uint32_t frames = PG_SHARED[0], nb = PG_SHARED[1];
    if (frames < 1u || frames > 6u) frames = 1u;
    if (nb < 8u || nb > 26u) nb = 26u;
    wire_config(BI, WIRE_SPI_CTRL(PIN_UO(0), PIN_UO(1), PIN_UI(3), 3, 5000000));
    BI->CTRL = (BI->CTRL & ~WIRE_LEN(32)) | WIRE_LEN(24);
    host_irq(1);                                    /* ready */
    delay_ticks(PG_US_TO_TICKS(40));                /* MA high longer than any encoder timeout */
    for (uint32_t k = 0; k < frames; k++) {
        while (!pin_read(PIN_UI(3))) { }            /* encoder ready (SL high) */
        wire_put(BI, 0xFFFFFFu);
        wire_put(BI, 0xFFFFFFu);
        uint32_t hi = wire_get(BI), lo = wire_get(BI);   /* 48 samples, first in bit 23 of hi */
        uint32_t st = 0, pos = 0, i = 0, bitv;
#define SAMPLE(j) (((j) < 24u ? hi >> (23u - (j)) : lo >> (47u - (j))) & 1u)
        while (i < 48u && SAMPLE(i)) i++;           /* before the encoder reacts: SL high */
        while (i < 48u && !SAMPLE(i)) i++;          /* Ack */
        if (i + 4u + nb + 6u <= 48u) {              /* Start found, the rest fits */
            st |= 8u | (i << 8);
            i += 2;                                 /* Start, CDS */
            for (uint32_t j = 0; j < nb + 2u; j++) pos = (pos << 1) | SAMPLE(i + j);   /* + nE nW */
            uint32_t crc = 0;
            for (uint32_t j = 0; j < 6u; j++) crc = (crc << 1) | SAMPLE(i + nb + 2u + j);
            if ((crc ^ 0x3Fu) == crc6(pos, nb + 2u)) st |= 1u;
            bitv = pos & 3u;                        /* nE, nW */
            st |= ((bitv >> 1) & 1u) << 1 | (bitv & 1u) << 2;
            pos >>= 2;
        }
#undef SAMPLE
        PG_SHARED[8 + 2 * k] = pos;
        PG_SHARED[9 + 2 * k] = st;
        delay_ticks(PG_US_TO_TICKS(2));             /* MA high: the encoder times out */
    }
    PG_SHARED[2] = 0x600D000Eu;
    host_irq(0);
    pg_park();
}
