/*
 * Copyright (c) 2026 Tyler Hong
 * SPDX-License-Identifier: Apache-2.0
 *
 * hdlc: synchronous HDLC frames (ISO/IEC 13239) with the transmit clock.
 * The engine does the bit-level work: zero insertion after five ones
 * (STUFF 5, runs of ones), and the 16-bit FCS (CRC-16/X.25: x^16 + x^12 +
 * x^5 + 1, initial 0xFFFF, sent complemented) appended and stuffed after
 * the last data byte. The flags 0x7E are pushed as words with the NOSTUFF
 * and NOCRC attributes, so they keep their six ones. Bytes go LSB first.
 *
 * Pins: TXD uo[0] (P8), TXC uo[1] (P9, data changes on the falling edge).
 * Sends three frames (address, control, information) at 1 Mbit/s with two
 * flags between frames.
 * Results (PG_SHARED): [0] 0x600D4D1C when done.
 */

#include "protocol.h"

#define HDLC W0

static void flag(void)
{
    HDLC->TXATTR = WIRE_A_NOSTUFF | WIRE_A_NOCRC;
    wire_putn(HDLC, 8, 0x7Eu);
}

static void hdlc_frame(const uint8_t *d, uint32_t n)
{
    flag();
    for (uint32_t i = 0; i < n; i++) {
        HDLC->TXATTR = (i == 0 ? WIRE_A_CRC_INIT : 0u) | (i == n - 1u ? WIRE_A_CRC_OUT : 0u);
        wire_putn(HDLC, 8, d[i]);
    }
    flag();
}

int main(void)
{
    static const uint8_t f1[] = { 0x03, 0x00, 'P', 'o', 'l', 'y', 'g', 'l', 'o', 't' };
    static const uint8_t f2[] = { 0xFF, 0x13, 0x7E, 0x7D, 0xFF, 0xFF, 0x3F, 0x1F, 0x00 };   /* worst case for stuffing */
    static const uint8_t f3[] = { 0x01, 0x11 };

    HDLC->CTRL   = 0;
    HDLC->DIV    = PG_BIT_TICKS(1000000);
    HDLC->CTRL2  = WIRE_STUFF(5);
    HDLC->PINS   = WIRE_PINS(PIN_UO(0), 0, PIN_UO(1));
    wire_crc(HDLC, 0x1021u, 16, 0xFFFFu, 1, 0);
    HDLC->CTRL   = WIRE_EN | WIRE_TX | WIRE_IDLE_HIGH | WIRE_CLK_OUT | WIRE_LEN(8);
    hdlc_frame(f1, sizeof f1);
    flag();
    hdlc_frame(f2, sizeof f2);
    hdlc_frame(f3, sizeof f3);
    wire_flush(HDLC);
    PG_SHARED[0] = 0x600D4D1Cu;
    host_irq(1);
    pg_park();
}
