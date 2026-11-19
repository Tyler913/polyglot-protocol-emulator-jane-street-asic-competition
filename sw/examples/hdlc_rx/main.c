/*
 * Copyright (c) 2026 Tyler Hong
 * SPDX-License-Identifier: Apache-2.0
 *
 * hdlc_rx: receives synchronous HDLC frames (ISO/IEC 13239) with the
 * receive clock from the line (modem or transceiver). The engine only
 * collects raw bits (external clock, 32 per word); C does the bit layer:
 * a 0 after five ones is a stuffed bit and is dropped, six ones and a 0
 * is a flag (frame boundary), seven or more ones abort the frame. Bytes
 * are assembled LSB first; the FCS (CRC-16/X.25) is checked by its
 * residual 0xF0B8 over data + FCS. About 15 cycles per bit, so 1 Mbit/s
 * (50 cycles per bit) leaves time to spare.
 *
 * Pins: RXD ui[0] (P16), RXC ui[1] (P17, data sampled on rising edges).
 * Host parameters (PG_SHARED): [0] frames to report (1..4).
 * Results: [2] 0x600D0013 when done, [3] aborted frames, [4] frames with a
 * bad FCS or length; frame k: [8 + 12k] length (bytes, FCS excluded),
 * [9 + 12k ..] its first 44 bytes, little endian.
 */

#include "protocol.h"

#define RX W2

static uint8_t buf[64];

static uint16_t fcs16(const uint8_t *p, uint32_t n)
{
    uint32_t c = 0xFFFFu;
    while (n--) {
        c ^= *p++;
        for (int i = 0; i < 8; i++) c = (c & 1u) ? (c >> 1) ^ 0x8408u : c >> 1;
    }
    return (uint16_t)c;
}

int main(void)
{
    uint32_t want = PG_SHARED[0], got = 0;
    uint32_t ones = 0, nbits = 0, byte = 0, len = 0, in_frame = 0;
    if (want < 1u || want > 4u) want = 1u;
    PG_SHARED[3] = PG_SHARED[4] = 0;
    RX->CTRL = 0;
    RX->PINS = WIRE_PINS(0, PIN_UI(0), PIN_UI(1));
    RX->CTRL = WIRE_EN | WIRE_RX | WIRE_CLK_EXT | WIRE_OE_NEVER | WIRE_LEN(32);
    host_irq(1);                                   /* ready */
    while (got < want) {
        uint32_t w = wire_get(RX);
        for (int i = 0; i < 32; i++, w >>= 1) {    /* first received bit = bit 0 */
            uint32_t b = w & 1u;
            if (b) {
                if (++ones >= 7u) {                /* abort (or idle ones) */
                    if (in_frame && len) PG_SHARED[3]++;
                    in_frame = 0;
                }
                if (ones > 5u) continue;           /* maybe a flag: decided by the next 0 */
            } else {
                if (ones == 5u) { ones = 0; continue; }   /* stuffed zero */
                if (ones == 6u) {                  /* flag: 0111 1110 */
                    ones = 0;
                    if (in_frame && len >= 3u) {   /* the flag's 0 and five ones were taken as data */
                        if (nbits == 6u && fcs16(buf, len) == 0xF0B8u) {
                            volatile uint32_t *r = &PG_SHARED[8 + 12 * got];
                            r[0] = len - 2u;
                            for (uint32_t j = 0; j < 44u; j++) {
                                if ((j & 3u) == 0) r[1 + (j >> 2)] = 0;
                                if (j < len - 2u) r[1 + (j >> 2)] |= (uint32_t)buf[j] << (8 * (j & 3u));
                            }
                            if (++got == want) break;
                        } else {
                            PG_SHARED[4]++;
                        }
                    }
                    in_frame = 1; len = 0; nbits = 0; byte = 0;
                    continue;
                }
                ones = 0;
            }
            if (!in_frame) continue;
            byte |= b << nbits;                    /* a data bit, LSB first */
            if (++nbits == 8u) {
                if (len < sizeof buf) buf[len++] = (uint8_t)byte;
                nbits = 0; byte = 0;
            }
        }
    }
    PG_SHARED[2] = 0x600D0013u;
    host_irq(0);
    pg_park();
}
