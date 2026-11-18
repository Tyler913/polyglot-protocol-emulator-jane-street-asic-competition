/*
 * Copyright (c) 2026 Tyler Hong
 * SPDX-License-Identifier: Apache-2.0
 *
 * i2s_rx: an I2S receiver (the clocks come from the other side, e.g. an ADC
 * or a codec): a lite engine samples SD on every rising BCLK edge (external
 * clock) into 32-bit words. To line the words up with the frames, C starts
 * the receiver right after a falling WS edge, stamped by the capture unit
 * (it retries if it was too slow to start before the next rising BCLK).
 * The first bit sampled after WS falls is still the right channel's LSB
 * (I2S sends the MSB one clock after WS changes), so each word holds that
 * LSB, the 16-bit left sample and the right sample's upper 15 bits.
 *
 * Pins: SD ui[1] (P17), BCLK ui[2] (P18), WS ui[3] (P19).
 * Host parameters: [0] frames to receive (<= 32, 0 = 16).
 * Results (PG_SHARED): [1] frames received, [2] start attempts; [16 + k]
 * frame k as right << 16 | left.
 */

#include "protocol.h"

#define RX  W3
#define SD  PIN_UI(1)
#define BCK PIN_UI(2)
#define WS  PIN_UI(3)

int main(void)
{
    uint32_t n = PG_SHARED[0] ? PG_SHARED[0] : 16u;
    if (n > 32u) n = 32u;
    static uint32_t w[33];

    RX->CTRL = 0;
    RX->PINS = WIRE_PINS(0, SD, BCK);
    uint32_t tries = 0;
    for (;;) {
        tries++;
        capture_arm(WS, EDGE_FALL);
        uint32_t t = capture_wait();
        RX->CTRL = WIRE_EN | WIRE_RX | WIRE_MSB_FIRST | WIRE_CLK_EXT | WIRE_OE_NEVER | WIRE_LEN(32);
        if (now() - t < 12u) break;          /* started before the next rising BCLK (16 cycles at 48 kHz) */
        RX->CTRL = 0;
    }
    for (uint32_t k = 0; k <= n; k++) w[k] = wire_get(RX);
    RX->CTRL = 0;
    for (uint32_t k = 0; k < n; k++) {
        uint32_t left = (w[k] >> 15) & 0xFFFFu;
        uint32_t right = ((w[k] & 0x7FFFu) << 1) | (w[k + 1] >> 31);
        PG_SHARED[16 + k] = (right << 16) | left;
    }
    PG_SHARED[1] = n;
    PG_SHARED[2] = tries;
    host_irq(1);
    pg_park();
}
