/*
 * Copyright (c) 2026 Tyler Hong
 * SPDX-License-Identifier: Apache-2.0
 *
 * dshot: DShot600, the digital protocol between drone flight controllers and
 * motor ESCs. A frame is 16 bits, MSB first: 11-bit throttle, telemetry
 * request, 4-bit checksum (XOR of the three nibbles above it). Each bit lasts
 * 1.667 us and is a high pulse of 75 % (1) or 37.5 % (0). As with WS2812,
 * that is an NRZ stream on a lite engine: 8 line bits per DShot bit at
 * 4.8 Mbit/s, 11111100 for a 1 and 11100000 for a 0; one frame = four
 * 32-bit words.
 *
 * Pin: uo[5] (P13) to the ESC signal input.
 * Sends frames for throttle values 0, 48, 1046 (telemetry), 2047.
 * Results (PG_SHARED): [0] 0x600DD540 when done.
 */

#include "protocol.h"

#define ESC W3

static uint32_t dshot_frame(uint32_t throttle, uint32_t telemetry)
{
    uint32_t v = (throttle << 1) | (telemetry & 1u);
    return (v << 4) | ((v ^ (v >> 4) ^ (v >> 8)) & 0xFu);
}

static void dshot_send(uint32_t frame)
{
    uint32_t w = 0;
    for (int i = 15; i >= 0; i--) {
        w = (w << 8) | (((frame >> i) & 1u) ? 0xFCu : 0xE0u);
        if ((i & 3) == 0) wire_putn(ESC, 32, w);      /* four DShot bits per word */
    }
}

int main(void)
{
    wire_config(ESC, (wire_cfg_t){
        WIRE_EN | WIRE_TX | WIRE_MSB_FIRST | WIRE_LEN(32),
        PG_BIT_FIX8(4800000) >> 8, WIRE_PINS(PIN_UO(5), 0, 0), WIRE_FRAC(PG_BIT_FIX8(4800000)) });
    static const uint16_t throttle[4] = { 0, 48, 1046, 2047 };
    for (int i = 0; i < 4; i++) {
        dshot_send(dshot_frame(throttle[i], i == 2));
        wire_flush(ESC);
        delay_ticks(PG_US_TO_TICKS(20));               /* gap between frames */
    }
    PG_SHARED[0] = 0x600DD540u;
    host_irq(1);
    pg_park();
}
