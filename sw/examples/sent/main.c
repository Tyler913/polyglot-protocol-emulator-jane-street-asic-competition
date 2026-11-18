/*
 * Copyright (c) 2026 Tyler Hong
 * SPDX-License-Identifier: Apache-2.0
 *
 * sent: a SAE J2716 SENT sensor (automotive single-edge nibble transmission).
 * Data are pulse lengths: every pulse starts with a falling edge and a low
 * time of at least 4 ticks, and a nibble n lasts 12 + n ticks. A message is
 * a 56-tick calibration/sync pulse, a status nibble, six data nibbles (two
 * 12-bit signals) and a CRC-4 (x^4 + x^3 + x^2 + 1, seed 0101, with the
 * extra zero nibble of the 2010 recommended method). On a lite engine with
 * one line bit per 3 us tick, a pulse of N ticks is one N-bit word
 * (4 zero bits, then ones), so the C code only builds words.
 *
 * Pin: uo[0] (P8). Host parameters: [0], [1] the two 12-bit signals.
 * Sends 3 messages with a counter in the status nibble.
 * Results (PG_SHARED): [2] 0x600D5E47 when done.
 */

#include "protocol.h"

#define SENT W2
#define TICK_HZ 333333u                /* 3 us tick */

static void pulse(uint32_t ticks)       /* ticks <= 32 */
{
    wire_putn(SENT, (int)ticks, (0xFFFFFFFFu >> (32u - ticks)) & ~0xFu);
}

/* CRC-4 by polynomial arithmetic: crc = crc * x^4 mod P, then + nibble */
static uint32_t crc4_mul(uint32_t c)
{
    for (int i = 0; i < 4; i++) {
        c <<= 1;
        if (c & 0x10u) c ^= 0x1Du;
    }
    return c;
}

static void sent_message(uint32_t status, const uint8_t *nib)
{
    uint32_t crc = 5u;
    pulse(28);                            /* sync: 56 ticks as 28 + 28 */
    wire_putn(SENT, 28, 0x0FFFFFFFu);
    pulse(12u + (status & 15u));
    for (int i = 0; i < 6; i++) {
        crc = crc4_mul(crc) ^ nib[i];
        pulse(12u + nib[i]);
    }
    crc = crc4_mul(crc);                  /* the extra zero nibble */
    pulse(12u + crc);
}

int main(void)
{
    uint32_t a = PG_SHARED[0] & 0xFFFu, b = PG_SHARED[1] & 0xFFFu;
    wire_config(SENT, (wire_cfg_t){
        WIRE_EN | WIRE_TX | WIRE_IDLE_HIGH | WIRE_LEN(32),
        PG_BIT_FIX8(TICK_HZ) >> 8, WIRE_PINS(PIN_UO(0), 0, 0), WIRE_FRAC(PG_BIT_FIX8(TICK_HZ)) });
    for (uint32_t m = 0; m < 3; m++) {
        uint32_t x = a + m, y = b - m;
        uint8_t nib[6] = { (uint8_t)(x >> 8), (uint8_t)((x >> 4) & 15u), (uint8_t)(x & 15u),
                           (uint8_t)(y & 15u), (uint8_t)((y >> 4) & 15u), (uint8_t)(y >> 8) };
        sent_message(m, nib);
    }
    pulse(20);                            /* pause pulse: its falling edge ends the last CRC nibble */
    wire_flush(SENT);
    PG_SHARED[2] = 0x600D5E47u;
    host_irq(1);
    pg_park();
}
