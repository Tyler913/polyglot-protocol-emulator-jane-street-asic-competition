/*
 * Copyright (c) 2026 Tyler Hong
 * SPDX-License-Identifier: Apache-2.0
 *
 * dmx512: a DMX512 (ANSI E1.11) stage-lighting controller, as an example of
 * a protocol added in a few lines with no hardware support. DMX512 is a
 * 250 kbit/s serial stream (start bit, 8 data bits LSB first, 2 stop bits)
 * whose packets begin with a BREAK (line low for at least 92 us) and a
 * MARK AFTER BREAK (high for at least 12 us). On a plain NRZ engine with the
 * line idling high, the BREAK + MAB is one 26-bit word and each slot an
 * 11-bit word: nothing else is needed.
 *
 * Pin: uo[1] (P9), to an RS-485 driver.
 * Host parameters (PG_SHARED): [0] channels per packet (1..512, 0 = 16),
 * [1] packets (0 = 2). Channel i of packet p is (7 * i + p) & 0xFF.
 * Results: [2] 0x600D0005 when done.
 */

#include "protocol.h"

#define DMX W3

static void dmx_slot(uint8_t b) { wire_putn(DMX, 11, 0x600u | ((uint32_t)b << 1)); }

static void dmx_packet(uint32_t n, uint32_t p)
{
    wire_putn(DMX, 26, 0x7u << 23);    /* BREAK: 23 bits low (92 us), MAB: 3 bits high (12 us) */
    dmx_slot(0x00);                    /* start code: dimmer data */
    for (uint32_t i = 1; i <= n; i++) dmx_slot((uint8_t)(7u * i + p));
}

int main(void)
{
    uint32_t n = PG_SHARED[0] ? PG_SHARED[0] : 16u, packets = PG_SHARED[1] ? PG_SHARED[1] : 2u;
    if (n > 512u) n = 512u;

    wire_config(DMX, (wire_cfg_t){ WIRE_EN | WIRE_TX | WIRE_IDLE_HIGH, PG_BIT_TICKS(250000),
                                   WIRE_PINS(PIN_UO(1), 0, 0), 0u });
    for (uint32_t p = 0; p < packets; p++) dmx_packet(n, p);
    wire_flush(DMX);
    PG_SHARED[2] = 0x600D0005u;
    host_irq(1);
    pg_park();
}
