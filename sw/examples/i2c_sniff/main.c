/*
 * Copyright (c) 2026 Tyler Hong
 * SPDX-License-Identifier: Apache-2.0
 *
 * i2c_sniff: a passive I2C bus analyzer. The same engine settings as the
 * I2C target (SCL as the external bit clock, frames from START to STOP,
 * frame events), but with the TX side off and 9-bit words, so every byte
 * arrives together with its ACK bit and the chip never touches the bus.
 * The C code only stores the words in a trace buffer.
 *
 * Pins: SDA uio[2] (P2), SCL uio[3] (P3), inputs only.
 * Results (PG_SHARED): [0] entries in the trace, [1] trace address. Each
 * entry is a 16-bit value: 0x1000 START (or repeated START), 0x2000 STOP,
 * otherwise byte << 1 | NACK.
 */

#include "protocol.h"

#define SNIFF W1

static uint16_t trace[256];

int main(void)
{
    PG_SHARED[0] = 0;
    PG_SHARED[1] = (uint32_t)trace;
    wire_config(SNIFF, (wire_cfg_t){
        WIRE_EN | WIRE_RX | WIRE_MSB_FIRST | WIRE_IDLE_HIGH | WIRE_OE_NEVER |
        WIRE_CLK_EXT | WIRE_GATE_I2C | WIRE_EVENTS | WIRE_LEN(9),
        2u, WIRE_PINS(PIN_UIO(2), PIN_UIO(2), PIN_UIO(3)), 0u });
    host_irq(1);
    for (uint32_t n = 0; n < 256u; n++) {
        uint32_t w = wire_get(SNIFF);
        trace[n] = (w == WIRE_EV_START) ? 0x1000u : (w == WIRE_EV_END) ? 0x2000u : (uint16_t)(w & 0x1FFu);
        PG_SHARED[0] = n + 1u;
    }
    pg_park();
}
