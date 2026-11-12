/*
 * Copyright (c) 2026 Tyler Hong
 * SPDX-License-Identifier: Apache-2.0
 *
 * eth_tx: a 10BASE-T Ethernet frame from Wire Engine 1: Manchester code,
 * differential TD+/TD-, and the frame check sequence (CRC-32) appended by
 * the engine. Built for a 40 MHz clock (4 cycles per bit, 2 per half bit).
 * Used by test/test_eth.py.
 *
 * Pins: TD+ uio[6] (P6), TD- uio[7] (P7); released (0 V differential) when idle.
 */

#define PG_F_CPU 40000000u
#include "protocol.h"

#define ETH W1

static const uint8_t frame[60] =
    "\xff\xff\xff\xff\xff\xff"            /* destination: broadcast */
    "\x02\x00\x00\x00\x00\x01"            /* source: locally administered */
    "\x88\xb5"                            /* EtherType: local experimental */
    "Polyglot 10BASE-T from plain C";     /* payload, zero padded to 46 bytes */

int main(void)
{
    eth_init(ETH, PIN_UIO(6), PIN_UIO(7));
    eth_send(ETH, frame, sizeof frame);
    wire_flush(ETH);
    host_irq(1);
    pg_park();
}
