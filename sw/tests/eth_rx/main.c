/*
 * Copyright (c) 2026 Tyler Hong
 * SPDX-License-Identifier: Apache-2.0
 *
 * eth_rx: 10BASE-T receive on Wire Engine 1. The engine decodes Manchester
 * by edge timing (clock recovery), hunts for the SFD, and checks the FCS
 * with its CRC unit; the C code only stores the words. Used by
 * test/test_eth_rx.py at 50 and 60 MHz.
 *
 * Pin: RD (a line receiver's output, idle low) on ui[0] (P16).
 * Host parameters (PG_SHARED): [0] frames to receive, [1] bit period in
 * cycles if the clock is not 50 MHz (6 at 60 MHz).
 * Results: [1 + i] length of frame i (FCS included) or -1 (bad FCS) / -2;
 * [16] address of the buffer holding the frames back to back, each padded to
 * 4 bytes; [17] 0x600D0E7E when done.
 */

#include "protocol.h"

#define ETH W1

static uint8_t buf[1024] __attribute__((aligned(4)));

int main(void)
{
    uint32_t frames = PG_SHARED[0], off = 0;
    PG_SHARED[16] = (uint32_t)buf;
    eth_rx_init(ETH, PIN_UI(0));
    if (PG_SHARED[1]) ETH->DIV = PG_SHARED[1];
    host_irq(1);                                   /* ready */
    for (uint32_t i = 0; i < frames && i < 15u; i++) {
        int n = eth_recv(ETH, buf + off, sizeof buf - off);
        PG_SHARED[1 + i] = (uint32_t)n;
        if (n > 0) off += ((uint32_t)n + 3u) & ~3u;
    }
    PG_SHARED[17] = 0x600D0E7Eu;
    host_irq(0);
    pg_park();
}
