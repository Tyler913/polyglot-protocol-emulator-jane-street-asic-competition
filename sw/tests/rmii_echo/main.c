/*
 * Copyright (c) 2026 Tyler Hong
 * SPDX-License-Identifier: Apache-2.0
 *
 * rmii_echo: 100 Mbit/s Ethernet through the experimental RMII port: every
 * frame with a good FCS is sent back with destination and source MAC
 * addresses swapped (the hardware recomputes the FCS). Needs the RTL built
 * with PG_RMII. Used by test/test_rmii.py.
 * Host parameters (PG_SHARED): [0] frames to handle.
 * Results: [2] 0x600D0017 when done, [3] frames echoed, [4] bad FCS,
 * [5] too long, [6] STATUS at the end (underrun / overflow flags).
 */

#include "protocol.h"

static uint32_t buf[384];                          /* 1536 bytes */

int main(void)
{
    uint32_t n = PG_SHARED[0];
    for (int i = 3; i < 7; i++) PG_SHARED[i] = 0;
    rmii_init();
    host_irq(1);                                   /* ready */
    while (n--) {
        int len = rmii_recv(buf, sizeof buf);
        if (len == -1) { PG_SHARED[4]++; continue; }
        if (len < 0) { PG_SHARED[5]++; continue; }
        uint8_t *b = (uint8_t *)buf;
        for (int i = 0; i < 6; i++) { uint8_t t = b[i]; b[i] = b[6 + i]; b[6 + i] = t; }
        rmii_send(buf, (uint32_t)len - 4u);
        PG_SHARED[3]++;
    }
    while (RMII->STATUS & RMII_TX_BUSY) { }
    PG_SHARED[6] = RMII->STATUS;
    PG_SHARED[2] = 0x600D0017u;
    host_irq(0);
    pg_park();
}
