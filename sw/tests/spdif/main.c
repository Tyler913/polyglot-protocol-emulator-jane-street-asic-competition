/*
 * Copyright (c) 2026 Tyler Hong
 * SPDX-License-Identifier: Apache-2.0
 *
 * spdif: S/PDIF (IEC 60958) transmitter on Wire Engine 0 and receiver on Wire
 * Engine 1, biphase mark in hardware. Used by test/test_spdif.py.
 *
 * Pins: TX uio[0] (P0), RX ui[3] (P19).
 * Host parameters (PG_SHARED): [0] mode (1 = send, 2 = receive), [1] sample
 * rate fs, [2] frames to send / subframes to receive (receive: up to 56).
 * Send: frame i carries A = 0x123456 + i * 0x0A0B0C, B = ~A (24 bits), C = 1
 * in frames 0 and 2, block start (B) every 192 frames.
 * Results: [0] 0x600D05D1 when done, [1] subframes received, [8..] the raw
 * RX words (type in [1:0], slots 4..31 in [29:2]).
 */

#include "protocol.h"

int main(void)
{
    uint32_t mode = PG_SHARED[0], fs = PG_SHARED[1], n = PG_SHARED[2];

    if (mode == 1u) {
        spdif_tx_init(W0, PIN_UIO(0), fs);
        host_irq(1);
        for (uint32_t i = 0; i < n; i++) {
            uint32_t a = 0x123456u + i * 0x0A0B0Cu, c = (i == 0 || i == 2) ? 4u : 0u;
            spdif_put(W0, (i % 192u) ? SPDIF_M : SPDIF_B, a, c);
            spdif_put(W0, SPDIF_W, ~a, c);
        }
        (void)W0->FLUSH;                              /* wait until it is all on the line */
    } else {
        if (n > 56u) n = 56u;
        spdif_rx_init(W1, PIN_UI(3), fs);
        host_irq(1);
        for (uint32_t i = 0; i < n; i++) {
            PG_SHARED[8 + i] = spdif_get(W1);
            PG_SHARED[1] = i + 1u;
        }
    }
    PG_SHARED[0] = 0x600D05D1u;
    host_irq(0);
    pg_park();
}
