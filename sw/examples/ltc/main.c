/*
 * Copyright (c) 2026 Tyler Hong
 * SPDX-License-Identifier: Apache-2.0
 *
 * ltc: SMPTE linear timecode at 30 fps. Wire Engine 0 sends LTC counting up
 * from a start time; Wire Engine 1 reads an incoming LTC stream (biphase mark
 * decoded by edge timing) and stores the times it reads. Used by
 * test/test_ltc.py.
 *
 * Pins: LTC out uio[0] (P0), LTC in ui[3] (P19).
 * Host parameters (PG_SHARED): [0] frames to send, [1] frames to read,
 * [2] start time h << 24 | m << 16 | s << 8 | f.
 * Results: [0] 0x600D017C when done, [1] frames read, [4+k] time k as
 * h << 24 | m << 16 | s << 8 | f (0xFFFFFFFF if the sync word was missing).
 */

#include "protocol.h"

int main(void)
{
    uint32_t nsend = PG_SHARED[0], nread = PG_SHARED[1], st = PG_SHARED[2];
    ltc_time_t t = {(uint8_t)(st >> 24), (uint8_t)(st >> 16), (uint8_t)(st >> 8), (uint8_t)st};
    ltc_time_t r;
    uint32_t x[5], nx = 0;

    if (nread > 32u) nread = 32u;
    ltc_init(W0, PIN_UIO(0), 0, 30, 1);
    ltc_init(W1, 0, PIN_UI(3), 30, 0);
    host_irq(1);
    uint32_t k = 0;
    for (uint32_t i = 0; i < nsend || k < nread; ) {
        if (i < nsend && WIRE_TX_LEVEL(W0->STATUS) <= 1u) {   /* room for a frame (3 words) */
            ltc_send(W0, t);
            i++;
            if (++t.f == 30u) { t.f = 0; if (++t.s == 60u) { t.s = 0; if (++t.m == 60u) { t.m = 0; t.h++; } } }
        }
        if (k < nread && wire_rx_count(W1)) {
            x[nx++] = wire_get(W1);
            if (nx == 5) {
                nx = 0;
                PG_SHARED[4 + k] = ltc_decode(x, &r) ? 0xFFFFFFFFu :
                                   ((uint32_t)r.h << 24 | (uint32_t)r.m << 16 | (uint32_t)r.s << 8 | r.f);
                PG_SHARED[1] = ++k;
            }
        }
        if (i >= nsend && k >= nread) break;
    }
    wire_flush(W0);
    PG_SHARED[0] = 0x600D017Cu;
    host_irq(0);
    pg_park();
}
