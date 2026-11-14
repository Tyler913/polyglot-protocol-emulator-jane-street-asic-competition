/*
 * Copyright (c) 2026 Tyler Hong
 * SPDX-License-Identifier: Apache-2.0
 *
 * can_rx: CAN 2.0A receiver at 500 kbit/s on Wire Engine 0 that acknowledges
 * every good frame (receiver ACK in the ACK slot, can_recv_ack). Used by
 * test/test_can.py; the bus model's node sends the frames.
 *
 * Pins: TXD uio[0] (P0), RXD ui[2] (P18).
 * Host parameters (PG_SHARED): [0] number of frames to receive (1..6).
 * Results: [0] 0x600D0004 when done, [1] frames received, then per frame k:
 * [2+3k] ID, [3+3k] DLC | CRC ok << 8, [4+3k] first 4 data bytes.
 */

#include "protocol.h"

#define CAN W0

int main(void)
{
    uint32_t n = PG_SHARED[0];
    can_frame_t f;

    if (n > 6u) n = 6u;
    can_init(CAN, PIN_UIO(0), PIN_UI(2), 500000);
    host_irq(1);                                     /* ready */
    for (uint32_t k = 0; k < n; ) {
        if (can_recv_ack(CAN, &f) < 0) continue;
        volatile uint32_t *r = &PG_SHARED[2 + 3 * k];
        r[0] = f.id;
        r[1] = f.dlc | ((uint32_t)f.crc_ok << 8);
        r[2] = f.data[0] | (f.data[1] << 8) | ((uint32_t)f.data[2] << 16) | ((uint32_t)f.data[3] << 24);
        PG_SHARED[1] = ++k;
    }
    PG_SHARED[0] = 0x600D0004u;
    host_irq(0);
    pg_park();
}
