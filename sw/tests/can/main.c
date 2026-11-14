/*
 * Copyright (c) 2026 Tyler Hong
 * SPDX-License-Identifier: Apache-2.0
 *
 * can: CAN 2.0A at 500 kbit/s from Wire Engine 0. Sends a frame; if it
 * loses arbitration it receives the winner's frame and retries. Used by
 * test/test_can.py, whose bus model holds a second node.
 *
 * Pins: TXD uio[0] (P0), RXD ui[2] (P18). TXD is on a bidirectional pin
 * because those are high-impedance after reset (the transceiver's TXD
 * pull-up keeps the bus recessive while the program is loaded); the uo
 * outputs are 0 = dominant after reset.
 * Results (PG_SHARED): [0] 0x600D0003 when done, [1] attempts, [2] our frame
 * acknowledged, [3] frames received from others, [4] ID of the last one,
 * [5] its first 4 data bytes, [6] its CRC ok.
 */

#include "protocol.h"

#define CAN W0

int main(void)
{
    static const uint8_t msg[4] = {0xDE, 0xAD, 0xBE, 0xEF};
    can_frame_t f;

    for (int i = 0; i < 7; i++) PG_SHARED[i] = 0;
    can_init(CAN, PIN_UIO(0), PIN_UI(2), 500000);
    host_irq(1);
    delay_ticks(PG_US_TO_TICKS(30));                 /* bus idle (11 recessive bits) */

    for (;;) {
        PG_SHARED[1]++;
        CAN->STATUS = WIRE_ARB_LOST;                 /* clear */
        can_send(CAN, 0x123, msg, 4);
        for (;;) {                                   /* every frame on the bus, ours included */
            if (can_recv_ack(CAN, &f) < 0) continue;
            if (f.id == 0x123) break;                /* our own frame went out */
            PG_SHARED[3]++;
            PG_SHARED[4] = f.id;
            PG_SHARED[5] = f.data[0] | (f.data[1] << 8) | ((uint32_t)f.data[2] << 16) | ((uint32_t)f.data[3] << 24);
            PG_SHARED[6] = f.crc_ok;
            if (CAN->STATUS & WIRE_ARB_LOST) break;  /* we lost: retry after this frame */
        }
        if (!(CAN->STATUS & WIRE_ARB_LOST)) {
            PG_SHARED[2] = f.acked && f.crc_ok;
            break;
        }
        wire_tx_discard(CAN);                        /* drop the rest of our lost frame */
        delay_ticks(PG_US_TO_TICKS(8));              /* intermission */
    }
    PG_SHARED[0] = 0x600D0003u;
    host_irq(0);
    pg_park();
}
