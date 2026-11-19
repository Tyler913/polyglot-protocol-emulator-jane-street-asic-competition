/*
 * Copyright (c) 2026 Tyler Hong
 * SPDX-License-Identifier: Apache-2.0
 *
 * can_b: CAN 2.0B node at 500 kbit/s on Wire Engine 0: standard and 29-bit
 * extended identifiers (SAE J1939 uses the latter). Optionally sends one
 * frame (retrying after a lost arbitration), then receives and acknowledges
 * frames of both formats. Used by test/test_can.py.
 *
 * Pins: TXD uio[0] (P0), RXD ui[2] (P18).
 * Host parameters (PG_SHARED): [0] frames to receive (0..6), [1] identifier
 * to send (CAN_EFF set for extended; 0 = send nothing), 8 data bytes
 * 0x10, 0x21, ... 0x87.
 * Results: [0] 0x600D0006 when done, [1] frames received, then per frame k:
 * [2+3k] ID, [3+3k] DLC | CRC ok << 8, [4+3k] first 4 data bytes;
 * [20] send attempts, [21] our frame acknowledged with a good CRC.
 */

#include "protocol.h"

#define CAN W0

static void store(uint32_t k, const can_frame_t *f)
{
    volatile uint32_t *r = &PG_SHARED[2 + 3 * k];
    r[0] = f->id;
    r[1] = f->dlc | ((uint32_t)f->crc_ok << 8);
    r[2] = f->data[0] | (f->data[1] << 8) | ((uint32_t)f->data[2] << 16) | ((uint32_t)f->data[3] << 24);
}

int main(void)
{
    uint32_t n = PG_SHARED[0], id = PG_SHARED[1], k = 0;
    uint8_t msg[8];
    can_frame_t f;

    if (n > 6u) n = 6u;
    for (int i = 0; i < 8; i++) msg[i] = (uint8_t)(0x10 + 0x11 * i);
    PG_SHARED[20] = PG_SHARED[21] = 0;
    can_init_b(CAN, PIN_UIO(0), PIN_UI(2), 500000);
    host_irq(1);                                     /* ready */
    if (id) {
        delay_ticks(PG_US_TO_TICKS(30));             /* bus idle (11 recessive bits) */
        for (;;) {
            PG_SHARED[20]++;
            CAN->STATUS = WIRE_ARB_LOST;             /* clear */
            can_send(CAN, id, msg, 8);
            for (;;) {                               /* every frame on the bus, ours included */
                if (can_recv_b(CAN, &f, 1) < 0) continue;
                if (f.id == id) break;               /* our own frame went out */
                if (k < n) { store(k, &f); PG_SHARED[1] = ++k; }
                if (CAN->STATUS & WIRE_ARB_LOST) break;
            }
            if (!(CAN->STATUS & WIRE_ARB_LOST)) {
                PG_SHARED[21] = f.acked && f.crc_ok;
                break;
            }
            wire_tx_discard(CAN);                    /* drop the rest of our lost frame */
            delay_ticks(PG_US_TO_TICKS(8));          /* intermission */
        }
    }
    while (k < n) {
        if (can_recv_b(CAN, &f, 1) < 0) continue;
        store(k, &f);
        PG_SHARED[1] = ++k;
    }
    PG_SHARED[0] = 0x600D0006u;
    host_irq(0);
    pg_park();
}
