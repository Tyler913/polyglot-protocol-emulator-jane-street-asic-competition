/*
 * Copyright (c) 2026 Tyler Hong
 * SPDX-License-Identifier: Apache-2.0
 *
 * canfd_rx: CAN FD (ISO) receiver on Wire Engine 0 that acknowledges good
 * frames (FD and classic): the engine's CAN FD unit sends the ACK itself when
 * the frame's CRC (and, for FD, the stuff count) checks. Used by test/test_canfd.py.
 *
 * Pins: TXD uio[0] (P0), RXD ui[2] (P18).
 * Host parameters (PG_SHARED): [0] frames to receive (1..12), [1] nominal bit
 * rate, [2] data bit rate.
 * Results: [0] 0x600D00FE when done, [1] frames received, per frame k:
 * [3+4k] ID, [4+4k] length | FD << 8 | BRS << 9 | CRC ok << 10,
 * [5+4k] checksum of the data (rotate left by 1, XOR each byte), [6+4k] the
 * CAN FD unit status for the frame.
 */

#include "protocol.h"

#define CAN W0

int main(void)
{
    static canfd_frame_t f;
    uint32_t n = PG_SHARED[0];

    if (n > 12u) n = 12u;
    canfd_init(CAN, PIN_UIO(0), PIN_UI(2), PG_SHARED[1], PG_SHARED[2]);
    host_irq(1);
    for (uint32_t k = 0; k < n; ) {
        if (canfd_recv(CAN, &f) < 0) continue;
        uint32_t sum = 0;
        for (uint32_t i = 0; i < f.len; i++) sum = ((sum << 1) | (sum >> 31)) ^ f.data[i];
        volatile uint32_t *r = &PG_SHARED[3 + 4 * k];
        r[0] = f.id;
        r[1] = f.len | ((uint32_t)f.fd << 8) | ((uint32_t)f.brs << 9) | ((uint32_t)f.crc_ok << 10);
        r[2] = sum;
        r[3] = f.status;
        PG_SHARED[1] = ++k;
    }
    PG_SHARED[0] = 0x600D00FEu;
    host_irq(0);
    pg_park();
}
