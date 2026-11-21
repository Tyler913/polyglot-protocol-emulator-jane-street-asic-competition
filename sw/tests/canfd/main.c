/*
 * Copyright (c) 2026 Tyler Hong
 * SPDX-License-Identifier: Apache-2.0
 *
 * canfd: CAN FD (ISO) transmitter on Wire Engine 0: six frames, FD with and
 * without bit rate switch, 0 to 64 data bytes, base and extended IDs, plus one
 * classic CAN frame on the same bus. The chip's own RX follows each frame
 * with the CAN FD unit (rate switch, fixed stuff bits, CRC-17/21 and stuff
 * count check); its status is reported. (The RX words of a long frame are not
 * read back: the CPU is busy queuing TX words, so the RX FIFO overflows.)
 * Used by test/test_canfd.py.
 *
 * Pins: TXD uio[0] (P0), RXD ui[2] (P18).
 * Host parameters (PG_SHARED): [0] nominal bit rate, [1] data bit rate.
 * Results: [0] 0x600D00FD when done, [2+k] FDCTL after frame k (FDF, BRS,
 * delimiter, CRC ok, stuff count ok, DLC as decoded by the chip's own RX).
 */

#include "protocol.h"

#define CAN W0

int main(void)
{
    static uint8_t buf[64];
    uint32_t nominal = PG_SHARED[0], data = PG_SHARED[1];

    for (uint32_t i = 0; i < 64; i++) buf[i] = (uint8_t)(3u * i + 1u);
    canfd_init(CAN, PIN_UIO(0), PIN_UI(2), nominal, data);
    host_irq(1);
    delay_ticks(PG_US_TO_TICKS(30));                 /* bus idle */

    for (uint32_t k = 0; k < 7; k++) {
        switch (k) {
        case 0: canfd_send(CAN, 0x123, buf, 8, 1); break;
        case 1: canfd_send(CAN, 0x1ABCDEF | CAN_EFF, buf, 64, 1); break;
        case 2: canfd_send(CAN, 0x7FF, buf, 0, 1); break;
        case 3: canfd_send(CAN, 0x055, buf + 40, 20, 0); break;
        case 4: can_send(CAN, 0x0F0, buf, 3); break;
        case 5: {
            static const uint8_t z[12] = {0};
            canfd_send(CAN, 0x400, z, 12, 1);         /* long runs: many stuff bits */
            break;
        }
        default: canfd_send(CAN, 0x0AA, buf + 5, 48, 1); break;
        }
        wire_flush(CAN);                              /* the frame and its EOF are on the bus */
        while (wire_rx_count(CAN)) (void)wire_get(CAN);
        CAN->STATUS = WIRE_RX_OVERFLOW;
        PG_SHARED[2 + k] = CAN->FDCTL;
        delay_ticks(PG_US_TO_TICKS(10));
    }
    PG_SHARED[0] = 0x600D00FDu;
    host_irq(0);
    pg_park();
}
