/*
 * Copyright (c) 2026 Tyler Hong
 * SPDX-License-Identifier: Apache-2.0
 *
 * usb_phy: the USB low-speed packet layer built from Wire Engine 1 (NRZI,
 * stuffing after six ones, CRC16, differential D+/D-, SE0 end of packet,
 * hardware-timed handshake). Used by test/test_usb.py.
 *   1. sends a DATA0 packet with an 8-byte payload (CRC16 by the engine)
 *   2. queues an ACK that the engine sends by itself a fixed turnaround
 *      after the next received packet ends, then receives a DATA1 packet
 *      from the host and records it with the hardware CRC residual
 *
 * Pins: D- uio[4] (P4), D+ uio[5] (P5). Low speed: idle J = D- high.
 * Results (PG_SHARED): [1] RX byte count, [2] RX CRC residual (0x800D = OK),
 * [3] STATUS after the packet, [8..] received bytes (PID, data, CRC; the engine consumes SYNC).
 */

#include "protocol.h"

#define USB    W1
#define DM     PIN_UIO(4)
#define DP     PIN_UIO(5)

static const uint8_t payload[8] = {0x80, 0x06, 0x00, 0x01, 0xFF, 0xFF, 0x40, 0x00};

int main(void)
{
    volatile uint8_t *rx = (volatile uint8_t *)&PG_SHARED[8];
    uint32_t n = 0;

    usb_ls_init(USB, DM, DP);

    /* 1. DATA0 with payload */
    usb_send_data(USB, USB_PID_DATA0, payload, 8, 0);
    wire_flush(USB);

    /* 2. hardware handshake after the next packet, then receive it */
    usb_queue_handshake(USB, USB_PID_ACK);
    host_irq(1);
    uint8_t buf[32];
    while (!usb_rx_word(wire_get(USB), buf, sizeof buf, &n)) { }
    for (uint32_t i = 0; i < n && i < sizeof buf; i++) rx[i] = buf[i];
    PG_SHARED[1] = n;
    PG_SHARED[2] = USB->RXCRC;
    PG_SHARED[3] = USB->STATUS;
    wire_flush(USB);
    host_irq(0);
    pg_park();
}
