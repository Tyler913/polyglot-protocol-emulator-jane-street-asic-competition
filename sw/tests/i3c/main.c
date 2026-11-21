/*
 * Copyright (c) 2026 Tyler Hong
 * SPDX-License-Identifier: Apache-2.0
 *
 * i3c: MIPI I3C SDR controller on Wire Engine 0 (open-drain header at 1 MHz,
 * push-pull data at 12.5 MHz). Assigns a dynamic address with SETDASA, reads
 * the target's PID, BCR and DCR (direct CCCs), writes and reads a register
 * block with private transfers, then resets the address (RSTDAA) and checks
 * that the old address is no longer acknowledged. Used by test/test_i3c.py.
 *
 * Pins: SDA uio[0] (P0), SCL uio[1] (P1), pull-ups on the board.
 * Results (PG_SHARED): [0] 0x600D013C when done, [1] ACK bits (bit k = step k
 * acknowledged), [2] PID bytes 0..3, [3] PID bytes 4..5, [4] BCR | DCR << 8,
 * [5] private read: 4 bytes, [6] bytes read in the private read.
 */

#include "protocol.h"

#define STATIC_ADDR 0x50u
#define DYN_ADDR    0x30u

int main(void)
{
    i3c_t b;
    uint8_t d[8];
    uint32_t acks = 0;
    int n;

    for (int i = 0; i < 7; i++) PG_SHARED[i] = 0;
    i3c_init(&b, W0, PIN_UIO(0), PIN_UIO(1), 1000000, 12500000);
    host_irq(1);

    acks |= (uint32_t)i3c_ccc(&b, 0x06, 0, 0, 1) << 0;          /* RSTDAA */
    i3c_ccc(&b, 0x87, 0, 0, 0);                                 /* SETDASA (direct) */
    i3c_start(&b);
    if (i3c_header(&b, STATIC_ADDR, 0)) {
        acks |= 1u << 1;
        i3c_write(&b, DYN_ADDR << 1);
    }
    i3c_stop(&b);

    i3c_ccc(&b, 0x8D, 0, 0, 0);                                 /* GETPID */
    i3c_start(&b);
    n = i3c_read_msg(&b, DYN_ADDR, d, 8);
    if (n == 6) acks |= 1u << 2;
    PG_SHARED[2] = d[0] | (d[1] << 8) | ((uint32_t)d[2] << 16) | ((uint32_t)d[3] << 24);
    PG_SHARED[3] = d[4] | (d[5] << 8);
    i3c_ccc(&b, 0x8E, 0, 0, 0);                                 /* GETBCR */
    i3c_start(&b);
    if (i3c_read_msg(&b, DYN_ADDR, d, 1) == 1) acks |= 1u << 3;
    PG_SHARED[4] = d[0];
    i3c_ccc(&b, 0x8F, 0, 0, 0);                                 /* GETDCR */
    i3c_start(&b);
    if (i3c_read_msg(&b, DYN_ADDR, d, 1) == 1) acks |= 1u << 4;
    PG_SHARED[4] |= (uint32_t)d[0] << 8;

    i3c_start(&b);                                              /* private write: pointer 0x10, 3 bytes */
    if (i3c_header(&b, DYN_ADDR, 0)) {
        acks |= 1u << 5;
        i3c_write(&b, 0x10);
        i3c_write(&b, 0xA1);
        i3c_write(&b, 0xB2);
        i3c_write(&b, 0xC3);
    }
    i3c_stop(&b);
    i3c_start(&b);                                              /* private read from the pointer */
    n = i3c_read_msg(&b, DYN_ADDR, d, 8);
    PG_SHARED[5] = d[0] | (d[1] << 8) | ((uint32_t)d[2] << 16) | ((uint32_t)d[3] << 24);
    PG_SHARED[6] = (uint32_t)n;

    acks |= (uint32_t)i3c_ccc(&b, 0x06, 0, 0, 1) << 6;          /* RSTDAA */
    i3c_start(&b);
    if (i3c_header(&b, DYN_ADDR, 0)) acks |= 1u << 7;          /* must not be acknowledged now */
    i3c_stop(&b);

    PG_SHARED[1] = acks;
    PG_SHARED[0] = 0x600D013Cu;
    host_irq(0);
    pg_park();
}
