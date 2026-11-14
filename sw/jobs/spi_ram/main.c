/*
 * Copyright (c) 2026 Tyler Hong
 * SPDX-License-Identifier: Apache-2.0
 *
 * Job spi_ram: SPI controller (mode 0) writes 16 bytes to a 23x-style SPI RAM
 * and reads them back. CS is a GPIO pin.
 * Mailbox: [1] SCK, [2] MOSI, [3] CS, [4] MISO pin, [5] SCK Hz, [6] address.
 * Results: [8] 0x600D0000 | mismatches, [9..12] bytes read back, [14] start, [15] end.
 */

#include "job.h"

#define N 16

static uint8_t buf[2 + N], back[2 + N];
static int frame, i;

JOB_ENTRY
{
    PT_BEGIN(pt);
    mb[14] = now();
    pin_write(mb[3], 1);
    pin_output(mb[3]);
    wire_config(JOB_W, WIRE_SPI_CTRL(mb[1], mb[2], mb[4], 0, mb[5]));
    buf[1] = (uint8_t)mb[6];
    for (i = 0; i < N; i++) buf[2 + i] = (uint8_t)(0x3C + 17 * i);
    for (frame = 0; frame < 2; frame++) {               /* WRITE, then READ */
        buf[0] = frame ? 0x03 : 0x02;
        pin_write(mb[3], 0);
        for (i = 0; i < 2 + N; i++) {
            wire_put(JOB_W, buf[i]);
            PT_WAIT(pt, wire_rx_count(JOB_W) > 0);
            back[i] = (uint8_t)wire_get(JOB_W);
        }
        PT_WAIT(pt, tx_idle(JOB_W));
        pin_write(mb[3], 1);
    }
    uint32_t bad = 0;
    for (i = 0; i < N; i++) bad += back[2 + i] != buf[2 + i];
    for (i = 0; i < N / 4; i++)
        mb[9 + i] = back[2 + 4 * i] | (back[3 + 4 * i] << 8) |
                    ((uint32_t)back[4 + 4 * i] << 16) | ((uint32_t)back[5 + 4 * i] << 24);
    mb[8] = 0x600D0000u | bad;
    mb[15] = now();
    PT_END(pt);
}
