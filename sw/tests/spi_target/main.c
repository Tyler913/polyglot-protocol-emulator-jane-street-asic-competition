/*
 * Copyright (c) 2026 Tyler Hong
 * SPDX-License-Identifier: Apache-2.0
 *
 * spi_target: Wire Engine 0 as an SPI target framed by chip select. Checks
 * the external clock, the CS gate and the frame events.
 *
 * Pins: SCK in ui[2] (P18), MOSI in ui[3] (P19), CS_N in ui[4] (P20),
 * MISO out uo[5] (P13).
 * Host parameters (PG_SHARED): [0] SPI mode.
 * Every frame answers 0xA5, 0x5A, then the inverse of each received byte,
 * two bytes later. Results: [1] frames seen, [2] bytes in the last frame,
 * [3] total bytes, [8..] the last frame's bytes (up to 32).
 */

#include "protocol.h"

#define SPI W0

int main(void)
{
    uint32_t mode = PG_SHARED[0] & 3u;
    volatile uint8_t *rx = (volatile uint8_t *)&PG_SHARED[8];
    uint32_t n = 0;

    PG_SHARED[1] = 0;
    PG_SHARED[2] = 0;
    PG_SHARED[3] = 0;
    wire_config(SPI, WIRE_SPI_TARGET(PIN_UI(2), PIN_UI(3), PIN_UO(5), PIN_UI(4), mode));
    wire_put(SPI, 0xA5);
    wire_put(SPI, 0x5A);
    host_irq(1);                                 /* ready: the controller may start */

    for (;;) {
        uint32_t w = wire_get(SPI);
        if (w == WIRE_EV_START) {
            n = 0;
        } else if (w == WIRE_EV_END) {
            PG_SHARED[1]++;
            PG_SHARED[2] = n;
            wire_put(SPI, 0xA5);                 /* the next frame's first two replies */
            wire_put(SPI, 0x5A);
        } else {
            if (n < 32) rx[n] = (uint8_t)w;
            n++;
            PG_SHARED[3]++;
            wire_put(SPI, (uint8_t)~w);
        }
    }
}
