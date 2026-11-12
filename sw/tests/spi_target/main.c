/*
 * Copyright (c) 2026 Tyler Hong
 * SPDX-License-Identifier: Apache-2.0
 *
 * spi_target: Wire Engine 0 clocked by an external SPI controller (the
 * testbench). Checks the external-clock path.
 *
 * Pins: SCK in ui[2] (P18), MOSI in ui[3] (P19), MISO out uo[5] (P13).
 * Host parameters (PG_SHARED): [0] SPI mode, [1] number of bytes n (<= 32).
 * The engine answers 0xA5, 0x5A, then the inverse of each received byte,
 * two bytes later. Results: [0] 0x600D0002 when done, [8..] received bytes.
 */

#include "protocol.h"

#define SPI W0

int main(void)
{
    uint32_t mode = PG_SHARED[0] & 3u;
    uint32_t n    = PG_SHARED[1];
    volatile uint8_t *rx = (volatile uint8_t *)&PG_SHARED[8];

    wire_config(SPI, WIRE_SPI_TARGET(PIN_UI(2), PIN_UI(3), PIN_UO(5), mode));
    wire_put(SPI, 0xA5);
    wire_put(SPI, 0x5A);
    host_irq(1);                                 /* ready: the controller may start */

    for (uint32_t i = 0; i < n; i++) {
        uint8_t b = (uint8_t)wire_get(SPI);
        rx[i] = b;
        wire_put(SPI, (uint8_t)~b);
    }

    PG_SHARED[0] = 0x600D0002u;
    host_irq(0);
    pg_park();
}
