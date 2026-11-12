/*
 * Copyright (c) 2026 Tyler Hong
 * SPDX-License-Identifier: Apache-2.0
 *
 * spi_ram: SPI controller on Wire Engine 1 talking to a small SPI RAM
 * (23x-style commands: 0x02 WRITE addr data..., 0x03 READ addr -> data...,
 * 8-bit address). Writes a 16-byte pattern, reads it back and compares.
 *
 * Pins: SCK uo[2] (P10), MOSI uo[3] (P11), CS_N uo[4] (P12), MISO ui[1] (P17).
 *
 * Host parameters (PG_SHARED): [0] SPI mode 0..3, [1] SCK frequency in Hz
 * (0 = 1 MHz), [2] RAM address.
 * Results: [4] 0x600D0000 | mismatches (or 0x600DBAD0 | mismatches),
 * [8..11] the 16 bytes read back.
 */

#include "protocol.h"

#define SPI      W1
#define SCK_PIN  PIN_UO(2)
#define MOSI_PIN PIN_UO(3)
#define CS_PIN   PIN_UO(4)
#define MISO_PIN PIN_UI(1)

#define N 16

int main(void)
{
    uint32_t mode = PG_SHARED[0] & 3u;
    uint32_t hz   = PG_SHARED[1] ? PG_SHARED[1] : 1000000u;
    uint8_t  addr = (uint8_t)PG_SHARED[2];
    uint8_t  wbuf[2 + N], rbuf[2 + N];

    pin_write(CS_PIN, 1);
    wire_config(SPI, WIRE_SPI_CTRL(SCK_PIN, MOSI_PIN, MISO_PIN, mode, hz));

    wbuf[0] = 0x02;
    wbuf[1] = addr;
    for (int i = 0; i < N; i++) wbuf[2 + i] = (uint8_t)(0x3C + 17 * i);
    spi_select(CS_PIN);
    spi_transfer(SPI, wbuf, 0, 2 + N);
    spi_deselect(SPI, CS_PIN);

    wbuf[0] = 0x03;
    spi_select(CS_PIN);
    spi_transfer(SPI, wbuf, rbuf, 2 + N);    /* data bytes after cmd + addr are ignored by the RAM */
    spi_deselect(SPI, CS_PIN);

    uint32_t bad = 0;
    for (int i = 0; i < N; i++) if (rbuf[2 + i] != wbuf[2 + i]) bad++;
    for (int i = 0; i < N / 4; i++)
        PG_SHARED[8 + i] = rbuf[2 + 4 * i] | (rbuf[3 + 4 * i] << 8) |
                           ((uint32_t)rbuf[4 + 4 * i] << 16) | ((uint32_t)rbuf[5 + 4 * i] << 24);
    PG_SHARED[4] = (bad ? 0x600DBAD0u : 0x600D0000u) | bad;

    host_irq(1);
    pg_park();
}
