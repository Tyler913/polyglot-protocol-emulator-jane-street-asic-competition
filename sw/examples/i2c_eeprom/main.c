/*
 * Copyright (c) 2026 Tyler Hong
 * SPDX-License-Identifier: Apache-2.0
 *
 * i2c_eeprom: I2C controller on Wire Engine 0 talking to a 24C02-style
 * EEPROM at address 0x50 (8-bit word address, 8-byte pages).
 *   1. probes an absent address (0x51): expects NACK
 *   2. page-writes 8 bytes at word address 0x10
 *   3. ACK-polls until the EEPROM finishes its write cycle
 *   4. random-reads the 8 bytes back (repeated START) and compares
 *
 * Pins: SDA uio[0] (P0), SCL uio[1] (P1), both open drain with pull-ups.
 * Host parameters (PG_SHARED): [0] SCL frequency in Hz (0 = 400 kHz).
 * Results: [2] ACK from the absent address (expect 0), [3] ACK polls,
 * [4] 0x600D0000 | mismatches (0x600DBAD0 | n on error), [8..9] bytes read.
 */

#include "protocol.h"

#define EEPROM 0x50
#define WADDR  0x10
#define N      8

int main(void)
{
    uint32_t hz = PG_SHARED[0] ? PG_SHARED[0] : 400000u;
    uint8_t data[N], back[N];
    i2c_t bus;

    i2c_init(&bus, W0, PIN_UIO(0), PIN_UIO(1), hz);
    for (int i = 0; i < N; i++) data[i] = (uint8_t)(0xA0 + 7 * i);

    /* 1. nobody at 0x51 */
    i2c_start(&bus);
    PG_SHARED[2] = (uint32_t)i2c_write(&bus, (EEPROM + 1) << 1);
    i2c_stop(&bus);

    /* 2. page write */
    int ok = 1;
    i2c_start(&bus);
    ok &= i2c_write(&bus, EEPROM << 1);
    ok &= i2c_write(&bus, WADDR);
    for (int i = 0; i < N; i++) ok &= i2c_write(&bus, data[i]);
    i2c_stop(&bus);

    /* 3. ACK polling: the EEPROM ignores its address while it writes */
    uint32_t polls = 0;
    for (;;) {
        polls++;
        i2c_start(&bus);
        if (i2c_write(&bus, EEPROM << 1)) break;
        i2c_stop(&bus);
    }
    PG_SHARED[3] = polls;

    /* 4. random read: set the word address, repeated START, read */
    ok &= i2c_write(&bus, WADDR);
    i2c_start(&bus);
    ok &= i2c_write(&bus, (EEPROM << 1) | 1);
    for (int i = 0; i < N; i++) back[i] = i2c_read(&bus, i < N - 1);
    i2c_stop(&bus);

    uint32_t bad = ok ? 0 : 0x80;
    for (int i = 0; i < N; i++) if (back[i] != data[i]) bad++;
    PG_SHARED[8] = back[0] | (back[1] << 8) | ((uint32_t)back[2] << 16) | ((uint32_t)back[3] << 24);
    PG_SHARED[9] = back[4] | (back[5] << 8) | ((uint32_t)back[6] << 16) | ((uint32_t)back[7] << 24);
    PG_SHARED[4] = (bad ? 0x600DBAD0u : 0x600D0000u) | bad;

    host_irq(1);
    pg_park();
}
