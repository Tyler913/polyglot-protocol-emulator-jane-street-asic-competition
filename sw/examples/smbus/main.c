/*
 * Copyright (c) 2026 Tyler Hong
 * SPDX-License-Identifier: Apache-2.0
 *
 * smbus: SMBus 3.x host transactions with Packet Error Checking (PEC) on the
 * I2C controller of protocol.h. PEC is a CRC-8 (x^8 + x^2 + x + 1, init 0)
 * over every byte of the transaction, addresses included: Write Byte,
 * Write Word and Read Byte, each with its PEC, plus a Read Byte whose PEC
 * the device got wrong (detected).
 *
 * Pins: SDA uio[0] (P0), SCL uio[1] (P1), open drain; 100 kHz (SMBus).
 * Device address 0x50. Results (PG_SHARED): [0] write ACKs ok, [1] first
 * read byte, [2] its PEC ok, [3] second read's PEC ok (expected 0),
 * [4] 0x600D5B05.
 */

#include "protocol.h"

#define DEV 0x50u

static uint8_t pec;
static void pec_add(uint8_t b)
{
    pec ^= b;
    for (int i = 0; i < 8; i++) pec = (pec & 0x80u) ? (uint8_t)((pec << 1) ^ 0x07u) : (uint8_t)(pec << 1);
}

static int wr(i2c_t *b, uint8_t x) { pec_add(x); return i2c_write(b, x); }

static int smbus_write(i2c_t *b, uint8_t cmd, const uint8_t *d, int n)
{
    pec = 0;
    i2c_start(b);
    int ok = wr(b, DEV << 1) & wr(b, cmd);
    for (int i = 0; i < n; i++) ok &= wr(b, d[i]);
    ok &= i2c_write(b, pec);
    i2c_stop(b);
    return ok;
}

/* Read Byte with PEC: returns the byte, *ok = 1 if the device's PEC matches */
static uint8_t smbus_read_byte(i2c_t *b, uint8_t cmd, int *ok)
{
    pec = 0;
    i2c_start(b);
    wr(b, DEV << 1);
    wr(b, cmd);
    i2c_start(b);                                 /* repeated START */
    wr(b, (DEV << 1) | 1u);
    uint8_t v = i2c_read(b, 1);
    pec_add(v);
    uint8_t p = i2c_read(b, 0);                   /* NACK the last byte */
    i2c_stop(b);
    *ok = (p == pec);
    return v;
}

int main(void)
{
    i2c_t bus;
    int ok;
    static const uint8_t word[2] = { 0x34, 0x12 };   /* Write Word: low byte first */
    i2c_init(&bus, W0, PIN_UIO(0), PIN_UIO(1), 100000);
    int w = smbus_write(&bus, 0x10, (const uint8_t *)"\xA5", 1);
    delay_ticks(PG_US_TO_TICKS(200));                /* the EEPROM-like device's write cycle */
    w &= smbus_write(&bus, 0x20, word, 2);
    delay_ticks(PG_US_TO_TICKS(200));
    PG_SHARED[0] = (uint32_t)w;
    PG_SHARED[1] = smbus_read_byte(&bus, 0x30, &ok);
    PG_SHARED[2] = (uint32_t)ok;
    (void)smbus_read_byte(&bus, 0x40, &ok);
    PG_SHARED[3] = (uint32_t)ok;
    PG_SHARED[4] = 0x600D5B05u;
    host_irq(1);
    pg_park();
}
