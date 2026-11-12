/*
 * Copyright (c) 2026 Tyler Hong
 * SPDX-License-Identifier: Apache-2.0
 *
 * onewire: a 1-Wire bus master made only from timed GPIO: every edge of the
 * reset pulse and of each time slot is scheduled with pin_oe_at() (open
 * drain: enabling the driver pulls the line low), so the slot timing is
 * exact whatever the compiler does. Reads the ROM code of the device on the
 * bus (READ ROM, 0x33) and checks its CRC-8.
 *
 * Pin: DQ on uio[6] (P6), with a pull-up.
 * Results (PG_SHARED): [1] presence (1 = device answered), [2..3] ROM code
 * (little endian), [4] 1 if the ROM CRC-8 is correct.
 */

#include "protocol.h"

#define DQ PIN_UIO(6)
#define US(x) PG_US_TO_TICKS(x)

/* Pull DQ low from t for `low` ticks, then release. */
static void pulse(uint32_t t, uint32_t low)
{
    pin_oe_at(DQ, 1, t);
    pin_oe_at(DQ, 0, t + low);
}

static int ow_reset(void)
{
    uint32_t t = now() + US(2);
    pulse(t, US(480));
    wait_until(t + US(480 + 70));          /* the device pulls low 15-60 us after release, for 60-240 us */
    int present = !pin_read(DQ);
    wait_until(t + US(960));
    return present;
}

static void ow_write_bit(int b)
{
    uint32_t t = now() + US(2);
    pulse(t, b ? US(6) : US(60));
    wait_until(t + US(70));                 /* slot 60 us + recovery */
}

static int ow_read_bit(void)
{
    uint32_t t = now() + US(2);
    pulse(t, US(6));
    wait_until(t + US(13));                 /* sample before 15 us */
    int b = pin_read(DQ);
    wait_until(t + US(70));
    return b;
}

static void ow_write(uint8_t x)
{
    for (int i = 0; i < 8; i++) ow_write_bit((x >> i) & 1);
}

static uint8_t ow_read(void)
{
    uint8_t x = 0;
    for (int i = 0; i < 8; i++) x |= (uint8_t)(ow_read_bit() << i);
    return x;
}

static uint8_t crc8_maxim(const uint8_t *d, int n)
{
    uint8_t c = 0;
    for (int i = 0; i < n; i++) {
        uint8_t x = d[i];
        for (int k = 0; k < 8; k++, x >>= 1) {
            uint8_t mix = (c ^ x) & 1;
            c >>= 1;
            if (mix) c ^= 0x8C;
        }
    }
    return c;
}

int main(void)
{
    uint8_t rom[8];

    pin_write(DQ, 0);                       /* open drain: output value 0, OE switches */
    pin_input(DQ);
    PG_SHARED[1] = (uint32_t)ow_reset();
    ow_write(0x33);                         /* READ ROM */
    for (int i = 0; i < 8; i++) rom[i] = ow_read();
    PG_SHARED[2] = rom[0] | (rom[1] << 8) | ((uint32_t)rom[2] << 16) | ((uint32_t)rom[3] << 24);
    PG_SHARED[3] = rom[4] | (rom[5] << 8) | ((uint32_t)rom[6] << 16) | ((uint32_t)rom[7] << 24);
    PG_SHARED[4] = crc8_maxim(rom, 7) == rom[7];
    host_irq(1);
    pg_park();
}
