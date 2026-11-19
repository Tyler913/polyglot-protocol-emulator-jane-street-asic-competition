/*
 * Copyright (c) 2026 Tyler Hong
 * SPDX-License-Identifier: Apache-2.0
 *
 * dht22: reads a DHT22 / AM2302 temperature and humidity sensor over its
 * single open-drain data line (AOSONG single-bus). The host pulls the line
 * low for at least 1 ms and releases it; the sensor answers low 80 us, high
 * 80 us, then 40 bits, each a 50 us low followed by a high of 26..28 us (0)
 * or 70 us (1), MSB first: humidity x 10, temperature x 10 (bit 15 = minus),
 * checksum = sum of the first four bytes.
 * No engine: the capture unit time-stamps every falling edge, and a bit is
 * 1 when its falling edge comes more than 100 us after the previous one.
 *
 * Pin: uio[3] (P3), with a pull-up.
 * Host parameters (PG_SHARED): [0] readings (1..4).
 * Results: [2] 0x600D000F when done; reading k: [8 + 2k] humidity x 10
 * (low half) | temperature x 10, signed (high half), [9 + 2k] status:
 * 1 = ok, 2 = checksum error, 3 = no answer / timeout.
 */

#include "protocol.h"

#define PIN PIN_UIO(3)
#define US  PG_US_TO_TICKS(1)

static uint32_t dht_read(uint32_t *out)
{
    uint32_t t, prev, data[2] = {0, 0}, bits = 0;
    pin_write(PIN, 0);
    pin_output(PIN);                               /* start: low for 1.1 ms */
    delay_ticks(1100u * US);
    capture_arm(PIN, EDGE_FALL);                   /* the sensor's first falling edge */
    pin_input(PIN);                                /* release: the pull-up takes the line high */
    block_timeout(300u * US);
    prev = capture_wait();                         /* response start (low 80 us) */
    if (block_timed_out()) goto timeout;
    capture_arm(PIN, EDGE_FALL);
    t = capture_wait();                            /* first bit's falling edge (after 80 + 80 us) */
    if (block_timed_out()) goto timeout;
    prev = t;
    for (bits = 0; bits < 40u; bits++) {
        capture_arm(PIN, EDGE_FALL);
        t = capture_wait();                        /* the next bit's (or the end's) falling edge */
        if (block_timed_out()) goto timeout;
        uint32_t b = (t - prev) > 100u * US;
        prev = t;
        if (bits < 32u) data[0] = (data[0] << 1) | b; else data[1] = (data[1] << 1) | b;
    }
    block_timeout(0);
    uint32_t sum = (data[0] >> 24) + (data[0] >> 16 & 0xFFu) + (data[0] >> 8 & 0xFFu) + (data[0] & 0xFFu);
    uint32_t hum = data[0] >> 16, tmp = data[0] & 0x7FFFu;
    if (data[0] & 0x8000u) tmp = (uint32_t)-(int32_t)tmp;
    *out = (hum & 0xFFFFu) | (tmp << 16);
    return ((sum & 0xFFu) == data[1]) ? 1u : 2u;
timeout:
    block_timeout(0);
    return 3;
}

int main(void)
{
    uint32_t n = PG_SHARED[0];
    if (n < 1u || n > 4u) n = 1u;
    host_irq(1);                                   /* ready */
    for (uint32_t k = 0; k < n; k++) {
        uint32_t v = 0;
        PG_SHARED[9 + 2 * k] = dht_read(&v);
        PG_SHARED[8 + 2 * k] = v;
        delay_ticks(2000u * US);                   /* (a real DHT22 needs 2 s between readings) */
    }
    PG_SHARED[2] = 0x600D000Fu;
    host_irq(0);
    pg_park();
}
