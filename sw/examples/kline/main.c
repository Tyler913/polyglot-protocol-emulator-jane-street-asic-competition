/*
 * Copyright (c) 2026 Tyler Hong
 * SPDX-License-Identifier: Apache-2.0
 *
 * kline: an OBD-II scan tool on the K-line, ISO 14230-4 (KWP2000) with fast
 * initialization, at 10400 baud 8N1, through a K-line transceiver (TX and
 * RX pins; RX reads the single wire back).
 *
 *  - Fast init: K low for 25 ms (TiniL), high for 25 ms (TWuP = 50 ms), by
 *    timed GPIO before the engine takes the TX pin; then
 *    StartCommunication (0x81) with functional addressing (0x33, tester 0xF1).
 *  - Tester bytes are P4 = 6 ms apart (start to start, timed; 5 ms
 *    minimum), each read back; the ECU's answer ends with its checksum
 *    (length from the format byte); P3 = 60 ms (55 ms minimum) before the
 *    next request.
 *  - Then service 0x01 (current data), PID 0x0C (engine speed) and 0x0D
 *    (vehicle speed).
 * Message: format (0xC0 | length: functional address), target, source,
 * data, checksum = sum of all bytes mod 256.
 *
 * Pins: TX uo[5] (P13), RX ui[2] (P18).
 * Results (PG_SHARED): [2] 0x600D000D when done (0xBAD0000D + step on a
 * failure), [3] key bytes of the StartCommunication reply, [4] engine speed
 * (rpm x 4, raw A*256+B), [5] vehicle speed (km/h), [6] ECU address.
 */

#include "protocol.h"

#define KL   W2
#define TXP  PIN_UO(5)
#define FMT  SER_FMT(8, SER_NONE, 1, 0)
#define MS   PG_US_TO_TICKS(1000)

static uint8_t rsp[16];
static uint32_t ecu;

/* Send a request (checksum added), read it back, then read the reply.
 * Returns the number of data bytes in rsp (after the header), or -1. */
static int kw_request(const uint8_t *d, uint32_t n)
{
    uint8_t m[8];
    uint32_t sum = 0, t = now();
    m[0] = (uint8_t)(0xC0u | n); m[1] = 0x33; m[2] = 0xF1;      /* functional, OBD, tester */
    for (uint32_t i = 0; i < n; i++) m[3 + i] = d[i];
    for (uint32_t i = 0; i < n + 3u; i++) sum += m[i];
    m[n + 3u] = (uint8_t)sum;
    for (uint32_t i = 0; i < n + 4u; i++) {          /* P4: 6 ms start to start */
        wait_until(t);
        ser_putc(KL, FMT, m[i]);
        t += 6u * MS;
        block_timeout(3u * MS);
        int b = ser_getc(KL, FMT);
        block_timeout(0);
        if (b != m[i]) return -1;                    /* read-back */
    }
    block_timeout(60u * MS);                         /* P2 max 50 ms */
    int b = ser_getc(KL, FMT);
    uint32_t len = 0, k = 0;
    if (b < 0 || (b & 0xC0) != 0x80u) goto fail;     /* physical addressing reply */
    len = (uint32_t)b & 0x3Fu;
    sum = (uint32_t)b;
    block_timeout(25u * MS);                         /* P1 max 20 ms */
    for (k = 0; k < len + 3u; k++) {                 /* target, source, data, checksum */
        b = ser_getc(KL, FMT);
        if (b < 0 || block_timed_out()) goto fail;
        if (k == 1u) ecu = (uint32_t)b;
        if (k >= 2u && k - 2u < sizeof rsp) rsp[k - 2u] = (uint8_t)b;
        if (k < len + 2u) sum += (uint32_t)b;
        else if ((uint8_t)sum != (uint8_t)b) goto fail;
    }
    block_timeout(0);
    return (int)len;
fail:
    block_timeout(0);
    return -1;
}

int main(void)
{
    static const uint8_t start_comm[1] = { 0x81 }, rpm[2] = { 0x01, 0x0C }, speed[2] = { 0x01, 0x0D };
    PG_SHARED[2] = 0xBAD0000Du;
    pin_write(TXP, 1);                               /* K idle high (GPIO owns TX) */
    host_irq(1);
    delay_ticks(300u * MS);                          /* W5: bus idle >= 300 ms before init */
    uint32_t t = now() + MS;
    pin_out_at(TXP, 0, t);                           /* fast init: TiniL 25 ms */
    pin_out_at(TXP, 1, t + 25u * MS);                /* TWuP 50 ms */
    timed_flush();
    ser_init(KL, TXP, PIN_UI(2), 10400, FMT);
    wait_until(t + 50u * MS);

    if (kw_request(start_comm, 1) != 3 || rsp[0] != 0xC1u) goto done;   /* C1 KB1 KB2 */
    PG_SHARED[3] = rsp[1] | (rsp[2] << 8);
    PG_SHARED[6] = ecu;
    PG_SHARED[2]++;
    delay_ticks(60u * MS);                           /* P3 */
    if (kw_request(rpm, 2) != 4 || rsp[0] != 0x41u || rsp[1] != 0x0Cu) goto done;
    PG_SHARED[4] = ((uint32_t)rsp[2] << 8) | rsp[3];
    PG_SHARED[2]++;
    delay_ticks(60u * MS);
    if (kw_request(speed, 2) != 3 || rsp[0] != 0x41u || rsp[1] != 0x0Du) goto done;
    PG_SHARED[5] = rsp[2];
    PG_SHARED[2] = 0x600D000Du;
done:
    host_irq(0);
    pg_park();
}
