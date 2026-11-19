/*
 * Copyright (c) 2026 Tyler Hong
 * SPDX-License-Identifier: Apache-2.0
 *
 * lin_slave: a LIN 2.x slave node (ISO 17987) on a lite engine at 19200
 * baud. A header is BREAK (at least 13 dominant bits, which arrives as an
 * all-zero frame word: ser_is_break), sync 0x55 and the protected ID (6-bit
 * ID + 2 parity bits). The node answers the IDs it publishes (data +
 * enhanced checksum over PID and data), receives the IDs it subscribes to,
 * and serves the diagnostic "Read by Identifier" request for the product
 * identification (master request 0x3C, slave response 0x3D, classic
 * checksum). Through the transceiver RX reads the bus back, so every byte the
 * node sends is checked (bit errors abort the response, as the standard
 * requires).
 *
 * Pins: TXD uo[3] (P11), RXD ui[4] (P20), to a LIN transceiver.
 * Published: ID 0x21 (2 bytes: a counter, then 0xA5), ID 0x22 (4 bytes:
 * PG_SHARED[8]). Subscribed: ID 0x10 (4 bytes). NAD 0x0A, supplier ID
 * 0x1234, function ID 0x5678, variant 0x01.
 * Host parameters: [0] headers to handle, [8] ID 0x22 data.
 * Results: [2] 0x600D000A when done, [3] responses sent, [4] headers
 * ignored or bad, [5] last 0x10 data (little endian), [6] 0x10 frames with
 * a good checksum, [7] read-back (bit) errors.
 */

#include "protocol.h"

#define LIN W2
#define FMT SER_FMT(8, SER_NONE, 1, 0)

static uint8_t lin_pid(uint32_t id)
{
    uint32_t p0 = (id ^ (id >> 1) ^ (id >> 2) ^ (id >> 4)) & 1u;
    uint32_t p1 = ~((id >> 1) ^ (id >> 3) ^ (id >> 4) ^ (id >> 5)) & 1u;
    return (uint8_t)((id & 0x3Fu) | (p0 << 6) | (p1 << 7));
}

static uint8_t lin_sum(const uint8_t *d, uint32_t n, uint32_t s)
{
    while (n--) { s += *d++; if (s > 0xFFu) s -= 0xFFu; }
    return (uint8_t)~s;
}

/* Send a response; each byte is read back from the bus */
static void lin_respond(const uint8_t *d, uint32_t n, uint32_t start)
{
    uint8_t c = lin_sum(d, n, start);
    for (uint32_t i = 0; i <= n; i++) {
        uint32_t b = i < n ? d[i] : c;
        ser_putc(LIN, FMT, b);
        if (ser_getc(LIN, FMT) != (int)b) { PG_SHARED[7]++; return; }   /* bit error: stop */
    }
    PG_SHARED[3]++;
}

/* Receive n data bytes + checksum; 1 if the checksum is good */
static int lin_receive(uint8_t *d, uint32_t n, uint32_t start)
{
    for (uint32_t i = 0; i < n; i++) {
        int b = ser_getc(LIN, FMT);
        if (b < 0) return 0;
        d[i] = (uint8_t)b;
    }
    return ser_getc(LIN, FMT) == lin_sum(d, n, start);
}

int main(void)
{
    uint32_t headers = PG_SHARED[0];
    uint8_t d[8], cnt = 0;
    int diag = 0;                                    /* a Read by Identifier answer is pending */

    for (int i = 3; i < 8; i++) PG_SHARED[i] = 0;
    ser_init(LIN, PIN_UO(3), PIN_UI(4), 19200, FMT);
    host_irq(1);                                     /* ready */
    while (headers) {
        uint32_t w = LIN->RX;
        if (!ser_is_break(FMT, w)) continue;         /* wait for a BREAK */
        headers--;
        block_timeout(30u * 2604u);                  /* the header must follow */
        int sync = ser_getc(LIN, FMT), p = ser_getc(LIN, FMT);
        block_timeout(0);
        if (sync != 0x55 || p < 0 || lin_pid((uint32_t)p & 0x3Fu) != p) { PG_SHARED[4]++; continue; }
        uint32_t id = (uint32_t)p & 0x3Fu;
        block_timeout(200u * 2604u);                 /* a frame's response must come in time */
        if (id == 0x21u) {
            d[0] = cnt++; d[1] = 0xA5;
            lin_respond(d, 2, (uint32_t)p);
        } else if (id == 0x22u) {
            uint32_t v = PG_SHARED[8];
            for (int i = 0; i < 4; i++) d[i] = (uint8_t)(v >> (8 * i));
            lin_respond(d, 4, (uint32_t)p);
        } else if (id == 0x10u) {
            if (lin_receive(d, 4, (uint32_t)p)) {
                PG_SHARED[5] = d[0] | (d[1] << 8) | ((uint32_t)d[2] << 16) | ((uint32_t)d[3] << 24);
                PG_SHARED[6]++;
            }
        } else if (id == 0x3Cu) {                    /* master request: NAD, PCI, SID, ... */
            diag = lin_receive(d, 8, 0) && (d[0] == 0x0A || d[0] == 0x7F) && d[1] == 0x06 &&
                   d[2] == 0xB2 && d[3] == 0x00 &&
                   (d[4] | (d[5] << 8)) == 0x1234 && ((d[6] | (d[7] << 8)) == 0x5678 || (d[6] & d[7]) == 0xFF);
        } else if (id == 0x3Du && diag) {            /* slave response: positive, product ID */
            static const uint8_t r[8] = { 0x0A, 0x06, 0xF2, 0x34, 0x12, 0x78, 0x56, 0x01 };
            lin_respond(r, 8, 0);
            diag = 0;
        } else {
            PG_SHARED[4]++;                          /* not ours */
        }
        block_timeout(0);
    }
    PG_SHARED[2] = 0x600D000Au;
    host_irq(0);
    pg_park();
}
