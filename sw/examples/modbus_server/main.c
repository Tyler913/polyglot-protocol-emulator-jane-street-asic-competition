/*
 * Copyright (c) 2026 Tyler Hong
 * SPDX-License-Identifier: Apache-2.0
 *
 * modbus_server: a Modbus RTU server ("slave") with 16 holding registers,
 * on a lite engine, at the specification's default line settings: 19200
 * baud, 8 data bits, even parity, 1 stop bit (ser_*). Framing per "MODBUS
 * over Serial Line" V1.02: a frame ends after 3.5 characters of silence (the
 * blocking-read timeout); a gap of more than 1.5 characters inside a frame,
 * a parity error or a bad CRC-16 discards it; frames for other addresses
 * are ignored, and broadcasts (address 0) are executed without a reply.
 * Functions 0x03 (read holding registers), 0x06 (write single register) and
 * 0x10 (write multiple registers); exceptions 0x01 (illegal function) and
 * 0x02 (illegal data address).
 *
 * Pins: TX uo[2] (P10), RX ui[3] (P19), to an RS-485 transceiver.
 * Host parameters (PG_SHARED): [0] server address, [1] frames to handle
 * (each received frame counts, answered or not), [8..23] initial registers.
 * Results: [2] 0x600D0009 when done, [3] requests answered, [4] frames
 * discarded or ignored, [32..47] the registers at the end.
 */

#include "protocol.h"

#define MB   W2
#define FMT  SER_FMT(8, SER_EVEN, 1, 0)
#define CHAR (11u * 2604u)                  /* 11 bits at 19200 baud, in cycles */
#define T15  (CHAR * 3u / 2u)
#define T35  (CHAR * 7u / 2u)
#define NREG 16u

static uint16_t reg[NREG];
static uint8_t f[48];

static uint16_t crc16(const uint8_t *p, uint32_t n)
{
    uint16_t c = 0xFFFFu;
    while (n--) {
        c ^= *p++;
        for (int i = 0; i < 8; i++) c = (c & 1u) ? (uint16_t)((c >> 1) ^ 0xA001u) : (uint16_t)(c >> 1);
    }
    return c;
}

/* Receive one frame; returns its length, or -1 if it must be discarded */
static int mb_recv(void)
{
    int n = 0, ok = 1, b;
    uint32_t t, last;
    block_timeout(0);
    b = ser_getc(MB, FMT);                         /* first byte: wait for a frame */
    last = now();
    block_timeout(T35);
    for (;;) {
        if (b < 0 || n >= (int)sizeof f) ok = 0;   /* parity / framing error, or too long */
        else f[n++] = (uint8_t)b;
        b = ser_getc(MB, FMT);
        if (block_timed_out()) break;              /* 3.5 characters of silence: frame end */
        t = now();
        if (t - last > CHAR + T15) ok = 0;         /* start-to-start > 1 character + 1.5 of gap */
        last = t;
    }
    block_timeout(0);
    if (!ok || n < 4 || crc16(f, n - 2u) != (f[n - 2] | ((uint16_t)f[n - 1] << 8))) return -1;
    return n;
}

static void mb_send(uint32_t n)
{
    uint16_t c = crc16(f, n);
    f[n] = (uint8_t)c;
    f[n + 1] = (uint8_t)(c >> 8);
    for (uint32_t i = 0; i < n + 2u; i++) ser_putc(MB, FMT, f[i]);
}

/* Handle a request in f; returns the reply length (without CRC) */
static uint32_t mb_serve(int n)
{
    uint32_t fn = f[1], a = ((uint32_t)f[2] << 8) | f[3], q = ((uint32_t)f[4] << 8) | f[5];
    uint32_t ex = 0;
    if (fn == 0x03 && n == 8) {
        if (q < 1u || q > NREG || a + q > NREG) ex = 2;
        else {
            f[2] = (uint8_t)(2u * q);
            for (uint32_t i = 0; i < q; i++) {
                f[3 + 2 * i] = (uint8_t)(reg[a + i] >> 8);
                f[4 + 2 * i] = (uint8_t)reg[a + i];
            }
            return 3u + 2u * q;
        }
    } else if (fn == 0x06 && n == 8) {
        if (a >= NREG) ex = 2;
        else { reg[a] = (uint16_t)q; return 6; }   /* the reply echoes the request */
    } else if (fn == 0x10 && n >= 9 && (uint32_t)n == 9u + f[6]) {
        if (q < 1u || a + q > NREG || f[6] != 2u * q) ex = 2;
        else {
            for (uint32_t i = 0; i < q; i++) reg[a + i] = (uint16_t)((f[7 + 2 * i] << 8) | f[8 + 2 * i]);
            return 6;                              /* address, function, start, quantity */
        }
    } else ex = 1;
    f[1] = (uint8_t)(fn | 0x80u);
    f[2] = (uint8_t)ex;
    return 3;
}

int main(void)
{
    uint32_t me = PG_SHARED[0], frames = PG_SHARED[1];
    for (uint32_t i = 0; i < NREG; i++) reg[i] = (uint16_t)PG_SHARED[8 + i];
    PG_SHARED[3] = PG_SHARED[4] = 0;
    ser_init(MB, PIN_UO(2), PIN_UI(3), 19200, FMT);
    host_irq(1);                                   /* ready */
    while (frames--) {
        int n = mb_recv();
        if (n < 0 || (f[0] != me && f[0] != 0)) { PG_SHARED[4]++; continue; }
        uint32_t r = mb_serve(n);
        if (f[0] == 0) { PG_SHARED[4]++; continue; }   /* broadcast: no reply */
        mb_send(r);
        PG_SHARED[3]++;
    }
    wire_flush(MB);
    for (uint32_t i = 0; i < NREG; i++) PG_SHARED[32 + i] = reg[i];
    PG_SHARED[2] = 0x600D0009u;
    host_irq(0);
    pg_park();
}
