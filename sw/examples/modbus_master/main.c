/*
 * Copyright (c) 2026 Tyler Hong
 * SPDX-License-Identifier: Apache-2.0
 *
 * modbus_master: a Modbus RTU client ("master") on a UART engine. RTU frames
 * are address, function, data and a CRC-16 (polynomial 0xA001 reflected,
 * initial 0xFFFF, low byte first), sent without gaps; a frame ends with
 * 3.5 character times of silence, which the receive side detects with the
 * blocking-read timeout. Writes holding register 5, then reads registers
 * 0..3 back, then reads an address the server does not have (exception).
 *
 * Pins: TX uo[1] (P9), RX ui[0] (P16), to an RS-485 transceiver; 115200 baud.
 * Server address 0x11.
 * Results (PG_SHARED): [0] write echoed ok, [1..4] registers 0..3 (or
 * 0xFFFFFFFF), [5] exception code of the third request, [6] 0x600D0B05.
 */

#include "protocol.h"

#define MB     W0
#define BAUD   115200u
#define SERVER 0x11u

static uint16_t crc16(const uint8_t *p, int n)
{
    uint16_t c = 0xFFFFu;
    while (n--) {
        c ^= *p++;
        for (int i = 0; i < 8; i++) c = (c & 1u) ? (uint16_t)((c >> 1) ^ 0xA001u) : (uint16_t)(c >> 1);
    }
    return c;
}

static void mb_send(uint8_t *f, int n)
{
    uint16_t c = crc16(f, n);
    f[n] = (uint8_t)c;
    f[n + 1] = (uint8_t)(c >> 8);
    for (int i = 0; i < n + 2; i++) uart_putc(MB, f[i]);
}

/* Receive a frame: bytes until 3.5 character times (> 19200 baud: 1.75 ms) of silence.
 * Returns its length without the CRC, or -1 if the CRC is wrong. */
static int mb_recv(uint8_t *f, int max, uint32_t first_timeout)
{
    int n = 0;
    block_timeout(first_timeout);
    for (;;) {
        int c = uart_getc(MB);
        if (block_timed_out()) break;
        if (n < max) f[n++] = (uint8_t)c;
        block_timeout(PG_US_TO_TICKS(1750));
    }
    block_timeout(0);
    if (n < 4 || crc16(f, n - 2) != (f[n - 2] | ((uint16_t)f[n - 1] << 8))) return -1;
    return n - 2;
}

static int mb_request(uint8_t *f, int n, int max)
{
    delay_ticks(PG_US_TO_TICKS(1750));   /* 3.5 characters of silence before a frame */
    mb_send(f, n);
    wire_flush(MB);
    return mb_recv(f, max, PG_US_TO_TICKS(100000));
}

int main(void)
{
    uint8_t f[40];
    wire_config(MB, WIRE_UART_8N1(PIN_UO(1), PIN_UI(0), BAUD));

    /* function 0x06: write single register 5 = 0x1234; the reply echoes the request */
    static const uint8_t wr[6] = { SERVER, 0x06, 0x00, 0x05, 0x12, 0x34 };
    for (int i = 0; i < 6; i++) f[i] = wr[i];
    int n = mb_request(f, 6, sizeof f), ok = (n == 6);
    for (int i = 0; ok && i < 6; i++) ok = (f[i] == wr[i]);
    PG_SHARED[0] = (uint32_t)ok;

    /* function 0x03: read holding registers 0..3 */
    static const uint8_t rd[6] = { SERVER, 0x03, 0x00, 0x00, 0x00, 0x04 };
    for (int i = 0; i < 6; i++) f[i] = rd[i];
    n = mb_request(f, 6, sizeof f);
    for (int i = 0; i < 4; i++)
        PG_SHARED[1 + i] = (n == 11 && f[1] == 0x03 && f[2] == 8)
                               ? ((uint32_t)f[3 + 2 * i] << 8) | f[4 + 2 * i] : 0xFFFFFFFFu;

    /* read register 0x0100, which the server does not have: exception 0x83, code 2 */
    static const uint8_t bad[6] = { SERVER, 0x03, 0x01, 0x00, 0x00, 0x01 };
    for (int i = 0; i < 6; i++) f[i] = bad[i];
    n = mb_request(f, 6, sizeof f);
    PG_SHARED[5] = (n == 3 && f[1] == 0x83) ? f[2] : 0xFFu;

    PG_SHARED[6] = 0x600D0B05u;
    host_irq(1);
    pg_park();
}
