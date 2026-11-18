/*
 * Copyright (c) 2026 Tyler Hong
 * SPDX-License-Identifier: Apache-2.0
 *
 * lin_master: a LIN 2.x master node (ISO 17987) on a UART-configured lite
 * Wire Engine. The only thing LIN adds to a UART is the BREAK: 13 dominant
 * bits and a recessive delimiter, which is one 14-bit word. Through the LIN
 * transceiver the engine's RX reads the bus back, so the master checks every
 * byte it sends (bit errors) and receives slave responses the same way.
 *
 * Pins: TXD uo[3] (P11), RXD ui[1] (P17), to a LIN transceiver; 19200 baud.
 * Frames: ID 0x10 published by the master (4 bytes, enhanced checksum),
 * ID 0x21 answered by a slave (2 bytes), ID 0x3C diagnostic master request
 * (8 bytes, classic checksum).
 * Results (PG_SHARED): [0] bytes of the 0x21 response (little endian),
 * [1] 0x21 checksum ok, [2] read-back errors, [3] 0x600D0011 when done.
 */

#include "protocol.h"

#define LIN W2

static uint8_t lin_pid(uint8_t id)
{
    uint8_t p0 = (id ^ (id >> 1) ^ (id >> 2) ^ (id >> 4)) & 1u;
    uint8_t p1 = ~((id >> 1) ^ (id >> 3) ^ (id >> 4) ^ (id >> 5)) & 1u;
    return (uint8_t)((id & 0x3Fu) | (p0 << 6) | (p1 << 7));
}

/* Sum with carry, inverted. Enhanced checksum starts with the PID, classic with 0. */
static uint8_t lin_checksum(uint32_t sum, const uint8_t *d, int n)
{
    while (n--) {
        sum += *d++;
        if (sum > 0xFFu) sum -= 0xFFu;
    }
    return (uint8_t)~sum;
}

static int errors;

/* Send a byte and check the bus read-back */
static void lin_put(uint8_t b)
{
    uart_putc(LIN, b);
    if (uart_getc(LIN) != b) errors++;
}

static void lin_header(uint8_t id)
{
    wire_putn(LIN, 14, 1u << 13);      /* BREAK: 13 bits dominant + delimiter */
    (void)LIN->RX;                     /* its read-back is a framing error */
    lin_put(0x55);                     /* sync */
    lin_put(lin_pid(id));
}

static void lin_publish(uint8_t id, const uint8_t *d, int n, int classic)
{
    lin_header(id);
    for (int i = 0; i < n; i++) lin_put(d[i]);
    lin_put(lin_checksum(classic ? 0u : lin_pid(id), d, n));
}

/* Header, then a slave's response: n bytes + checksum. Returns 1 if valid. */
static int lin_request(uint8_t id, uint8_t *d, int n)
{
    lin_header(id);
    block_timeout(PG_US_TO_TICKS(5000));
    int ok = 1;
    for (int i = 0; i < n; i++) {
        int c = uart_getc(LIN);
        if (c < 0 || block_timed_out()) ok = 0;
        d[i] = (uint8_t)c;
    }
    int c = uart_getc(LIN);
    block_timeout(0);
    delay_ticks(PG_BIT_TICKS(19200) / 2u);   /* RX returns at mid stop bit: let the slave finish it */
    return ok && c == lin_checksum(lin_pid(id), d, n);
}

int main(void)
{
    static const uint8_t pub[4]  = { 0x11, 0x22, 0x33, 0x44 };
    static const uint8_t diag[8] = { 0x7F, 0x06, 0xB2, 0x00, 0xFF, 0x7F, 0xFF, 0xFF };
    uint8_t resp[2];

    wire_config(LIN, WIRE_UART_8N1(PIN_UO(3), PIN_UI(1), 19200));
    delay_ticks(2u * PG_BIT_TICKS(19200));  /* bus recessive before the first break */
    lin_publish(0x10, pub, 4, 0);
    PG_SHARED[1] = (uint32_t)lin_request(0x21, resp, 2);
    PG_SHARED[0] = resp[0] | ((uint32_t)resp[1] << 8);
    lin_publish(0x3C, diag, 8, 1);
    wire_flush(LIN);
    PG_SHARED[2] = (uint32_t)errors;
    PG_SHARED[3] = 0x600D0011u;
    host_irq(1);
    pg_park();
}
