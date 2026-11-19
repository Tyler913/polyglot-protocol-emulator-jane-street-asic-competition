/*
 * Copyright (c) 2026 Tyler Hong
 * SPDX-License-Identifier: Apache-2.0
 *
 * iso7816: a smart-card reader (ISO/IEC 7816-3, protocol T=0): SIM, bank
 * and ID cards. The card needs a free-running clock; here a lite engine
 * makes it by shifting 0xAAAAAAAA at twice the card clock (1 MHz card
 * clock: 32 bits = 16 clocks = 800 cycles), and C tops up its FIFO in every
 * wait loop (4 words = 3200 cycles of slack). The I/O line is open drain on
 * a second engine, 8 data bits, even parity, 2 stop bits (guard time), one
 * elementary time unit (etu) = 372 card clocks = 18600 cycles, so the
 * reader's UART is locked to the card clock. RST is a GPIO.
 *
 * Sequence: RST low for 500 card clocks, then high; the Answer-To-Reset
 * (TS, T0, historical bytes; direct convention only); then GET CHALLENGE
 * (00 84 00 00 08): procedure byte = INS, 8 bytes, status word 90 00.
 *
 * Pins: CLK uo[0] (P8), RST uo[1] (P9), I/O uio[2] (P2, pull-up).
 * Results (PG_SHARED): [2] 0x600D0015 when done (0xBAD00015 + step on a
 * failure), [3] ATR length, [4..7] ATR bytes, [8] status word, [9..10] the
 * 8 challenge bytes, [11] parity errors.
 */

#include "protocol.h"

#define CK  W3
#define IO  W2
#define ETU (372u * 50u)                       /* cycles: 372 card clocks of 50 cycles */
#define FMT SER_FMT(8, SER_EVEN, 2, 0)

static uint32_t perr;

static void clk_feed(void)                     /* keep the card clock running */
{
    while (WIRE_TX_LEVEL(CK->STATUS) < 3u) wire_put(CK, 0xAAAAAAAAu);
}

static void wait_ticks(uint32_t n)
{
    uint32_t t = now();
    while (now() - t < n) clk_feed();
}

/* Next character from the card (our own echo is read by io_put), or -1 on timeout */
static int io_get(uint32_t timeout)
{
    uint32_t t = now();
    while (!wire_rx_count(IO)) {
        clk_feed();
        if (now() - t > timeout) return -1;
    }
    int c = ser_decode(FMT, wire_get(IO));
    if (c < 0) perr++;
    return c & 0xFF;
}

static void io_put(uint32_t b)
{
    ser_putc(IO, FMT, b);
    (void)io_get(20u * ETU);                   /* the echo on the shared line */
}

int main(void)
{
    uint8_t atr[16];
    int n = 0, c;
    PG_SHARED[2] = 0xBAD00015u;
    pin_write(PIN_UO(1), 0);                   /* RST low */
    wire_config(CK, (wire_cfg_t){ WIRE_EN | WIRE_TX | WIRE_LEN(32), 25u, WIRE_PINS(PIN_UO(0), 0, 0), 0u });
    wire_config(IO, (wire_cfg_t){ WIRE_EN | WIRE_TX | WIRE_RX | WIRE_RX_ASYNC | WIRE_IDLE_HIGH |
                                  WIRE_OE_OPEN_DRAIN | WIRE_LEN(ser_bits(FMT)),
                                  ETU, WIRE_PINS(PIN_UIO(2), PIN_UIO(2), 0), WIRE_RESYNC });
    host_irq(1);                               /* ready */
    clk_feed();
    wait_ticks(500u * 50u);                    /* RST low for 500 clocks */
    pin_write(PIN_UO(1), 1);

    while ((c = io_get(n ? 20u * ETU : 50000u * 50u)) >= 0 && n < (int)sizeof atr) atr[n++] = (uint8_t)c;
    PG_SHARED[3] = (uint32_t)n;
    for (int i = 0; i < 16; i++) {
        if ((i & 3) == 0) PG_SHARED[4 + (i >> 2)] = 0;
        if (i < n) PG_SHARED[4 + (i >> 2)] |= (uint32_t)atr[i] << (8 * (i & 3));
    }
    if (n < 2 || atr[0] != 0x3Bu) goto done;   /* TS: direct convention */
    PG_SHARED[2]++;

    static const uint8_t cmd[5] = { 0x00, 0x84, 0x00, 0x00, 0x08 };   /* GET CHALLENGE */
    wait_ticks(16u * ETU);                     /* guard time before we talk */
    for (int i = 0; i < 5; i++) io_put(cmd[i]);
    do c = io_get(9600u * ETU); while (c == 0x60);   /* NULL procedure bytes: wait more */
    if (c != 0x84) goto done;                  /* procedure byte = INS: all data follow */
    PG_SHARED[2]++;
    uint32_t ch[2] = {0, 0};
    for (int i = 0; i < 8; i++) {
        if ((c = io_get(20u * ETU)) < 0) goto done;
        ch[i >> 2] |= (uint32_t)c << (8 * (i & 3));
    }
    int sw1 = io_get(20u * ETU), sw2 = io_get(20u * ETU);
    PG_SHARED[8] = (uint32_t)((sw1 << 8) | (sw2 & 0xFF));
    PG_SHARED[9] = ch[0];
    PG_SHARED[10] = ch[1];
    PG_SHARED[11] = perr;
    if (sw1 == 0x90 && sw2 == 0x00) PG_SHARED[2] = 0x600D0015u;
done:
    for (int i = 0; i < 64; i++) clk_feed();   /* (a real reader keeps the clock or deactivates) */
    host_irq(0);
    pg_park();
}
