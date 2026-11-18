/*
 * Copyright (c) 2026 Tyler Hong
 * SPDX-License-Identifier: Apache-2.0
 *
 * ps2_device: the keyboard side of PS/2 (examples/ps2_host is the other
 * side). The device always makes the clock (here 12.5 kHz).
 * Device to host: a lite engine with an open-drain bit clock (CPOL = 1:
 * CLK high, then low in the middle of each bit, where the host reads) and
 * an open-drain DATA that changes a quarter bit after the rising edge
 * (TX_LATE). One 11-bit word per byte: start, 8 data bits, odd parity, stop.
 * Host to device: the host inhibits (CLK low >= 100 us), pulls DATA low and
 * releases CLK. C then clocks the 10 bits in with the timed GPIO queue,
 * reading DATA at each rising edge, and pulls DATA low during an 11th clock
 * as the acknowledge. Each command is answered with 0xFA.
 *
 * Pins: CLK uio[2] (P2), DATA uio[3] (P3), open drain with pull-ups.
 * Types "hi" (make and break codes), then answers the host's commands; after
 * Set LEDs (0xED) and its argument it stops.
 * Results (PG_SHARED): [0] LED byte, [1] commands received, [2] 0x600D0B0D.
 */

#include "protocol.h"

#define KBD  W2
#define CLK  PIN_UIO(2)
#define DAT  PIN_UIO(3)
#define HALF PG_US_TO_TICKS(40)

static uint32_t frame(uint8_t b)
{
    uint32_t p = 1;
    for (int i = 0; i < 8; i++) p ^= (b >> i) & 1u;      /* odd parity */
    return ((uint32_t)b << 1) | (p << 9) | (1u << 10);  /* start 0 in bit 0 */
}

static int bus_idle(void) { return pin_read(CLK) && pin_read(DAT); }

static void ps2_send(uint8_t b)
{
    uint32_t t = now();
    while (now() - t < PG_US_TO_TICKS(60)) if (!bus_idle()) t = now();   /* 60 us of idle bus */
    KBD->CTRL = WIRE_EN | WIRE_TX | WIRE_IDLE_HIGH | WIRE_OE_OPEN_DRAIN | WIRE_CLK_OUT | WIRE_CLK_OD |
                WIRE_CPOL | WIRE_TX_LATE | WIRE_LEN(11);
    wire_put(KBD, frame(b));
    wire_flush(KBD);
    KBD->CTRL = 0;                                       /* pins back to GPIO (released) */
}

/* After a request to send: clock 10 bits in, acknowledge on the 11th clock. Returns the byte or -1. */
static int ps2_receive(void)
{
    uint32_t v = 0, t = now() + HALF;
    for (int k = 0; k < 10; k++) {
        pin_oe_at(CLK, 1, t);                            /* falling: the host sets the next bit */
        pin_oe_at(CLK, 0, t + HALF);                     /* rising: we read it */
        wait_until(t + HALF + 4u);
        v |= (uint32_t)pin_read(DAT) << k;
        t += 2u * HALF;
    }
    pin_oe_at(DAT, 1, t - HALF + 8u);                    /* ACK: DATA low from the 10th rising edge */
    pin_oe_at(CLK, 1, t);                                /* 11th clock */
    pin_oe_at(CLK, 0, t + HALF);
    pin_oe_at(DAT, 0, t + HALF + 8u);
    timed_flush();
    uint32_t b = v & 0xFFu, p = (v >> 8) & 1u;
    for (int i = 0; i < 8; i++) p ^= (b >> i) & 1u;
    return (p == 1u && (v >> 9)) ? (int)b : -1;          /* odd parity, stop 1 */
}

int main(void)
{
    static const uint8_t keys[6] = { 0x33, 0xF0, 0x33, 0x43, 0xF0, 0x43 };   /* h, i: make, break */
    KBD->CTRL = 0;
    KBD->DIV  = 2u * HALF;
    KBD->PINS = WIRE_PINS(DAT, DAT, CLK);
    pin_write(CLK, 0);                                   /* open drain: value 0, OE switches */
    pin_write(DAT, 0);
    pin_input(CLK);
    pin_input(DAT);
    delay_ticks(PG_US_TO_TICKS(200));
    for (int i = 0; i < 6; i++) ps2_send(keys[i]);

    uint32_t ncmd = 0;
    int want_arg = 0;
    for (;;) {
        while (pin_read(CLK)) { }                        /* host inhibits */
        while (!pin_read(CLK)) { }                       /* ... and releases CLK */
        delay_ticks(PG_US_TO_TICKS(20));
        if (pin_read(DAT)) continue;                     /* no request to send */
        int c = ps2_receive();
        if (c < 0) { ps2_send(0xFE); continue; }         /* resend */
        PG_SHARED[1] = ++ncmd;
        ps2_send(0xFA);
        if (want_arg) { PG_SHARED[0] = (uint32_t)c; break; }
        want_arg = (c == 0xED);
    }
    PG_SHARED[2] = 0x600D0B0Du;
    host_irq(1);
    pg_park();
}
