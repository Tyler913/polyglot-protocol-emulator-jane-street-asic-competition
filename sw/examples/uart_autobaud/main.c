/*
 * Copyright (c) 2026 Tyler Hong
 * SPDX-License-Identifier: Apache-2.0
 *
 * uart_autobaud: a UART that finds the sender's baud rate. The host sends
 * 'U' (0x55): with the start bit its falling edges are exactly two bits
 * apart, so the capture unit's timestamps of the first and fifth falling
 * edge are 8 bits apart. That gives the bit period in 1/32 cycles, which
 * goes straight into the engine's integer + fractional divider; the code
 * then echoes every byte at the measured rate.
 *
 * Pins: TX uo[1] (P9), RX ui[0] (P16).
 * Results (PG_SHARED): [0] measured bit period in 1/256 cycles, [1] bytes echoed.
 */

#include "protocol.h"

#define UART W0

int main(void)
{
    capture_arm(PIN_UI(0), EDGE_FALL);
    uint32_t t0 = capture_wait(), t = t0;
    for (int i = 0; i < 4; i++) t = capture_wait();
    uint32_t fix8 = (t - t0) * 32u;                       /* (8 bits) * 256 / 8 */
    PG_SHARED[0] = fix8;
    delay_ticks(fix8 >> 7);                               /* let the 'U' stop bit pass */
    wire_config(UART, WIRE_UART_8N1_FIX8(PIN_UO(1), PIN_UI(0), fix8));
    uart_putc(UART, 'U');
    for (uint32_t n = 1;; n++) {
        int c = uart_getc(UART);
        if (c >= 0) uart_putc(UART, (uint8_t)c);
        PG_SHARED[1] = n;
    }
}
