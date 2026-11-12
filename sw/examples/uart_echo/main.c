/*
 * Copyright (c) 2026 Tyler Hong
 * SPDX-License-Identifier: Apache-2.0
 *
 * uart_echo: a UART built from Wire Engine 0 with three lines of setup.
 * Sends a banner, then echoes every received byte, with lower-case letters
 * turned into upper case. TX on uo[1] (pin P9), RX on ui[0] (pin P16).
 *
 * Host parameters (PG_SHARED, written before the CPU starts):
 *   [0] baud rate (0 = 115200)
 * Results: [1] bytes echoed, [2] framing errors.
 */

#include "protocol.h"

#define UART      W0
#define TX_PIN    PIN_UO(1)
#define RX_PIN    PIN_UI(0)

int main(void)
{
    uint32_t baud = PG_SHARED[0] ? PG_SHARED[0] : 115200u;
    PG_SHARED[1] = 0;
    PG_SHARED[2] = 0;

    wire_config(UART, WIRE_UART_8N1(TX_PIN, RX_PIN, baud));
    uart_puts(UART, "Polyglot\r\n");
    host_irq(1);                       /* tell the host we are ready */

    for (;;) {
        int c = uart_getc(UART);
        if (c < 0) {
            PG_SHARED[2]++;
            continue;
        }
        if (c >= 'a' && c <= 'z') c -= 'a' - 'A';
        uart_putc(UART, (uint8_t)c);
        PG_SHARED[1]++;
    }
}
