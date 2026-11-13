/*
 * Copyright (c) 2026 Tyler Hong
 * SPDX-License-Identifier: Apache-2.0
 *
 * quad_uart: all four Wire Engines run as UARTs at the same time.
 * Engine n: TX on uo[n+1] (P9..P12), RX on ui[n] (P16..P19).
 * Each engine sends "W<n>\r\n", then the CPU serves the engines round robin:
 * one received byte from engine n is sent back on engine n plus one.
 *
 * Host parameters (PG_SHARED, written before the CPU starts):
 *   [0] baud rate (0 = 1000000)
 * Results: [1] bytes echoed, [2] framing errors.
 */

#include "protocol.h"

static pg_wire_t *const uart[4] = { W0, W1, W2, W3 };

int main(void)
{
    uint32_t baud = PG_SHARED[0] ? PG_SHARED[0] : 1000000u;
    PG_SHARED[1] = 0;
    PG_SHARED[2] = 0;

    for (int n = 0; n < 4; n++)
        wire_config(uart[n], WIRE_UART_8N1(PIN_UO(n + 1), PIN_UI(n), baud));
    for (int n = 0; n < 4; n++) {      /* 4 bytes each: fits the TX FIFOs */
        uart_putc(uart[n], 'W');
        uart_putc(uart[n], (uint8_t)('0' + n));
        uart_putc(uart[n], '\r');
        uart_putc(uart[n], '\n');
    }
    host_irq(1);

    for (;;) {
        for (int n = 0; n < 4; n++) {
            int c = uart_getc(uart[n]);
            if (c < 0) {
                PG_SHARED[2]++;
                continue;
            }
            uart_putc(uart[n], (uint8_t)(c + 1));
            PG_SHARED[1]++;
        }
    }
}
