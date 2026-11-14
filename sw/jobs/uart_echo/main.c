/*
 * Copyright (c) 2026 Tyler Hong
 * SPDX-License-Identifier: Apache-2.0
 *
 * Job uart_echo: UART 8N1, echo every byte + 1.
 * Mailbox: [1] TX pin, [2] RX pin, [3] bit period in 1/256 cycles (PG_BIT_FIX8,
 * computed by the host), [4] bytes to echo.
 * Results: [8] bytes echoed, [9] framing errors, [14] start, [15] end (NOW).
 * At the end the TX pin is left high through GPIO, so the line stays idle
 * when the next job reconfigures the engine.
 */

#include "job.h"

JOB_ENTRY
{
    PT_BEGIN(pt);
    mb[14] = now();
    wire_config(JOB_W, WIRE_UART_8N1_FIX8(mb[1], mb[2], mb[3]));
    while (mb[8] < mb[4]) {
        PT_WAIT(pt, wire_rx_count(JOB_W) > 0);
        int c = uart_getc(JOB_W);
        if (c < 0) { mb[9]++; continue; }
        uart_putc(JOB_W, (uint8_t)(c + 1));
        mb[8]++;
    }
    PT_WAIT(pt, tx_idle(JOB_W));
    pin_write(mb[1], 1);                /* the line stays idle (high) when the engine lets go */
    mb[15] = now();
    PT_END(pt);
}
