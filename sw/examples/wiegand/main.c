/*
 * Copyright (c) 2026 Tyler Hong
 * SPDX-License-Identifier: Apache-2.0
 *
 * wiegand: both sides of the Wiegand access-control interface. Receives a
 * 26-bit card read from a reader (DATA0/DATA1 idle high; a short low pulse
 * on DATA0 is a 0, on DATA1 a 1), checks the two parity bits (even over
 * bits 1-12, odd over bits 13-24 with the parity bit), then sends the same
 * card on to a controller as a reader would: 50 us pulses every 0.5 ms, from
 * the timed GPIO queue so every pulse edge lands on its cycle.
 *
 * Pins: in DATA0 ui[0] (P16), DATA1 ui[1] (P17); out DATA0 uo[0] (P8),
 * DATA1 uo[1] (P9).
 * Results (PG_SHARED): [0] facility code, [1] card number, [2] parity ok,
 * [3] 0x600D3126 after the card was sent on.
 */

#include "protocol.h"

#define D0_IN  PIN_UI(0)
#define D1_IN  PIN_UI(1)
#define D0_OUT PIN_UO(0)
#define D1_OUT PIN_UO(1)

/* Receive bits until the lines stay idle for 3 ms (readers send a bit every 0.2..2 ms). Returns the bit count. */
static int wiegand_read(uint32_t *v)
{
    int n = 0;
    uint32_t idle_since = 0;
    *v = 0;
    for (;;) {
        uint32_t in = PG_GPIO_IN, t = now();
        int d0 = !((in >> D0_IN) & 1u), d1 = !((in >> D1_IN) & 1u);
        if (d0 || d1) {
            *v = (*v << 1) | (uint32_t)d1;
            n++;
            while ((~PG_GPIO_IN >> D0_IN) & 3u) { }   /* wait for the pulse to end */
            idle_since = now();
        } else if (n && t - idle_since > PG_US_TO_TICKS(3000)) {
            return n;
        }
    }
}

static int parity(uint32_t x) { x ^= x >> 16; x ^= x >> 8; x ^= x >> 4; x ^= x >> 2; x ^= x >> 1; return (int)(x & 1u); }

static void wiegand_send(uint32_t v, int n)
{
    uint32_t t = now() + PG_US_TO_TICKS(100);
    for (int i = n - 1; i >= 0; i--) {
        int pin = ((v >> i) & 1u) ? D1_OUT : D0_OUT;
        pin_out_at(pin, 0, t);
        pin_out_at(pin, 1, t + PG_US_TO_TICKS(50));
        t += PG_US_TO_TICKS(500);
    }
    timed_flush();
}

int main(void)
{
    pin_write(D0_OUT, 1);
    pin_write(D1_OUT, 1);
    uint32_t v;
    int n = wiegand_read(&v);
    int ok = (n == 26) && parity(v >> 13) == 0 && parity(v & 0x1FFFu) == 1;
    PG_SHARED[0] = (v >> 17) & 0xFFu;
    PG_SHARED[1] = (v >> 1) & 0xFFFFu;
    PG_SHARED[2] = (uint32_t)ok;
    wiegand_send(v, n);
    PG_SHARED[3] = 0x600D3126u;
    host_irq(1);
    pg_park();
}
