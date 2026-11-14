/*
 * Copyright (c) 2026 Tyler Hong
 * SPDX-License-Identifier: Apache-2.0
 *
 * ps2_host: reads a PS/2 keyboard and sends it commands, on a lite Wire
 * Engine clocked by the keyboard (external clock).
 *
 * Keyboard to host: the engine samples DATA on each falling CLK edge, 11-bit
 * words (start, 8 data bits LSB first, odd parity, stop).
 * Host to keyboard: C holds CLK low for 120 us (GPIO), then pushes the
 * 11-bit frame on an open-drain TX: its start bit pulls DATA low at once
 * (request to send). C releases CLK; the keyboard clocks the bits in on its
 * rising edges, and the engine moves to the next bit on each falling edge.
 * The keyboard acknowledges with DATA low on its 11th clock.
 *
 * Pins: CLK uio[2] (P2), DATA uio[3] (P3), both open drain with pull-ups.
 * Host parameters (PG_SHARED): [0] bytes to read first (1..8, 0 = 4).
 * Then sends 0xED 0x02 (set LEDs: Num Lock) and reads the two answers.
 * Results: [1] 0x600D0006 when done, [2] errors (framing/parity/no ACK),
 * [8..] the bytes read, then the two answers.
 */

#include "protocol.h"

#define PS2  W2
#define CLK  PIN_UIO(2)
#define DAT  PIN_UIO(3)

static uint32_t errors;

static void ps2_rx_mode(void)
{
    /* External clock, sample on the falling edge (CPOL 0, CPHA 1), RX only */
    wire_config(PS2, (wire_cfg_t){ WIRE_EN | WIRE_RX | WIRE_CLK_EXT | WIRE_CPHA | WIRE_OE_NEVER |
                                   WIRE_LEN(11), 2u, WIRE_PINS(DAT, DAT, CLK), 0u });
}

static int ps2_get(void)
{
    uint32_t f = wire_get(PS2), d = (f >> 1) & 0xFFu;
    if ((f & 1u) || !((f >> 10) & 1u) || ((f >> 9) & 1u) == pg_parity32(d)) {
        errors++;
        return -1;
    }
    return (int)d;
}

static int wait_pin(int pin, int v, uint32_t us)
{
    uint32_t t = now() + PG_US_TO_TICKS(us);
    while (pin_read(pin) != v)
        if (time_reached(t)) return -1;
    return 0;
}

static void ps2_put(uint8_t b)
{
    /* The keyboard holds CLK low for half a clock after the last bit we read:
     * inhibit only once it has released CLK, or the inhibit looks shorter to it */
    if (wait_pin(CLK, 1, 1000)) errors++;
    pin_write(CLK, 0);
    pin_output(CLK);                                  /* inhibit: CLK low */
    delay_ticks(PG_US_TO_TICKS(120));
    /* External clock, the keyboard samples on the rising edge (CPOL 0, CPHA 0), open drain */
    /* IDLE_HIGH: between words the open-drain line is released, not pulled low */
    wire_config(PS2, (wire_cfg_t){ WIRE_EN | WIRE_TX | WIRE_CLK_EXT | WIRE_OE_OPEN_DRAIN | WIRE_IDLE_HIGH |
                                   WIRE_LEN(11), 2u, WIRE_PINS(DAT, DAT, CLK), 0u });
    wire_put(PS2, ((uint32_t)b << 1) | ((pg_parity32(b) ^ 1u) << 9) | (1u << 10));
    pin_input(CLK);                                   /* release: the keyboard starts clocking */
    /* Wait until the stop bit is out (our own start bit holds DATA low, so the
     * ACK can only be looked for after the frame), then for the ACK */
    uint32_t t = now() + PG_US_TO_TICKS(3000);
    while (PS2->STATUS & WIRE_TX_ACTIVE)
        if (time_reached(t)) { errors++; break; }
    if (wait_pin(DAT, 0, 200) || wait_pin(DAT, 1, 200)) errors++;     /* ACK, then released */
    ps2_rx_mode();
}

int main(void)
{
    uint32_t n = PG_SHARED[0] ? PG_SHARED[0] : 4u, k = 0;
    if (n > 8u) n = 8u;
    volatile uint32_t *res = &PG_SHARED[8];

    ps2_rx_mode();
    host_irq(1);                                      /* ready */
    while (k < n) {
        int b = ps2_get();
        if (b >= 0) res[k++] = (uint32_t)b;
    }
    ps2_put(0xED);
    res[k++] = (uint32_t)ps2_get();
    ps2_put(0x02);
    res[k++] = (uint32_t)ps2_get();

    PG_SHARED[2] = errors;
    PG_SHARED[1] = 0x600D0006u;
    host_irq(0);
    pg_park();
}
