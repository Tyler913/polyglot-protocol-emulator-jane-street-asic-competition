/*
 * Copyright (c) 2026 Tyler Hong
 * SPDX-License-Identifier: Apache-2.0
 *
 * jtag: an IEEE 1149.1 host on a lite Wire Engine with TMS on a GPIO pin.
 * Scans a chain of two devices: reads both IDCODEs after reset, reads the
 * captured instruction registers, counts the devices in BYPASS, then selects
 * an 8-bit user data register in the first device (the second in BYPASS),
 * writes it twice and gets the old values back.
 *
 * Pins: TCK uo[2] (P10), TDI uo[3] (P11), TMS uo[4] (P12), TDO ui[1] (P17).
 * Chain: TDI -> device 0 (IR 4 bits, USER = 0xA) -> device 1 (IR 5 bits) -> TDO.
 *
 * Host parameters (PG_SHARED): [0] TCK frequency in Hz (0 = 1 MHz).
 * Results: [1] IDCODE of device 1 (nearest TDO), [2] IDCODE of device 0,
 * [3] captured IR bits (9), [4] devices in BYPASS, [5] USER before the first
 * write, [6] USER read back by the second write, [7] 0x600D0000.
 */

#include "protocol.h"

#define JTAG W3
#define TCK  PIN_UO(2)
#define TDI  PIN_UO(3)
#define TMS  PIN_UO(4)
#define TDO  PIN_UI(1)

#define IR_LEN   9u              /* 4 + 5 */
#define BYPASS1  0x1Fu
#define USER0    0xAu

/* DR scan of n bits from Run-Test/Idle back to Run-Test/Idle. */
static uint32_t dr(uint32_t tdi, int n)
{
    jtag_tms(JTAG, TMS, JTAG_TO_SHIFT_DR);
    uint32_t r = jtag_shift(JTAG, TMS, tdi, n, 1);
    jtag_tms(JTAG, TMS, JTAG_TO_IDLE);
    return r;
}

static uint32_t ir(uint32_t tdi)
{
    jtag_tms(JTAG, TMS, JTAG_TO_SHIFT_IR);
    uint32_t r = jtag_shift(JTAG, TMS, tdi, IR_LEN, 1);
    jtag_tms(JTAG, TMS, JTAG_TO_IDLE);
    return r;
}

int main(void)
{
    uint32_t hz = PG_SHARED[0] ? PG_SHARED[0] : 1000000u;
    volatile uint32_t *res = PG_SHARED;

    pin_write(TMS, 1);
    wire_config(JTAG, WIRE_JTAG(TCK, TDI, TDO, hz));
    jtag_tms(JTAG, TMS, JTAG_RESET);

    /* After reset every device selects IDCODE: 64 bits, nearest TDO first */
    jtag_tms(JTAG, TMS, JTAG_TO_SHIFT_DR);
    res[1] = jtag_shift(JTAG, TMS, 0u, 32, 0);
    res[2] = jtag_shift(JTAG, TMS, 0u, 32, 1);
    jtag_tms(JTAG, TMS, JTAG_TO_IDLE);

    /* All ones = BYPASS everywhere; Capture-IR shows ...01 per device */
    res[3] = ir((1u << IR_LEN) - 1u);

    /* BYPASS registers capture 0: shift ones and count the zeros that come out first */
    uint32_t z = dr(0xFFFFFFFFu, 32), n = 0;
    while (n < 32 && !((z >> n) & 1u)) n++;
    res[4] = n;

    /* Device 0 USER, device 1 BYPASS: DR = device 1's bit, then device 0's 8 bits */
    ir(BYPASS1 | (USER0 << 5));
    res[5] = dr(0x5Au << 1, 9) >> 1;
    res[6] = dr(0xC3u << 1, 9) >> 1;

    res[7] = 0x600D0000u;
    host_irq(1);
    pg_park();
}
