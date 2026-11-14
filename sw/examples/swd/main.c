/*
 * Copyright (c) 2026 Tyler Hong
 * SPDX-License-Identifier: Apache-2.0
 *
 * swd: an ARM Serial Wire Debug host on a lite Wire Engine. It switches the
 * target's debug port from JTAG to SWD, reads the DP and AP identification
 * registers, powers up the debug domain, then writes four words into the
 * target's memory through the MEM-AP and reads them back, and reads one
 * system register (CPUID). WAIT answers are retried; the AP access before
 * power-up is expected to FAULT and is cleared through ABORT.
 *
 * Pins: SWCLK uio[0] (P0), SWDIO uio[1] (P1, pull-up on the target side).
 *
 * Host parameters (PG_SHARED): [0] SWCLK frequency in Hz (0 = 1 MHz).
 * Results: [1] DPIDR, [2] AP IDR, [3] CTRL/STAT after power-up, [4..7] the
 * four words read back, [8] CPUID, [9] WAIT retries, [10] ACK of the AP access
 * before power-up, [11] 0x600D0000 | errors.
 */

#include "protocol.h"

#define SWD     W2
#define SWCLK   PIN_UIO(0)
#define SWDIO   PIN_UIO(1)

#define DP_IDR_ABORT 0x0u
#define DP_CTRL_STAT 0x4u
#define DP_SELECT    0x8u
#define DP_RDBUFF    0xCu
#define AP_CSW       0x0u
#define AP_TAR       0x4u
#define AP_DRW       0xCu

static uint32_t retries, errors;

/* One transfer with WAIT retries; counts an error unless the ACK is OK. */
static int xfer(int ap, int rnw, uint32_t a, uint32_t *d)
{
    int ack;
    uint32_t req = SWD_REQ(ap, rnw, a);
    for (int n = 0; (ack = swd_xfer(SWD, req, d)) == SWD_WAIT && n < 100; n++) retries++;
    if (ack != SWD_OK) errors++;
    return ack;
}

static uint32_t rd(int ap, uint32_t a) { uint32_t d = 0; xfer(ap, 1, a, &d); return d; }
static void     wr(int ap, uint32_t a, uint32_t d) { xfer(ap, 0, a, &d); }

int main(void)
{
    uint32_t hz = PG_SHARED[0] ? PG_SHARED[0] : 1000000u;
    volatile uint32_t *res = PG_SHARED;
    uint32_t d;

    wire_config(SWD, WIRE_SWD(SWCLK, SWDIO, hz));
    swd_connect(SWD);
    res[1] = rd(0, DP_IDR_ABORT);

    /* An AP access before debug power-up: FAULT, sticky error; clear it */
    res[10] = (uint32_t)swd_xfer(SWD, SWD_REQ(1, 1, AP_CSW), &d);
    wr(0, DP_IDR_ABORT, 0x1Eu);

    /* Debug and system power-up requests; wait for both acknowledges */
    wr(0, DP_CTRL_STAT, 0x50000000u);
    for (int n = 0; n < 100 && ((d = rd(0, DP_CTRL_STAT)) & 0xA0000000u) != 0xA0000000u; n++) { }
    res[3] = d;

    /* AP identification: bank 0xF, register 0xC; AP reads are posted */
    wr(0, DP_SELECT, 0xF0u);
    (void)rd(1, 0xCu);
    res[2] = rd(0, DP_RDBUFF);
    wr(0, DP_SELECT, 0u);

    /* Word accesses, TAR auto-increment */
    wr(1, AP_CSW, 0x23000012u);
    wr(1, AP_TAR, 0x20000000u);
    for (uint32_t i = 0; i < 4; i++) wr(1, AP_DRW, 0xC0DE0000u + 0x1111u * i);
    wr(1, AP_TAR, 0x20000000u);
    (void)rd(1, AP_DRW);                     /* returns the previous AP read */
    for (int i = 0; i < 3; i++) res[4 + i] = rd(1, AP_DRW);
    res[7] = rd(0, DP_RDBUFF);

    wr(1, AP_TAR, 0xE000ED00u);
    (void)rd(1, AP_DRW);
    res[8] = rd(0, DP_RDBUFF);

    res[9]  = retries;
    res[11] = 0x600D0000u | errors;
    host_irq(1);
    pg_park();
}
