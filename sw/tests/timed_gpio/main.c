/*
 * Copyright (c) 2026 Tyler Hong
 * SPDX-License-Identifier: Apache-2.0
 *
 * timed_gpio: checks the timed GPIO queue, edge capture and the blocking
 * timeout. Used by test/test_tgpio.py, which records every pin change.
 *
 * Results (PG_SHARED):
 *   [0] 0x600D0001 when done
 *   [1] t0: base time of the schedule below
 *   [2] LATE flag after the on-time schedule (expect 0)
 *   [3] LATE flag after scheduling in the past (expect 1)
 *   [4] capture time - scheduled time for a looped-back edge (expect 0)
 *   [5] block_timed_out() after waiting for an edge that never comes (expect 1)
 *   [6] value read by that timed-out access (expect 0xFFFFFFFF)
 *   [7] cycles spent in that access
 */

#include "protocol.h"

#define P_A   PIN_UO(0)
#define P_B   PIN_UO(5)
#define P_OE  PIN_UIO(7)

int main(void)
{
    pin_write(P_OE, 1);                          /* uio[7] drives 1 once enabled */

    uint32_t t0 = now() + 300;
    PG_SHARED[1] = t0;

    /* 9 changes, queue depth 4: the later calls block until slots free up. */
    pin_out_at(P_A, 1, t0 + 0);
    pin_out_at(P_A, 0, t0 + 1);
    pin_out_at(P_A, 1, t0 + 2);
    pin_out_at(P_A, 0, t0 + 10);
    pin_out_at(P_A, 1, t0 + 11);
    pin_out_at(P_A, 0, t0 + 500);
    pins_out_at((1u << P_A) | (1u << P_B), (1u << P_A) | (1u << P_B), t0 + 600);
    pins_out_at((1u << P_A) | (1u << P_B), 0, t0 + 601);
    pin_oe_at(P_OE, 1, t0 + 700);
    timed_flush();
    PG_SHARED[2] = timed_late();

    /* A change scheduled in the past happens at once and sets LATE. */
    pin_out_at(P_B, 1, now() - 10);
    timed_flush();
    PG_SHARED[3] = timed_late();
    pin_write(P_B, 0);

    /* Loopback: the capture sees our own timed edge on uo[1] at the exact cycle. */
    uint32_t t1 = now() + 100;
    capture_arm(PIN_UO(1), EDGE_RISE);
    pin_out_at(PIN_UO(1), 1, t1);
    PG_SHARED[4] = capture_wait() - t1;

    /* Blocking timeout: wait for an edge on ui[4], which the testbench never moves. */
    capture_arm(PIN_UI(4), EDGE_BOTH);
    block_timeout(50);
    uint32_t a = now();
    uint32_t v = capture_wait();
    uint32_t b = now();
    PG_SHARED[5] = block_timed_out();
    block_timeout(0);
    PG_SHARED[6] = v;
    PG_SHARED[7] = b - a;

    PG_SHARED[0] = 0x600D0001u;
    host_irq(1);
    pg_park();
}
