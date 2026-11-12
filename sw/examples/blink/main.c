/*
 * Copyright (c) 2026 Tyler Hong
 * SPDX-License-Identifier: Apache-2.0
 *
 * blink: toggle uo[0] every 100 cycles and count the toggles in the mailbox.
 * Used by the bring-up test (test/test_boot.py).
 */

#include "protocol.h"

#define HALF_PERIOD 100u

int main(void)
{
    uint32_t t = now();
    uint32_t count = 0;

    PG_HOST_MBOX = 0x600DB007u;    /* "good boot" marker */
    for (;;) {
        t += HALF_PERIOD;
        wait_until(t);
        pin_toggle(PIN_UO(0));
        PG_HOST_MBOX = ++count;
    }
}
