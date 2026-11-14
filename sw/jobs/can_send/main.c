/*
 * Copyright (c) 2026 Tyler Hong
 * SPDX-License-Identifier: Apache-2.0
 *
 * Job can_send: CAN 2.0A, send one 4-byte frame and receive it back from the
 * bus; check that another node acknowledged it and that the CRC is good.
 * Needs a full engine (slot 0 or 1).
 * Mailbox: [1] TXD pin (a uio pin), [2] RXD pin, [3] bit period in 1/256 cycles
 * (PG_BIT_FIX8, computed by the host), [4] identifier, [5] data bytes 0..3 (byte 0 in [31:24]).
 * Results: [8] 1 if acknowledged with a good CRC, [14] start, [15] end.
 */

#include "job.h"

static uint32_t words[4], nw, x, t;
static uint8_t msg[4];
static can_frame_t f;

JOB_ENTRY
{
    PT_BEGIN(pt);
    mb[14] = now();
    pin_input(mb[1]);
    can_init_fix8(JOB_W, mb[1], mb[2], mb[3]);
    for (int i = 0; i < 4; i++) msg[i] = (uint8_t)(mb[5] >> (24 - 8 * i));
    t = now() + PG_US_TO_TICKS(30);                     /* bus idle */
    PT_WAIT(pt, time_reached(t));
    can_send(JOB_W, mb[4], msg, 4);
    nw = 0;
    for (;;) {
        PT_WAIT(pt, wire_rx_count(JOB_W) > 0);
        x = wire_get(JOB_W);
        if (WIRE_IS_END(x)) break;
        if (nw < 4) words[nw] = x;
        nw++;
    }
    mb[8] = can_parse(words, nw, x, &f) == 0 && f.id == mb[4] && f.acked && f.crc_ok;
    mb[15] = now();
    PT_END(pt);
}
