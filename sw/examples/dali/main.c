/*
 * Copyright (c) 2026 Tyler Hong
 * SPDX-License-Identifier: Apache-2.0
 *
 * dali: a DALI lighting application controller (IEC 62386-101/102) on a full
 * Wire Engine. DALI is 1200 bit/s Manchester (bi-phase) on a bus that idles
 * high; a 1 is a rising edge in the middle of the bit. The engine encodes
 * the 16-bit forward frame (start bit + address byte + command byte, MSB
 * first) and decodes the 8-bit backward frame from a control gear with its
 * Manchester receiver (clock recovery from the edges; MSTART because the
 * first edge from the idle level is the start of the start bit).
 *
 * Pins: TX uo[4] (P12) and RX ui[2] (P18), to a DALI bus interface.
 * Sends DAPC 0xC8 to short address 3, then QUERY ACTUAL LEVEL to it, then a
 * query to short address 9 (no gear: no answer).
 * Results (PG_SHARED): [0] answer to the first query (or 0x100 = none),
 * [1] answer to the second, [2] 0x600DDA11.
 */

#include "protocol.h"

#define DALI W1
#define TE   PG_US_TO_TICKS(417)    /* half a bit */

static void dali_on(uint32_t rx)
{
    DALI->CTRL = WIRE_EN | WIRE_TX | WIRE_MSB_FIRST | WIRE_IDLE_HIGH | WIRE_RX_ASYNC | WIRE_LEN(8) |
                 (rx ? WIRE_RX : 0u);
}

static void dali_send(uint8_t addr, uint8_t data)
{
    dali_on(0);                                        /* the RX would hear the forward frame */
    wire_putn(DALI, 17, (1u << 16) | ((uint32_t)addr << 8) | data);
    wire_flush(DALI);
}

/* Forward frame, then the backward frame: the gear answers 5.5..10.5 ms
 * after the end of a query (settling time + its frame). Returns 0..255 or 0x100. */
static uint32_t dali_query(uint8_t addr, uint8_t cmd)
{
    dali_send(addr, cmd);
    delay_ticks(6u * TE);       /* the decoder has heard our own frame: let it end (2 bits */
    dali_on(1);                 /* after its last edge) before RX is on; a gear answers 7 Te at the earliest */
    block_timeout(PG_US_TO_TICKS(20000));            /* 22 Te settling + 10 bits */
    uint32_t a = DALI->RX;
    if (block_timed_out()) a = 0x100u;
    block_timeout(0);
    delay_ticks(4u * TE);                             /* stop condition before the next frame */
    return a & 0x1FFu;
}

int main(void)
{
    DALI->CTRL  = 0;
    DALI->DIV   = PG_BIT_TICKS(1200);
    DALI->CTRL2 = WIRE_MANCHESTER | WIRE_MSTART;
    DALI->PINS  = WIRE_PINS(PIN_UO(4), PIN_UI(2), 0);
    dali_on(0);
    delay_ticks(8u * TE);                             /* bus idle before the first frame */

    dali_send(3u << 1, 0xC8);                         /* DAPC: short address 3, level 200 */
    delay_ticks(22u * TE);                            /* settling time between forward frames */
    PG_SHARED[0] = dali_query((3u << 1) | 1u, 0xA0);  /* QUERY ACTUAL LEVEL */
    PG_SHARED[1] = dali_query((9u << 1) | 1u, 0xA0);
    PG_SHARED[2] = 0x600DDA11u;
    host_irq(1);
    pg_park();
}
