/*
 * Copyright (c) 2026 Tyler Hong
 * SPDX-License-Identifier: Apache-2.0
 *
 * mil1553: a MIL-STD-1553B bus controller (BC), to a 1553 transceiver with
 * TXA/TXB inputs (TXA high = bus positive, TXB high = bus negative, both
 * low = no drive) and an RXA output (high while the bus is positive).
 *
 * A 1553 word is 20 bit times at 1 Mbit/s: a 3-bit sync that is not
 * Manchester (command/status: 1.5 bits positive then 1.5 negative; data:
 * the opposite), 16 data bits MSB first and an odd parity bit, Manchester
 * II (1 = positive then negative). Everything is built in C as 2 Mbit/s
 * half-bit "chips": 40 per word, sent as two 20-chip engine words.
 *  - TX: two lite engines in lockstep (started by back-to-back register
 *    writes), TXA = chips, TXB = inverted chips, both idle low.
 *  - RX: a full engine receives RXA asynchronously in packet mode (frame
 *    end after 6 chips without a positive level); C checks the sync, every
 *    Manchester pair and the parity. RXA also shows the BC's own words,
 *    which are read and dropped first.
 * Messages: BC-to-RT (receive command + data words, RT answers with its
 * status), RT-to-BC (transmit command, RT answers status + data words),
 * and a command to an absent RT (no response within 14 us).
 *
 * Pins: TXA uo[4] (P12), TXB uo[5] (P13), RXA ui[0] (P16).
 * Results (PG_SHARED): [2] 0x600D0012 when done; per message k (3):
 * [8 + 8k] status word (or 0xFFFFFFFF: no response, 0xEEEEEEEE: bad word),
 * [9 + 8k] words received after the status, [10 + 8k ..] data words.
 */

#include "protocol.h"

#define TXA W2
#define TXB W3
#define RX  W0
#define CHIP_FIX8 PG_BIT_FIX8(2000000)

static uint32_t chips20[2];                       /* a word as two 20-chip halves, first chip = MSB */

static void encode(uint32_t data16, int cmd_sync)
{
    uint32_t hi = cmd_sync ? 0x38u : 0x07u;       /* sync: 111000 or 000111 */
    uint32_t par = 1u;                            /* odd parity over data + parity */
    uint64_t c = hi;
    for (int i = 15; i >= 0; i--) {
        uint32_t b = (data16 >> i) & 1u;
        par ^= b;
        c = (c << 2) | (b ? 2u : 1u);             /* 1 = 10, 0 = 01 */
    }
    c = (c << 2) | (par ? 2u : 1u);
    chips20[0] = (uint32_t)(c >> 20) & 0xFFFFFu;
    chips20[1] = (uint32_t)c & 0xFFFFFu;
}

/* Send n words (w[0] with command sync, the rest data sync) as one message */
static void bc_send(const uint32_t *w, uint32_t n)
{
    uint32_t base = WIRE_EN | WIRE_MSB_FIRST | WIRE_LEN(20);
    TXA->CTRL = base; TXB->CTRL = base;           /* stop: TX off, FIFOs kept */
    encode(w[0], 1);
    wire_put(TXA, chips20[0]); wire_put(TXB, ~chips20[0] & 0xFFFFFu);
    wire_put(TXA, chips20[1]); wire_put(TXB, ~chips20[1] & 0xFFFFFu);
    TXA->CTRL = base | WIRE_TX;                   /* start both, two stores apart */
    TXB->CTRL = base | WIRE_TX;
    for (uint32_t i = 1; i < n; i++) {
        encode(w[i], 0);
        wire_put(TXA, chips20[0]); wire_put(TXB, ~chips20[0] & 0xFFFFFu);
        wire_put(TXA, chips20[1]); wire_put(TXB, ~chips20[1] & 0xFFFFFu);
    }
}

/* Receive the RT's response: up to max words into out; returns the number
 * of words, -1 for no response, -2 for a bad word. */
static int bc_recv(uint32_t *out, uint32_t max, uint32_t timeout)
{
    uint32_t n = 0, half = 0, first = 0;
    int bad = 0;
    block_timeout(timeout);
    for (;;) {
        uint32_t x = wire_get(RX);
        if (block_timed_out()) { block_timeout(0); return n ? (bad ? -2 : (int)n) : -1; }
        block_timeout(60u * 50u);                 /* the rest follows at once */
        if (x == WIRE_EV_START) continue;
        if (WIRE_IS_END(x)) break;
        if (!half) { first = x; half = 1; continue; }
        half = 0;
        uint64_t c = ((uint64_t)first << 20) | (x & 0xFFFFFu);
        uint32_t sync = (uint32_t)(c >> 34) & 0x3Fu, d = 0, par = 0;
        if (sync != (n ? 0x07u : 0x38u)) bad = 1;
        for (int i = 16; i >= 0; i--) {           /* 16 data bits + parity */
            uint32_t p = (uint32_t)(c >> (2 * i)) & 3u;
            if (p != 1u && p != 2u) bad = 1;      /* not Manchester */
            uint32_t b = p == 2u;
            par ^= b;
            if (i) d = (d << 1) | b;
        }
        if (!par) bad = 1;                        /* odd parity */
        if (n < max) out[n] = d;
        n++;
    }
    block_timeout(0);
    return bad ? -2 : (int)n;
}

/* RXA also shows the BC's own words: read that echo up to its end marker */
static void bc_skip_echo(void)
{
    block_timeout(PG_US_TO_TICKS(10));
    while (!WIRE_IS_END(wire_get(RX)) && !block_timed_out()) { }
    block_timeout(0);
}

static uint32_t cmd(uint32_t rt, uint32_t tr, uint32_t sa, uint32_t wc)
{
    return (rt << 11) | (tr << 10) | (sa << 5) | (wc & 31u);
}

int main(void)
{
    uint32_t w[8], r[8];
    int n;
    wire_config(TXA, (wire_cfg_t){ WIRE_EN | WIRE_MSB_FIRST | WIRE_LEN(20), CHIP_FIX8 >> 8,
                                   WIRE_PINS(PIN_UO(4), 0, 0), WIRE_FRAC(CHIP_FIX8) });
    wire_config(TXB, (wire_cfg_t){ WIRE_EN | WIRE_MSB_FIRST | WIRE_LEN(20), CHIP_FIX8 >> 8,
                                   WIRE_PINS(PIN_UO(5), 0, 0), WIRE_FRAC(CHIP_FIX8) });
    wire_config(RX, (wire_cfg_t){ WIRE_EN | WIRE_RX | WIRE_RX_ASYNC | WIRE_MSB_FIRST | WIRE_EVENTS | WIRE_LEN(20),
                                  CHIP_FIX8 >> 8, WIRE_PINS(0, PIN_UI(0), 0),
                                  WIRE_FRAC(CHIP_FIX8) | WIRE_RESYNC | WIRE_RX_PACKET | WIRE_EOF_RUN(6) });
    host_irq(1);
    delay_ticks(PG_US_TO_TICKS(20));

    /* 1. BC to RT 5, subaddress 1, 4 data words */
    w[0] = cmd(5, 0, 1, 4);
    for (int i = 1; i <= 4; i++) w[i] = 0x1000u * (uint32_t)i + 0x0A5u;
    bc_send(w, 5);
    bc_skip_echo();
    n = bc_recv(r, 8, PG_US_TO_TICKS(14));
    PG_SHARED[8] = n > 0 ? r[0] : (n == -1 ? 0xFFFFFFFFu : 0xEEEEEEEEu);
    PG_SHARED[9] = n > 0 ? (uint32_t)n - 1u : 0u;
    delay_ticks(PG_US_TO_TICKS(10));

    /* 2. RT 5 to BC, subaddress 2, 3 data words */
    w[0] = cmd(5, 1, 2, 3);
    bc_send(w, 1);
    bc_skip_echo();
    n = bc_recv(r, 8, PG_US_TO_TICKS(14));
    PG_SHARED[16] = n > 0 ? r[0] : (n == -1 ? 0xFFFFFFFFu : 0xEEEEEEEEu);
    PG_SHARED[17] = n > 0 ? (uint32_t)n - 1u : 0u;
    for (int i = 1; i < n && i < 6; i++) PG_SHARED[17 + i] = r[i];
    delay_ticks(PG_US_TO_TICKS(10));

    /* 3. BC to an absent RT 9: no response */
    w[0] = cmd(9, 0, 1, 1);
    w[1] = 0xBEEFu;
    bc_send(w, 2);
    bc_skip_echo();
    n = bc_recv(r, 8, PG_US_TO_TICKS(14));
    PG_SHARED[24] = n > 0 ? r[0] : (n == -1 ? 0xFFFFFFFFu : 0xEEEEEEEEu);
    PG_SHARED[25] = n > 0 ? (uint32_t)n - 1u : 0u;

    PG_SHARED[2] = 0x600D0012u;
    host_irq(0);
    pg_park();
}
