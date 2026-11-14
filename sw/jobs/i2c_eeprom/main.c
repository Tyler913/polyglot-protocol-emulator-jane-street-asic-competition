/*
 * Copyright (c) 2026 Tyler Hong
 * SPDX-License-Identifier: Apache-2.0
 *
 * Job i2c_eeprom: I2C controller against a 24C02-style EEPROM at 0x50: page
 * write of 8 bytes, ACK polling during the write cycle, random read back.
 *
 * Written to fit an 832-byte job slot. The transaction is an op list run by
 * one loop; ACK polling is part of the list (a NACK on the polled address
 * jumps back to the STOP before it: STOP, START, address, again). START and
 * STOP are short tables of pin steps run by one loop (same sequence as
 * i2c_start/i2c_stop in protocol.h). The host passes the SCL period in
 * cycles, so no division is linked in.
 *
 * Mailbox: [1] SDA pin, [2] SCL pin (uio pins, open drain), [3] SCL period in
 * cycles (a multiple of 4), [4] word address.
 * Results: [8] 0x600D0000 | errors, [9] polls, [10..11] bytes read back, [14] start, [15] end.
 */

#include "job.h"

enum { S = 0x100, P = 0x200, RETRY = 0x300, R = 0x400, RL = 0x500, A = 0x600, END = 0x700 };

static const uint16_t prog[] = {
    S, 0xA0, A, 0xA0, 0xA7, 0xAE, 0xB5, 0xBC, 0xC3, 0xCA, 0xD1,   /* page write (A = word address) */
    P, S, RETRY,                                                /* ACK polling: 0xA0 until ACK */
    A, S, 0xA1, R, R, R, R, R, R, R, RL, P,                     /* random read */
    END,
};
#define DATA 3                                                  /* prog index of the first data byte */

/* Pin steps: [0] SDA low, [1] SCL low, [2] engine on after, [3] engine off first,
 * [4] wait for SCL high, [7:5] quarter bits to wait (1 or 2). */
#define ST(sda, scl, q, flags) ((sda) | (scl) << 1 | (flags) | (q) << 5)
#define ON 4u
#define OFF 8u
#define WSCL 16u
static const uint8_t st_rep[] = {ST(0, 1, 1, OFF), ST(0, 0, 2, WSCL)};   /* release before rep. START */
static const uint8_t st_start[] = {ST(1, 0, 2, 0), ST(1, 1, 1, ON)};
static const uint8_t st_stop[] = {ST(1, 1, 1, OFF), ST(1, 0, 2, WSCL), ST(0, 0, 2, 0)};

static i2c_t bus;
static const uint8_t *seq;
static uint32_t pc, op, t, polls, bad, nback, n, k, s;
static uint32_t back[2];
static uint8_t acked;
static pt_t sub;

/* Run n pin steps from seq (START, repeated START or STOP). */
static int steps(pt_t *p)
{
    PT_BEGIN(p);
    for (k = 0; k < n; k++) {
        s = seq[k];
        if (s & OFF) { PT_WAIT(p, tx_idle(bus.w)); i2c__engine(&bus, 0); }
        i2c__pins(s & 1u, (s >> 1) & 1u, &bus);
        if (s & WSCL) PT_WAIT(p, pin_read(bus.scl));
        t = now() + (bus.quarter << ((s >> 5) - 1u));        /* 1 or 2 quarters: no multiply */
        PT_WAIT(p, time_reached(t));
        if (s & ON) i2c__engine(&bus, 1);
    }
    PT_END(p);
}

/* One byte plus the ACK slot; a read keeps the byte. */
static int byte(pt_t *p, uint32_t tx, uint32_t ack_bit)
{
    PT_BEGIN(p);
    wire_putn(bus.w, 8, tx);
    wire_putn(bus.w, 1, ack_bit);
    PT_WAIT(p, wire_rx_count(bus.w) >= 2);
    ((uint8_t *)back)[nback & 7] = (uint8_t)wire_get(bus.w);
    acked = (wire_get(bus.w) & 1u) == 0u;
    PT_END(p);
}

JOB_ENTRY
{
    PT_BEGIN(pt);
    mb[14] = now();
    bus.w = JOB_W; bus.sda = mb[1]; bus.scl = mb[2];
    bus.quarter = mb[3] / 4u;                                   /* constant divisor: a shift */
    i2c__pins(0, 0, &bus);
    JOB_W->CTRL = 0;
    JOB_W->DIV  = mb[3];
    JOB_W->PINS = WIRE_PINS(bus.sda, bus.sda, bus.scl);
    i2c__engine(&bus, 0);
    for (pc = 0; (op = prog[pc]) != END; pc++) {
        if (op == S || op == P) {
            if (op == S && (bus.w->CTRL & WIRE_TX)) {           /* repeated START: release first */
                seq = st_rep; n = 2;
                PT_SPAWN(pt, &sub, steps(&sub));
            }
            if (op == P) { seq = st_stop; n = 3; }
            else         { seq = st_start; n = 2; }
            PT_SPAWN(pt, &sub, steps(&sub));
            continue;
        }
        /* ACK slot: release SDA (1) after a write and after the last read; 0 = ACK a read */
        PT_SPAWN(pt, &sub, byte(&sub, op == A ? mb[4] : op == RETRY ? 0xA0 : op >= R ? 0xFF : op, op != R));
        if (op == RETRY) {
            polls++;
            if (!acked) pc -= 3;                                /* back to the STOP before it */
        } else if (op == R || op == RL) {
            nback++;
        } else {
            bad += !acked;
        }
    }
    for (k = 0; k < 8; k++) bad += ((uint8_t *)back)[k] != prog[DATA + k];
    mb[9] = polls;
    mb[10] = back[0];
    mb[11] = back[1];
    mb[8] = 0x600D0000u | bad;
    mb[15] = now();
    PT_END(pt);
}
