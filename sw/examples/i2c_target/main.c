/*
 * Copyright (c) 2026 Tyler Hong
 * SPDX-License-Identifier: Apache-2.0
 *
 * i2c_target: the chip as an I2C target (address 0x42) with 16 byte
 * registers, on Wire Engine 1. A write sets the register pointer (first
 * byte) and then writes registers; a read returns registers from the
 * pointer on. The engine stretches SCL whenever C has not decided the next
 * bit yet, so this plain C state machine works at any optimisation level.
 *
 * Pins: SDA uio[2] (P2), SCL uio[3] (P3), open drain with pull-ups.
 * Results (PG_SHARED): [1] completed transactions (STOP seen),
 * [2] transactions for other addresses, [8..11] the registers.
 */

#include "protocol.h"

#define T      W1
#define MYADDR 0x42

enum { IDLE, ADDR, ACKED_W, GOT_REG, ACKED_DATA, GOT_DATA,
       ACKED_R, SENT, MASTER_ACK, OTHER_ACK, OTHER_BYTE };

static uint8_t regs[16];

static void publish(void)
{
    for (int i = 0; i < 4; i++)
        PG_SHARED[8 + i] = regs[4 * i] | (regs[4 * i + 1] << 8) |
                           ((uint32_t)regs[4 * i + 2] << 16) | ((uint32_t)regs[4 * i + 3] << 24);
}

int main(void)
{
    int st = IDLE;
    uint8_t ptr = 0;

    for (int i = 0; i < 16; i++) regs[i] = (uint8_t)(0x30 + i);
    publish();
    PG_SHARED[1] = 0;
    PG_SHARED[2] = 0;
    wire_config(T, WIRE_I2C_TARGET(PIN_UIO(2), PIN_UIO(3)));
    host_irq(1);

    for (;;) {
        uint32_t w = wire_get(T);       /* every word, including our own ACK slots */
        if (w == WIRE_EV_START) { st = ADDR; continue; }
        if (w == WIRE_EV_END)   { if (st != IDLE) PG_SHARED[1]++; st = IDLE; publish(); continue; }

        switch (st) {
        case ADDR:                      /* address byte */
            if ((w >> 1) == MYADDR) {
                wire_putn(T, 1, 0);     /* ACK */
                st = (w & 1) ? ACKED_R : ACKED_W;
            } else {
                wire_putn(T, 1, 1);     /* not us: release */
                PG_SHARED[2]++;
                st = OTHER_ACK;
            }
            break;
        case ACKED_W:                   /* our ACK went out; receive the register number */
            wire_putn(T, 8, 0xFF);
            st = GOT_REG;
            break;
        case GOT_REG:
            ptr = (uint8_t)(w & 15);
            wire_putn(T, 1, 0);
            st = ACKED_DATA;
            break;
        case ACKED_DATA:                /* receive a data byte (or a STOP / repeated START) */
            wire_putn(T, 8, 0xFF);
            st = GOT_DATA;
            break;
        case GOT_DATA:
            regs[ptr] = (uint8_t)w;
            ptr = (ptr + 1) & 15;
            wire_putn(T, 1, 0);
            st = ACKED_DATA;
            break;
        case ACKED_R:                   /* send a register */
            wire_putn(T, 8, regs[ptr]);
            ptr = (ptr + 1) & 15;
            st = SENT;
            break;
        case SENT:                      /* our byte went out; let the controller ACK */
            wire_putn(T, 1, 1);
            st = MASTER_ACK;
            break;
        case MASTER_ACK:
            if ((w & 1) == 0) {         /* ACK: next register */
                wire_putn(T, 8, regs[ptr]);
                ptr = (ptr + 1) & 15;
                st = SENT;
            } else {                    /* NACK: release until STOP */
                wire_putn(T, 8, 0xFF);
                st = OTHER_BYTE;
            }
            break;
        case OTHER_ACK:                 /* not addressed: keep releasing */
            wire_putn(T, 8, 0xFF);
            st = OTHER_BYTE;
            break;
        case OTHER_BYTE:
            wire_putn(T, 1, 1);
            st = OTHER_ACK;
            break;
        default:
            break;
        }
    }
}
