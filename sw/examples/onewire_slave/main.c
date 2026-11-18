/*
 * Copyright (c) 2026 Tyler Hong
 * SPDX-License-Identifier: Apache-2.0
 *
 * onewire_slave: the chip as a 1-Wire device (a DS18B20-style thermometer),
 * the other side of examples/onewire. Every time slot starts with the
 * master's falling edge, which the capture unit stamps with its cycle; the
 * C code then either samples the line 30 us later (master writes) or, to
 * send a 0, pulls the line low at once and lets the timed GPIO queue release
 * it 30 us after the edge (master reads). A low longer than 400 us is a
 * reset: the device answers with a presence pulse. ROM commands: READ ROM,
 * MATCH ROM, SKIP ROM and SEARCH ROM (bit, complement, then the master's
 * choice); function: READ SCRATCHPAD (9 bytes with the Maxim CRC8).
 *
 * Pin: DQ uio[6] (P6), open drain with a pull-up.
 * Results (PG_SHARED): [0] resets seen, [1] ROM commands, [2] last function
 * command; HOST_IRQ is set once a scratchpad was read.
 */

#include "protocol.h"

#define DQ  PIN_UIO(6)
#define US(x) PG_US_TO_TICKS(x)

static const uint8_t rom[8] = { 0x28, 0x5A, 0x33, 0x01, 0x80, 0x00, 0x00, 0xD8 };   /* family 28, CRC last */
static uint8_t scratch[9] = { 0x91, 0x01, 0x4B, 0x46, 0x7F, 0xFF, 0x0C, 0x10, 0 };  /* +25.0625 C */

static uint8_t crc8(const uint8_t *d, int n)
{
    uint8_t c = 0;
    while (n--) {
        uint8_t x = *d++;
        for (int i = 0; i < 8; i++) {
            uint8_t mix = (c ^ x) & 1u;
            c >>= 1;
            if (mix) c ^= 0x8C;
            x >>= 1;
        }
    }
    return c;
}

static int reset_seen;

/* The slot that began at t is over when the line is high again. A low of
 * more than 400 us was a reset: answer with a presence pulse, return -1. */
static int end_of_slot(uint32_t t)
{
    while (!pin_read(DQ)) {
        if (now() - t > US(400)) {
            while (!pin_read(DQ)) { }                    /* end of the reset pulse */
            uint32_t r = now();
            pin_oe_at(DQ, 1, r + US(25));                /* presence: 25 us later, 120 us long */
            pin_oe_at(DQ, 0, r + US(145));
            timed_flush();
            capture_arm(DQ, EDGE_FALL);                  /* forget our own presence edge */
            reset_seen = 1;
            PG_SHARED[0]++;
            return -1;
        }
    }
    return 0;
}

static int read_bit(void)                                 /* the master writes */
{
    uint32_t t = capture_wait();
    wait_until(t + US(30));
    int b = pin_read(DQ);
    return end_of_slot(t) < 0 ? -1 : b;
}

static int write_bit(int b)                               /* the master reads */
{
    uint32_t t = capture_wait();
    if (!b) {
        pin_output(DQ);                                   /* pull low within a microsecond of the edge */
        pin_oe_at(DQ, 0, t + US(30));
        timed_flush();
    }
    return end_of_slot(t);
}

static int read_byte(void)
{
    int v = 0;
    for (int i = 0; i < 8; i++) {
        int b = read_bit();
        if (b < 0) return -1;
        v |= b << i;
    }
    return v;
}

static int write_bytes(const uint8_t *d, int n)
{
    for (int i = 0; i < n; i++)
        for (int k = 0; k < 8; k++)
            if (write_bit((d[i] >> k) & 1) < 0) return -1;
    return 0;
}

/* SEARCH ROM: for each ROM bit send it and its complement, then follow the master's choice */
static int search(void)
{
    for (int i = 0; i < 64; i++) {
        int b = (rom[i >> 3] >> (i & 7)) & 1;
        if (write_bit(b) < 0 || write_bit(!b) < 0) return -1;
        int d = read_bit();
        if (d < 0 || d != b) return -1;                   /* not our branch: drop out */
    }
    return 0;
}

int main(void)
{
    scratch[8] = crc8(scratch, 8);
    pin_write(DQ, 0);                                     /* open drain: value 0, OE does the rest */
    pin_input(DQ);
    capture_arm(DQ, EDGE_FALL);
    for (;;) {
        while (!reset_seen) (void)end_of_slot(capture_wait());   /* wait for a reset */
        reset_seen = 0;
        int cmd = read_byte();
        if (cmd < 0) continue;                            /* (a reset: start over) */
        PG_SHARED[1]++;
        int sel = 1;
        if (cmd == 0x33) { write_bytes(rom, 8); continue; }          /* READ ROM: no function after */
        if (cmd == 0xF0) { search(); continue; }                      /* SEARCH ROM */
        if (cmd == 0x55) {                                            /* MATCH ROM */
            for (int i = 0; i < 8 && sel; i++) sel = (read_byte() == rom[i]);
        } else if (cmd != 0xCC) sel = 0;                              /* SKIP ROM */
        if (!sel) continue;
        int f = read_byte();
        PG_SHARED[2] = (uint32_t)f;
        if (f == 0xBE && write_bytes(scratch, 9) == 0) host_irq(1);
    }
}
