/*
 * Copyright (c) 2026 Tyler Hong
 * SPDX-License-Identifier: Apache-2.0
 *
 * sbus: Futaba S.BUS, the RC receiver to flight controller link, both ways,
 * on a lite engine. S.BUS is 100000 baud, 8 data bits, even parity, 2 stop
 * bits, on an inverted line (idle low); a frame is 25 bytes: 0x0F, 16
 * channels of 11 bits packed LSB first into 22 bytes, a flag byte (bit 0
 * channel 17, bit 1 channel 18, bit 2 frame lost, bit 3 failsafe) and 0x00.
 * All of it is ser_* settings and C: no S.BUS hardware.
 *
 * Pins: decode: S.BUS in on ui[0] (P16); encode: S.BUS out on uo[0] (P8).
 * Host parameters (PG_SHARED): [0] mode (0 = decode frames, as a flight
 * controller; 1 = send frames, as a receiver), [1] frames (1..3).
 * Encode: [8..15] channels 2k (low half) and 2k+1 (high half), [16] flags.
 * Results: [2] 0x600D0007 when done, [3] frames rejected (parity, stop bit,
 * header or footer, or a gap inside the frame); decoded frame k:
 * [32 + 9k .. 39 + 9k] channels in pairs as above, [40 + 9k] flags.
 */

#include "protocol.h"

#define SB W2
#define FMT SER_FMT(8, SER_EVEN, 2, 1)
#define BYTE_TICKS (12u * 500u)          /* 12 bits of 10 us */

static uint8_t buf[25];

/* One frame into buf, or -1. Frames are found by the gap before them, as
 * flight controllers do: 0x0F may also occur inside a frame. */
static int sbus_read(void)
{
    int b;
    block_timeout(2u * BYTE_TICKS);
    while ((void)ser_getc(SB, FMT), !block_timed_out()) { }   /* until 2 bytes of silence */
    block_timeout(0);
    if (ser_getc(SB, FMT) != 0x0F) return -1;         /* header (waits for the next frame) */
    buf[0] = 0x0F;
    block_timeout(2u * BYTE_TICKS);                   /* bytes follow back to back */
    for (int i = 1; i < 25; i++) {
        b = ser_getc(SB, FMT);
        if (block_timed_out() || b < 0) { block_timeout(0); return -1; }
        buf[i] = (uint8_t)b;
    }
    block_timeout(0);
    return buf[24] == 0x00 ? 0 : -1;
}

static void sbus_unpack(volatile uint32_t *out)
{
    uint32_t acc = 0, nb = 0, ch = 0;
    for (int i = 1; i <= 22; i++) {
        acc |= (uint32_t)buf[i] << nb;
        nb += 8;
        while (nb >= 11) {
            uint32_t v = acc & 0x7FFu;
            if (ch & 1u) out[ch >> 1] |= v << 16; else out[ch >> 1] = v;
            ch++;
            acc >>= 11;
            nb -= 11;
        }
    }
    out[8] = buf[23];
}

static void sbus_send(void)
{
    uint32_t acc = 0, nb = 0;
    ser_putc(SB, FMT, 0x0F);
    for (uint32_t ch = 0; ch < 16; ch++) {
        uint32_t v = PG_SHARED[8 + (ch >> 1)];
        acc |= ((ch & 1u ? v >> 16 : v) & 0x7FFu) << nb;
        nb += 11;
        while (nb >= 8) {
            ser_putc(SB, FMT, acc & 0xFFu);
            acc >>= 8;
            nb -= 8;
        }
    }
    ser_putc(SB, FMT, PG_SHARED[16] & 0xFFu);
    ser_putc(SB, FMT, 0x00);
}

int main(void)
{
    uint32_t mode = PG_SHARED[0], n = PG_SHARED[1], k = 0;
    if (n < 1u || n > 3u) n = 1u;
    PG_SHARED[3] = 0;
    ser_init(SB, PIN_UO(0), PIN_UI(0), 100000, FMT);
    host_irq(1);                                      /* ready */
    if (mode == 0u) {
        while (k < n) {
            if (sbus_read() < 0) { PG_SHARED[3]++; continue; }
            sbus_unpack(&PG_SHARED[32 + 9 * k]);
            k++;
        }
    } else {
        uint32_t t = now();
        for (; k < n; k++) {
            sbus_send();
            t += PG_US_TO_TICKS(7000);                /* high-speed mode: a frame every 7 ms */
            if (k + 1u < n) wait_until(t);
        }
        wire_flush(SB);
    }
    PG_SHARED[2] = 0x600D0007u;
    host_irq(0);
    pg_park();
}
