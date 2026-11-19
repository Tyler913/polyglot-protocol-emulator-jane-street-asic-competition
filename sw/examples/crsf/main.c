/*
 * Copyright (c) 2026 Tyler Hong
 * SPDX-License-Identifier: Apache-2.0
 *
 * crsf: the flight-controller side of CRSF, the serial protocol of TBS
 * Crossfire and ExpressLRS receivers, on a lite engine. 420000 baud 8N1,
 * both directions. A frame is [0xC8][len][type][payload][CRC-8], len counts
 * type + payload + CRC, and the CRC (polynomial 0xD5, "DVB-S2") covers type
 * and payload. Decoded: RC_CHANNELS_PACKED (0x16: 16 channels x 11 bits,
 * packed LSB first) and LINK_STATISTICS (0x14). After every good RC frame
 * the chip answers with a BATTERY_SENSOR telemetry frame (0x08).
 *
 * Pins: from the receiver's TX on ui[1] (P17), to its RX on uo[1] (P9).
 * Host parameters (PG_SHARED): [0] RC frames to receive (1..3), [1] battery
 * voltage (0.1 V), [2] current (0.1 A), [3] remaining (%).
 * Results: [4] 0x600D0008 when done, [5] frames rejected (CRC or length),
 * [6] uplink RSSI 1 | LQ << 8 from the last LINK_STATISTICS; RC frame k:
 * [32 + 8k .. 39 + 8k] channels 2i (low half) and 2i+1 (high half).
 * Telemetry: voltage, current, capacity used = RC frames so far (mAh),
 * remaining.
 */

#include "protocol.h"

#define CR W2
#define FMT SER_FMT(8, SER_NONE, 1, 0)

static uint8_t f[64];

static uint8_t crc8_d5(const uint8_t *p, uint32_t n)
{
    uint32_t c = 0;
    while (n--) {
        c ^= *p++;
        for (int i = 0; i < 8; i++) c = (c & 0x80u) ? ((c << 1) ^ 0xD5u) & 0xFFu : (c << 1) & 0xFFu;
    }
    return (uint8_t)c;
}

/* One frame into f ([0] = len, [1] = type, ...), or -1 */
static int crsf_read(void)
{
    int b;
    block_timeout(0);
    while (ser_getc(CR, FMT) != 0xC8) { }            /* sync */
    block_timeout(2000u);                            /* bytes follow back to back (1190 cycles each) */
    b = ser_getc(CR, FMT);
    if (b < 2 || b > 62) goto bad;
    f[0] = (uint8_t)b;
    for (int i = 1; i <= f[0]; i++) {
        b = ser_getc(CR, FMT);
        if (block_timed_out() || b < 0) goto bad;
        f[i] = (uint8_t)b;
    }
    block_timeout(0);
    return crc8_d5(&f[1], f[0] - 1u) == f[f[0]] ? 0 : -1;
bad:
    block_timeout(0);
    return -1;
}

static void crsf_send(const uint8_t *p, uint32_t n)  /* p = type + payload */
{
    ser_putc(CR, FMT, 0xC8);
    ser_putc(CR, FMT, n + 1u);
    for (uint32_t i = 0; i < n; i++) ser_putc(CR, FMT, p[i]);
    ser_putc(CR, FMT, crc8_d5(p, n));
}

int main(void)
{
    uint32_t n = PG_SHARED[0], k = 0;
    if (n < 1u || n > 3u) n = 1u;
    PG_SHARED[5] = PG_SHARED[6] = 0;
    ser_init(CR, PIN_UO(1), PIN_UI(1), 420000, FMT);
    host_irq(1);                                     /* ready */
    while (k < n) {
        if (crsf_read() < 0) { PG_SHARED[5]++; continue; }
        if (f[1] == 0x14 && f[0] == 12) {            /* LINK_STATISTICS */
            PG_SHARED[6] = f[2] | ((uint32_t)f[4] << 8);
        } else if (f[1] == 0x16 && f[0] == 24) {     /* RC_CHANNELS_PACKED */
            volatile uint32_t *out = &PG_SHARED[32 + 8 * k];
            uint32_t acc = 0, nb = 0, ch = 0;
            for (int i = 2; i < 24; i++) {
                acc |= (uint32_t)f[i] << nb;
                for (nb += 8; nb >= 11; nb -= 11, acc >>= 11, ch++) {
                    uint32_t v = acc & 0x7FFu;
                    if (ch & 1u) out[ch >> 1] |= v << 16; else out[ch >> 1] = v;
                }
            }
            k++;
            uint32_t v = PG_SHARED[1], a = PG_SHARED[2];
            uint8_t t[9] = { 0x08, (uint8_t)(v >> 8), (uint8_t)v, (uint8_t)(a >> 8), (uint8_t)a,
                             0, 0, (uint8_t)k, (uint8_t)PG_SHARED[3] };   /* big endian */
            crsf_send(t, 9);
        }
    }
    wire_flush(CR);
    PG_SHARED[4] = 0x600D0008u;
    host_irq(0);
    pg_park();
}
