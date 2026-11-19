/*
 * Copyright (c) 2026 Tyler Hong
 * SPDX-License-Identifier: Apache-2.0
 *
 * sdi12: an SDI-12 (v1.4) data recorder, the bus of environmental sensors
 * (soil moisture, weather, water level). One wire, 1200 baud, 7 data bits,
 * even parity, 1 stop bit, negative logic (marking = low = idle). Each
 * command starts with a BREAK (spacing >= 12 ms) and marking (>= 8.33 ms):
 * one 25-bit word here. The engine drives the line only while a word is
 * shifting (WIRE_OE_FRAME), so the sensor can answer on the same wire; RX
 * reads the recorder's own characters back first.
 *
 * Sequence: "?!" (address query), "aI!" (identification), "aM!" (start
 * measurement: reply atttn), wait for the service request "a<CR><LF>",
 * then "aD0!" (send data: values like +21.5-3.25). Values are returned as
 * integers x 1000.
 *
 * Pin: uio[1] (P1), to an SDI-12 line driver (or 5 V level shifter).
 * Results (PG_SHARED): [2] 0x600D000C when done (0xBAD0000C + step on a
 * failure), [3] sensor address (ASCII), [4] measurement time (s) | values
 * << 16, [5..] values x 1000, [16..] identification string (4 chars per
 * word, little endian).
 */

#include "protocol.h"

#define SD   W3
#define PIN  PIN_UIO(1)
#define FMT  SER_FMT(7, SER_EVEN, 1, 1)
#define MS   PG_US_TO_TICKS(1000)

static char resp[40];

/* Send a command after a BREAK; read the response up to <LF>. Returns its
 * length without <CR><LF>, or -1. */
static int sdi_cmd(const char *c, uint32_t first_ticks)
{
    int n = 0, b;
    wire_putn(SD, 25, 0x7FFFu);                    /* 15 bits spacing (12.5 ms), 10 marking */
    for (const char *p = c; *p; p++) ser_putc(SD, FMT, (uint8_t)*p);
    block_timeout(40u * MS);
    (void)SD->RX;                                  /* the BREAK, read back */
    for (const char *p = c; *p; p++)
        if (ser_getc(SD, FMT) != *p) goto fail;    /* our own characters */
    block_timeout(first_ticks);                    /* sensor: 8.33 ms marking, then data */
    for (;;) {
        b = ser_getc(SD, FMT);
        if (block_timed_out() || b < 0 || n >= (int)sizeof resp) goto fail;
        resp[n++] = (char)b;
        block_timeout(12u * MS);                   /* next character: 8.33 ms + 1.66 ms max gap */
        if (b == '\n') break;
    }
    block_timeout(0);
    if (n < 2 || resp[n - 2] != '\r') return -1;
    return n - 2;
fail:
    block_timeout(0);
    return -1;
}

/* Parse "+21.5-3.25" style values into x 1000 integers */
static uint32_t sdi_values(const char *s, int n, volatile uint32_t *out, uint32_t max)
{
    uint32_t k = 0;
    int i = 0;
    while (i < n && k < max && (s[i] == '+' || s[i] == '-')) {
        int neg = s[i++] == '-';
        int32_t ip = 0, fp = 0, fd = 0;
        while (i < n && s[i] >= '0' && s[i] <= '9') ip = ip * 10 + (s[i++] - '0');
        if (i < n && s[i] == '.')
            for (i++; i < n && s[i] >= '0' && s[i] <= '9'; i++)
                if (fd < 3) { fp = fp * 10 + (s[i] - '0'); fd++; }
        for (; fd < 3; fd++) fp *= 10;
        int32_t v = ip * 1000 + fp;
        out[k++] = (uint32_t)(neg ? -v : v);
    }
    return k;
}

int main(void)
{
    char cmd[5];
    int n;
    PG_SHARED[2] = 0xBAD0000Cu;
    wire_config(SD, (wire_cfg_t){ WIRE_EN | WIRE_TX | WIRE_RX | WIRE_RX_ASYNC | WIRE_OE_FRAME | WIRE_LEN(10),
                                  PG_BIT_FIX8(1200) >> 8, WIRE_PINS(PIN, PIN, 0),
                                  WIRE_FRAC(PG_BIT_FIX8(1200)) | WIRE_RESYNC });
    host_irq(1);                                   /* ready */
    delay_ticks(20u * MS);

    if (sdi_cmd("?!", 30u * MS) != 1) goto done;   /* address query: "a" */
    char a = resp[0];
    PG_SHARED[3] = (uint8_t)a;
    PG_SHARED[2]++;
    cmd[0] = a; cmd[1] = 'I'; cmd[2] = '!'; cmd[3] = 0;
    if ((n = sdi_cmd(cmd, 30u * MS)) < 20) goto done;
    for (int i = 0; i < n && i < 36; i++) {
        if ((i & 3) == 0) PG_SHARED[16 + (i >> 2)] = 0;
        PG_SHARED[16 + (i >> 2)] |= (uint32_t)(uint8_t)resp[i] << (8 * (i & 3));
    }
    PG_SHARED[2]++;
    cmd[1] = 'M';
    if (sdi_cmd(cmd, 30u * MS) != 5 || resp[0] != a) goto done;   /* "atttn" */
    uint32_t ttt = (uint32_t)(resp[1] - '0') * 100u + (uint32_t)(resp[2] - '0') * 10u + (uint32_t)(resp[3] - '0');
    uint32_t nv = (uint32_t)(resp[4] - '0');
    PG_SHARED[4] = ttt | (nv << 16);
    PG_SHARED[2]++;
    if (ttt) {                                     /* service request "a<CR><LF>" within ttt s */
        block_timeout(ttt * PG_F_CPU + 100u * MS);
        int c0 = ser_getc(SD, FMT), c1 = ser_getc(SD, FMT), c2 = ser_getc(SD, FMT);
        int to = block_timed_out();
        block_timeout(0);
        if (to || c0 != a || c1 != '\r' || c2 != '\n') goto done;
    }
    PG_SHARED[2]++;
    cmd[1] = 'D'; cmd[2] = '0'; cmd[3] = '!'; cmd[4] = 0;
    if ((n = sdi_cmd(cmd, 30u * MS)) < 2 || resp[0] != a) goto done;
    if (sdi_values(resp + 1, n - 1, &PG_SHARED[5], 9) != nv) goto done;
    PG_SHARED[2] = 0x600D000Cu;
done:
    host_irq(0);
    pg_park();
}
