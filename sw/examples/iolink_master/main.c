/*
 * Copyright (c) 2026 Tyler Hong
 * SPDX-License-Identifier: Apache-2.0
 *
 * iolink_master: an IO-Link (IEC 61131-9, SDCI) master port, to a transceiver
 * that drives the C/Q line (e.g. one with separate TxD/RxD pins).
 *
 *  1. Wake-up request (WURQ): TxD inverted for 80 us, by timed GPIO before
 *     the engine takes the pin (75..85 us allowed).
 *  2. Establish communication: a TYPE_0 read of MinCycleTime at COM3
 *     (230.4 kbaud), then COM2 (38.4k), then COM1 (4.8k), until the device
 *     answers with a good checksum. UART frames are 8E1 (ser_*).
 *  3. STARTUP: read Direct Parameter page 1, 0x02..0x0B (MinCycleTime,
 *     M-sequence capability, RevisionID, ProcessDataIn/Out, VendorID,
 *     DeviceID), write MasterCycleTime (= the device's MinCycleTime), then
 *     MasterCommand DevicePreoperate
 *     (0x9A) and DeviceOperate (0x99).
 *  4. OPERATE: cyclic TYPE_2 M-sequences that read the device's process
 *     data input (length from ProcessDataIn) and one on-request octet.
 *
 * Message check: XOR of all octets (checksum bits zero) and the seed 0x52,
 * compressed from 8 to 6 bits (specification A.1.6). MC octet: bit 7 read,
 * bits 6..5 channel (01 = page), bits 4..0 address; CKT: bits 7..6
 * M-sequence type, 5..0 checksum; device CKS: bit 7 event, bit 6 PD invalid.
 *
 * Pins: TxD uo[4] (P12), RxD ui[1] (P17).
 * Host parameters (PG_SHARED): [0] OPERATE cycles (1..8).
 * Results: [2] 0x600D000B when done (0xBAD0000B if no device answered),
 * [3] baud rate found, [4] messages with a bad or missing reply,
 * [8..17] page 1 octets 0x02..0x0B, [20 + k] process data of cycle k
 * (big endian), [28 + k] its CKS flags (bits 7..6).
 */

#include "protocol.h"

#define IOL   W2
#define TXD   PIN_UO(4)
#define FMT   SER_FMT(8, SER_EVEN, 1, 0)

static uint32_t bit_ticks;
static uint8_t rx[8];

static uint32_t ck6(const uint8_t *p, uint32_t n)
{
    uint32_t c = 0x52u;
    while (n--) c ^= *p++;
    uint32_t b = c;
    return ((((b >> 7) ^ (b >> 5) ^ (b >> 3) ^ (b >> 1)) & 1u) << 5) |
           ((((b >> 6) ^ (b >> 4) ^ (b >> 2) ^ b) & 1u) << 4) |
           ((((b >> 7) ^ (b >> 6)) & 1u) << 3) | ((((b >> 5) ^ (b >> 4)) & 1u) << 2) |
           ((((b >> 3) ^ (b >> 2)) & 1u) << 1) | (((b >> 1) ^ b) & 1u);
}

/* One M-sequence: m = master message (the CKT checksum is filled in here),
 * nr = device reply length (CKS included). The C/Q line is one wire, so RX
 * first reads back the master's own octets. Returns 0 with the reply in rx
 * (checksum good), or -1. */
static int mseq(uint8_t *m, uint32_t nm, uint32_t nr)
{
    int b;
    m[1] = (uint8_t)((m[1] & 0xC0u) | ck6(m, nm));
    for (uint32_t i = 0; i < nm; i++) ser_putc(IOL, FMT, m[i]);
    block_timeout(40u * bit_ticks);
    for (uint32_t i = 0; i < nm; i++)
        if (ser_getc(IOL, FMT) != m[i]) goto fail;  /* read-back */
    for (uint32_t i = 0; i < nr; i++) {
        b = ser_getc(IOL, FMT);
        if (block_timed_out() || b < 0) goto fail;
        rx[i] = (uint8_t)b;
    }
    block_timeout(0);
    uint32_t cks = rx[nr - 1u];
    rx[nr - 1u] &= 0xC0u;
    if (ck6(rx, nr) != (cks & 0x3Fu)) { rx[nr - 1u] = (uint8_t)cks; goto fail2; }
    rx[nr - 1u] = (uint8_t)cks;
    return 0;
fail:
    block_timeout(0);
fail2:
    PG_SHARED[4]++;
    delay_ticks(30u * bit_ticks);                   /* let the line settle */
    return -1;
}

static int page_read(uint32_t a, uint8_t *v)        /* TYPE_0 read, page channel */
{
    uint8_t m[2] = { (uint8_t)(0xA0u | a), 0x00 };
    if (mseq(m, 2, 2) < 0) return -1;
    *v = rx[0];
    return 0;
}

static int page_write(uint32_t a, uint8_t v)        /* TYPE_0 write */
{
    uint8_t m[3] = { (uint8_t)(0x20u | a), 0x00, v };
    return mseq(m, 3, 1);
}

int main(void)
{
    static const uint32_t rates[3] = { 230400u, 38400u, 4800u };
    static const uint32_t ticks[3] = { PG_F_CPU / 230400u, PG_F_CPU / 38400u, PG_F_CPU / 4800u };
    uint32_t cycles = PG_SHARED[0], baud = 0;
    uint8_t v;
    if (cycles < 1u || cycles > 8u) cycles = 1u;
    PG_SHARED[3] = PG_SHARED[4] = 0;
    PG_SHARED[2] = 0xBAD0000Bu;

    pin_write(TXD, 1);                              /* line idle (GPIO owns TxD) */
    host_irq(1);                                    /* ready */
    uint32_t t = now() + PG_US_TO_TICKS(200);
    pin_out_at(TXD, 0, t);                          /* WURQ: 80 us inverted */
    pin_out_at(TXD, 1, t + PG_US_TO_TICKS(80));
    timed_flush();
    delay_ticks(PG_US_TO_TICKS(500));               /* TREN: device ready */

    for (int i = 0; i < 3 && !baud; i++) {          /* COM3, COM2, COM1 */
        bit_ticks = ticks[i];
        ser_init(IOL, TXD, PIN_UI(1), rates[i], FMT);
        if (page_read(0x02, &v) == 0) baud = rates[i];
        else if (i < 2) delay_ticks(27u * ticks[i + 1]);   /* TDMT, in bit times of the next rate */
    }
    if (!baud) goto done;
    PG_SHARED[3] = baud;

    for (uint32_t a = 0x02; a <= 0x0Bu; a++) {      /* STARTUP: Direct Parameter page 1 */
        if (page_read(a, &v) < 0) goto done;
        PG_SHARED[8 + a - 2u] = v;
    }
    uint32_t cyc = PG_SHARED[8], cyc_us;            /* MinCycleTime: bits 7..6 time base, 5..0 multiplier */
    if ((cyc >> 6) == 0u) cyc_us = 100u * (cyc & 0x3Fu);
    else cyc_us = 6400u + 400u * (cyc & 0x3Fu);    /* time base 01 (10 and 11 not used here) */
    if (page_write(0x01, cyc) < 0 || page_write(0x00, 0x9A) < 0 ||   /* DevicePreoperate */
        page_write(0x00, 0x99) < 0) goto done;                        /* DeviceOperate */

    uint32_t pdi = PG_SHARED[8 + 5 - 2];            /* ProcessDataIn: bit 7 octets, 4..0 length */
    uint32_t npd = (pdi & 0x80u) ? (pdi & 0x1Fu) + 1u : ((pdi & 0x1Fu) + 7u) / 8u;
    if (npd > 4u) npd = 4u;
    t = now();
    for (uint32_t k = 0; k < cycles; k++) {         /* OPERATE: TYPE_2, OD + PD in + CKS */
        uint8_t m[2] = { 0xA2u, 0x80u };            /* read page 0x02 as on-request data */
        if (mseq(m, 2, 2u + npd) == 0) {
            uint32_t pd = 0;
            for (uint32_t i = 0; i < npd; i++) pd = (pd << 8) | rx[1 + i];
            PG_SHARED[20 + k] = pd;
            PG_SHARED[28 + k] = rx[1 + npd] & 0xC0u;
        }
        t += PG_US_TO_TICKS(1) * cyc_us;            /* MasterCycleTime */
        wait_until(t);
    }
    PG_SHARED[2] = 0x600D000Bu;
done:
    host_irq(0);
    pg_park();
}
