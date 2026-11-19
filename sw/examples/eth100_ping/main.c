/*
 * Copyright (c) 2026 Tyler Hong
 * SPDX-License-Identifier: Apache-2.0
 *
 * eth100_ping: a 100 Mbit/s Ethernet node that answers ARP requests and
 * ICMP echo requests (ping) for its address, through an RMII PHY (e.g. a
 * LAN8720) on the experimental RMII port (only in the RTL variant built
 * with make RMII=yes; not in the hardened chip). The port sends preamble,
 * SFD and FCS and checks the received FCS; this program only builds the
 * replies. The chip clock is the PHY's 50 MHz REF_CLK. One CPU does both
 * directions: a frame that arrives while a reply goes out is lost, so like
 * a host waiting for each answer, send the next request after the reply.
 *
 * Pins: TXD0 uo[0], TXD1 uo[1], TX_EN uo[2]; RXD0 ui[0], RXD1 ui[1],
 * CRS_DV ui[2].
 * Address: 02:00:00:00:00:02, 10.0.0.2.
 * Results (PG_SHARED): [0] frames received, [1] bad or oversized frames,
 * [2] ARP replies, [3] echo replies; HOST_IRQ pulses after each reply.
 */

#include "protocol.h"

static const uint8_t my_mac[6] = { 0x02, 0, 0, 0, 0, 0x02 };
static const uint8_t my_ip[4]  = { 10, 0, 0, 2 };

static uint32_t fw[1100 / 4];
#define f ((uint8_t *)fw)

static void copy(uint8_t *d, const uint8_t *s, int n) { while (n--) *d++ = *s++; }
static int same(const uint8_t *a, const uint8_t *b, int n) { while (n--) if (*a++ != *b++) return 0; return 1; }
static uint32_t be16(const uint8_t *p) { return ((uint32_t)p[0] << 8) | p[1]; }

/* Internet checksum (RFC 1071) of n bytes */
static uint32_t csum(const uint8_t *p, int n)
{
    uint32_t s = 0;
    for (; n > 1; n -= 2, p += 2) s += be16(p);
    if (n) s += (uint32_t)p[0] << 8;
    while (s >> 16) s = (s & 0xFFFFu) + (s >> 16);
    return ~s & 0xFFFFu;
}

static void reply(int n)
{
    copy(f, f + 6, 6);                      /* destination = the sender */
    copy(f + 6, my_mac, 6);
    while (n < 60) f[n++] = 0;              /* minimum frame (without FCS) */
    rmii_send(fw, (uint32_t)n);
    host_irq(1);
    host_irq(0);
}

int main(void)
{
    rmii_init();
    for (;;) {
        int n = rmii_recv(fw, sizeof fw);
        PG_SHARED[0]++;
        if (n < 64) { PG_SHARED[1]++; continue; }
        n -= 4;                                       /* drop the FCS */
        uint32_t type = be16(f + 12);
        if (type == 0x0806u && be16(f + 20) == 1u && same(f + 38, my_ip, 4)) {
            /* ARP request for us: answer with our MAC */
            f[21] = 2;                                /* operation: reply */
            copy(f + 32, f + 22, 10);                 /* target = the asker */
            copy(f + 22, my_mac, 6);
            copy(f + 28, my_ip, 4);
            PG_SHARED[2]++;
            reply(42);
        } else if (type == 0x0800u && f[23] == 1u && same(f + 30, my_ip, 4)) {
            int ihl = (f[14] & 15) * 4, len = (int)be16(f + 16);
            uint8_t *icmp = f + 14 + ihl;
            if (icmp[0] != 8u || 14 + len > n) continue;
            uint8_t a[4];                              /* swap the IP addresses */
            copy(a, f + 26, 4);
            copy(f + 26, f + 30, 4);
            copy(f + 30, a, 4);
            icmp[0] = 0;                              /* echo reply */
            icmp[2] = icmp[3] = 0;
            uint32_t c = csum(icmp, len - ihl);
            icmp[2] = (uint8_t)(c >> 8);
            icmp[3] = (uint8_t)c;
            PG_SHARED[3]++;
            reply(14 + len);
        }
    }
}
