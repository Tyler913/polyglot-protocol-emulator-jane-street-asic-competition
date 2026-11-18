/*
 * Copyright (c) 2026 Tyler Hong
 * SPDX-License-Identifier: Apache-2.0
 *
 * eth_ping: a 10BASE-T node that answers ARP requests and ICMP echo
 * requests (ping) for its address, in plain C. Receive on Wire Engine 1
 * (Manchester decode with clock recovery, SFD hunt, FCS check in the
 * engine), transmit on Wire Engine 0 (Manchester, differential, FCS
 * appended by the engine). Built for a 60 MHz clock: 6 cycles per bit.
 * One CPU does both directions, so while it sends a reply (about 70 us for
 * a ping) a frame arriving would overflow the 4-word RX FIFO and be counted
 * as bad: like a host waiting for each answer, send the next request after
 * the reply. While idle it sends a normal link pulse every 16 ms, which a
 * link partner needs to see before it reports a link.
 *
 * Pins: RD (line receiver output, idle low) ui[0] (P16); TD+ uio[6] (P6),
 * TD- uio[7] (P7).
 * Address: 02:00:00:00:00:02, 10.0.0.2.
 * Results (PG_SHARED): [0] frames received, [1] bad frames, [2] ARP replies,
 * [3] echo replies; HOST_IRQ pulses after each reply.
 */

#define PG_F_CPU 60000000u
#include "protocol.h"

#define RX W1
#define TX W0

static const uint8_t my_mac[6] = { 0x02, 0, 0, 0, 0, 0x02 };
static const uint8_t my_ip[4]  = { 10, 0, 0, 2 };

static uint8_t f[600] __attribute__((aligned(4)));

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
    eth_send(TX, f, (uint32_t)n);
    wire_flush(TX);
    host_irq(1);
    host_irq(0);
}

int main(void)
{
    eth_init(TX, PIN_UIO(6), PIN_UIO(7));
    eth_rx_init(RX, PIN_UI(0));
    uint32_t last_nlp = now();
    for (;;) {
        uint32_t since = now() - last_nlp;
        block_timeout(since < PG_US_TO_TICKS(15990) ? PG_US_TO_TICKS(16000) - since : 500u);
        (void)wait_events(EV_WIRE(1));                   /* a frame, or time for a link pulse */
        int idle = block_timed_out();
        block_timeout(0);
        if (now() - last_nlp >= PG_US_TO_TICKS(16000)) {
            eth_link_pulse(TX);
            last_nlp = now();
        }
        if (idle) continue;
        int n = eth_recv(RX, f, sizeof f);
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
