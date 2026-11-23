/*
 * Copyright (c) 2026 Tyler Hong
 * SPDX-License-Identifier: Apache-2.0
 *
 * can_gateway: a CAN <-> serial adapter (what a USB-CAN dongle does), speaking
 * a subset of the Lawicel / SLCAN text protocol that PC tools (python-can,
 * SocketCAN slcand, SavvyCAN, ...) use. The host side is a UART; put a USB
 * serial chip on it and a PC sees a standard CAN adapter.
 *
 *   CAN  : Wire Engine 0, classic CAN 2.0A/B at 500 kbit/s or 1 Mbit/s, the
 *          engine's CAN FD unit acknowledges good frames from other nodes.
 *          TXD uio[0] (to the transceiver), RXD ui[2].
 *   UART : Wire Engine 2, 8N1, rate from the host (default 3 Mbaud),
 *          TX uo[1], RX ui[0]; both directions through DMA rings
 *          (DMA0 receives commands, DMA1 sends text), so neither direction
 *          loses bytes while the CPU decodes a frame.
 *
 * Commands (each ends with CR; reply CR = ok, BEL = error):
 *   Sn      bit rate: S6 = 500 kbit/s, S8 = 1 Mbit/s (channel closed)
 *   O / C   open / close the channel
 *   tiiildd..        send a standard frame: 3 hex ID digits, length 0-8, data
 *   Tiiiiiiiildd..   send an extended frame (8 hex ID digits)
 *                    reply "z" (or "Z") + CR when queued
 *   Mxxx / mxxx      acceptance code / mask for received standard IDs
 *                    (simplified: a frame passes if (id & mask) == (code & mask))
 *   Z0 / Z1 timestamps off / on (4 hex digits, milliseconds 0..59999)
 *   F       status flags: bit 0 RX lost, bit 1 TX queue full, bit 3 text lost
 *   V / N   version "V0101" / serial number "NPG12"
 * Received frames go out as "tiiildd..[tttt]" or "Tiiiiiiiildd..[tttt]" + CR.
 * The chip's own frames are not echoed; a frame that loses arbitration is
 * sent again after the winner.
 *
 * Host mailbox: [0] UART baud (0 = 3 Mbaud), [1] CAN bit rate before the first
 * S command (0 = 1 Mbit/s), [2] 1 = open at start.
 * Statistics (read any time over SPI): [8] frames received, [9] forwarded,
 * [10] filtered out, [11] frames sent, [12] arbitration retries, [13] CAN RX
 * FIFO overflows, [14] text bytes dropped (output ring full), [15] command
 * errors, [16] longest main-loop pass (cycles), [17] commands, [18] heartbeat.
 */

#include "protocol.h"

#define CAN  W0
#define UART W2

/* ---- output text ring (DMA1 -> UART TX): one 32-bit word per character ---- */
#define TXR 512
static uint32_t txring[TXR], rxring[64];
static uint8_t  line[40];
static uint32_t nline;

static const char HEX[] = "0123456789ABCDEF";

static void out_flush(void)                         /* line[] -> DMA ring, or drop it whole */
{
    if (dma_room(DMA1) < nline) { PG_SHARED[14] += nline; nline = 0; return; }
    for (uint32_t i = 0; i < nline; i++) dma_put(DMA1, 0x200u | ((uint32_t)line[i] << 1));
    nline = 0;
}
static void out_c(uint32_t c) { if (nline < sizeof line) line[nline++] = (uint8_t)c; }
static void out_hex(uint32_t v, int digits) { while (digits--) out_c((uint8_t)HEX[(v >> (4 * digits)) & 15u]); }

/* ---- CAN ------------------------------------------------------------------ */
typedef struct { uint32_t id; uint8_t len, d[8]; } frame_t;

#define TXQ 8
static frame_t txq[TXQ];
static uint32_t txq_head, txq_tail;                 /* head: next to send; tail: next free */
static int      tx_busy;                            /* a frame of ours is on the bus */
static uint32_t bit_ticks, last_busy;
static int      open_, stamps;
static uint32_t acode, amask, t0_ms_ticks;

static void can_setup(uint32_t rate)
{
    canfd_init(CAN, PIN_UIO(0), PIN_UI(2), rate, rate);
    bit_ticks = PG_F_CPU / rate;
}

static void forward(uint32_t id, uint32_t len, const uint8_t *d)
{
    PG_SHARED[8]++;
    if (!(id & CAN_EFF) && ((id ^ acode) & amask)) { PG_SHARED[10]++; return; }
    nline = 0;
    if (id & CAN_EFF) { out_c('T'); out_hex(id & 0x1FFFFFFFu, 8); }
    else              { out_c('t'); out_hex(id, 3); }
    out_c('0' + len);
    for (uint32_t i = 0; i < len; i++) out_hex(d[i], 2);
    if (stamps) {
        uint32_t ms = ((now() - t0_ms_ticks) / (PG_F_CPU / 1000u)) % 60000u;
        out_hex(ms, 4);
    }
    out_c('\r');
    out_flush();
    PG_SHARED[9]++;
}

/* Frame words arrive one by one (20 destuffed bits each); keep them until the
 * end marker, then decode with the same packing as canfd__recv. */
static uint32_t fw[8], fwn;

static void frame_end(uint32_t endw)
{
    static canfd_frame_t f;
    uint32_t a[8], nb = 0, st = (endw >> 8) & 0x1F00u;
    for (uint32_t i = 0; i < 8u; i++) a[i] = 0;
    for (uint32_t i = 0; i < fwn && i < 8u; i++) {  /* pack 20-bit words MSB first */
        uint32_t k = nb >> 5, s = nb & 31u, x = fw[i] & 0xFFFFFu;
        if (k >= 7u) break;
        if (s <= 12u) a[k] |= x << (12u - s);
        else { a[k] |= x >> (s - 12u); a[k + 1u] |= x << (44u - s); }
        nb += 20u;
    }
    if (WIRE_END_BITS(endw) && nb >= 20u) nb -= 20u - WIRE_END_BITS(endw);
    fwn = 0;
    if (nb < 19u) return;
    uint32_t ide = canfd_field(a, 13, 1), hdr = ide ? 39u : 19u;
    if (nb < hdr) return;
    uint32_t dlc = canfd_field(a, hdr - 4u, 4), len = dlc > 8u ? 8u : dlc;
    if (nb < hdr + 8u * len + 17u) return;
    f.id = canfd_field(a, 1, 11);
    if (ide) f.id = (f.id << 18) | canfd_field(a, 14, 18) | CAN_EFF;
    for (uint32_t i = 0; i < len; i++) f.data[i] = (uint8_t)canfd_field(a, hdr + 8u * i, 8);
    int good = (st & (FDCTL_DLM | FDCTL_CRC_OK)) == (FDCTL_DLM | FDCTL_CRC_OK);

    if (tx_busy) {                                  /* our frame, or the one that beat it */
        frame_t *q = &txq[txq_head];
        int ours = !(CAN->STATUS & WIRE_ARB_LOST) && f.id == q->id && len == q->len;
        for (uint32_t i = 0; ours && i < len; i++) ours = f.data[i] == q->d[i];
        if (ours) {
            txq_head = (txq_head + 1u) % TXQ;
            PG_SHARED[11]++;
            tx_busy = 0;
            return;
        }
        if (CAN->STATUS & WIRE_ARB_LOST) {          /* lost: send it again later */
            wire_tx_discard(CAN);
            CAN->STATUS = WIRE_ARB_LOST;
            PG_SHARED[12]++;
            tx_busy = 0;
        }
    }
    if (good) forward(f.id, len, f.data);
}

static void can_poll(void)
{
    uint32_t st = CAN->STATUS;
    if (st & WIRE_RX_BUSY) last_busy = now();
    if (st & WIRE_RX_OVERFLOW) { PG_SHARED[13]++; CAN->STATUS = WIRE_RX_OVERFLOW; }
    while (wire_rx_count(CAN)) {
        uint32_t x = wire_get(CAN);
        if (WIRE_IS_END(x)) frame_end(x);
        else if (fwn < 8u) fw[fwn++] = x;
    }
    /* send when the bus has been idle for 4 bits (EOF seen, intermission over) */
    if (open_ && !tx_busy && txq_head != txq_tail && !(CAN->STATUS & WIRE_RX_BUSY) &&
        now() - last_busy > 4u * bit_ticks) {
        frame_t *q = &txq[txq_head];
        CAN->STATUS = WIRE_ARB_LOST;
        can_send(CAN, q->id, q->d, q->len);
        tx_busy = 1;
        last_busy = now();
    }
}

/* ---- commands ------------------------------------------------------------- */
static uint8_t cmd[32];
static uint32_t ncmd;

static int hexv(uint8_t c)
{
    if (c >= '0' && c <= '9') return c - '0';
    if (c >= 'A' && c <= 'F') return c - 'A' + 10;
    if (c >= 'a' && c <= 'f') return c - 'a' + 10;
    return -1;
}
static int hexn(const uint8_t *p, int n, uint32_t *v)
{
    *v = 0;
    while (n--) { int h = hexv(*p++); if (h < 0) return -1; *v = (*v << 4) | (uint32_t)h; }
    return 0;
}
static void reply(const char *s) { nline = 0; while (*s) out_c((uint8_t)*s++); out_flush(); }

static void command(void)
{
    uint32_t v, id;
    PG_SHARED[17]++;
    switch (cmd[0]) {
    case 'S':
        if (open_ || (cmd[1] != '6' && cmd[1] != '8')) break;
        can_setup(cmd[1] == '6' ? 500000u : 1000000u);
        reply("\r"); return;
    case 'O': if (open_) break; open_ = 1; reply("\r"); return;
    case 'C': open_ = 0; reply("\r"); return;
    case 'Z': stamps = cmd[1] == '1'; t0_ms_ticks = now(); reply("\r"); return;
    case 'V': reply("V0101\r"); return;
    case 'N': reply("NPG12\r"); return;
    case 'F':
        nline = 0; out_c('F');
        out_hex((PG_SHARED[13] ? 1u : 0u) | ((txq_tail + 1u) % TXQ == txq_head ? 2u : 0u) |
                (PG_SHARED[14] ? 8u : 0u), 2);
        out_c('\r'); out_flush(); return;
    case 'M': case 'm':
        if (ncmd != 4 || hexn(cmd + 1, 3, &v)) break;
        if (cmd[0] == 'M') acode = v; else amask = v;
        reply("\r"); return;
    case 't': case 'T': {
        int ext = cmd[0] == 'T', nd = ext ? 8 : 3;
        if (!open_ || ncmd < (uint32_t)(nd + 2) || hexn(cmd + 1, nd, &id)) break;
        uint32_t len = (uint32_t)(cmd[1 + nd] - '0');
        if (len > 8u || ncmd != 2u + (uint32_t)nd + 2u * len) break;
        if ((txq_tail + 1u) % TXQ == txq_head) break;   /* queue full */
        frame_t *q = &txq[txq_tail];
        q->id = ext ? (id & 0x1FFFFFFFu) | CAN_EFF : id & 0x7FFu;
        q->len = (uint8_t)len;
        for (uint32_t i = 0; i < len; i++) {
            if (hexn(cmd + 2 + nd + 2 * i, 2, &v)) goto bad;
            q->d[i] = (uint8_t)v;
        }
        txq_tail = (txq_tail + 1u) % TXQ;
        reply(ext ? "Z\r" : "z\r");
        return;
    }
    default: break;
    }
bad:
    PG_SHARED[15]++;
    reply("\a");
}

static void uart_poll(void)
{
    for (uint32_t k = 0; k < 8u && dma_count(DMA0); k++) {   /* bounded step */
        uint32_t f = dma_get(DMA0);
        if ((f & 0x201u) != 0x200u) continue;               /* framing error */
        uint8_t c = (uint8_t)(f >> 1);
        if (c == '\r') { if (ncmd) command(); ncmd = 0; }
        else if (ncmd < sizeof cmd) cmd[ncmd++] = c;
    }
}

int main(void)
{
    uint32_t baud = PG_SHARED[0] ? PG_SHARED[0] : 3000000u;
    uint32_t rate = PG_SHARED[1] ? PG_SHARED[1] : 1000000u;
    open_ = PG_SHARED[2] == 1u;
    for (int i = 8; i < 20; i++) PG_SHARED[i] = 0;
    amask = 0;
    wire_config(UART, WIRE_UART_8N1(PIN_UO(1), PIN_UI(0), baud));
    dma_rx(DMA0, UART, rxring, 64);
    dma_tx(DMA1, UART, txring, TXR);
    can_setup(rate);
    t0_ms_ticks = now();
    host_irq(1);
    uint32_t t = now(), worst = 0;
    for (;;) {
        can_poll();
        uart_poll();
        uint32_t t1 = now();
        if (t1 - t > worst) PG_SHARED[16] = worst = t1 - t;
        t = t1;
        PG_SHARED[18]++;
    }
}
