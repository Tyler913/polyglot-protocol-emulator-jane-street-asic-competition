/*
 * Copyright (c) 2026 Tyler Hong
 * SPDX-License-Identifier: Apache-2.0
 *
 * usb_host: a USB low-speed host in plain C: the other side of
 * examples/usb_hid. It resets the bus, enumerates a device (device and
 * configuration descriptors, SET_ADDRESS, SET_CONFIGURATION) and polls its
 * interrupt IN endpoint for HID reports, keeping the bus awake with a
 * low-speed keep-alive (an EOP) every millisecond. The line layer is the
 * same engine set-up as the device (NRZI, stuffing, CRC16 on transmit and
 * receive, SE0); the host's ACK after a device's DATA packet is queued
 * before the IN token goes out and leaves by itself a fixed turnaround after
 * the packet (RXMATCH: only after a DATA packet, so a NAK is not ACKed).
 *
 * Pins: D- uio[4] (P4), D+ uio[5] (P5), 15 kOhm pull-downs (host); the
 * device's 1.5 kOhm pull-up on D- makes the idle J state.
 * Host parameters: [0] reset length in us (0 = 10000), [1] 1 to scale the
 * other waits for simulation (recovery 100 us instead of 10 ms).
 * Results (PG_SHARED): [0] VID | PID << 16, [1] configuration total length,
 * [2] interface class/subclass/protocol, [3..4] first two non-empty reports
 * (first 4 bytes), [5] NAKs seen, [6] errors, [7] 0x600D05B0 when done.
 */

#include "protocol.h"

#define USB W1
#ifdef USB_FULL_SPEED                 /* examples/usb_host_fs: 12 Mbit/s, SOF every 1 ms */
#define BIT_TICKS (PG_BIT_FIX8(12000000) >> 8)
#define USB_INIT  usb_fs_init
#else
#define BIT_TICKS (PG_BIT_FIX8(1500000) >> 8)
#define USB_INIT  usb_ls_init
#endif

static uint32_t errors, naks, last_keepalive;

/* Wait until the packet in the TX FIFO is out, EOP and J included, then
 * the inter-packet gap. Words pushed while the last one is still shifting
 * would follow it directly (no J): every packet ends with this. */
static void tx_end(void)
{
    wire_flush(USB);
    delay_ticks(2u * BIT_TICKS);
}

static void tx_token(uint8_t pid, uint32_t addr, uint32_t ep)
{
    uint32_t f = addr | (ep << 7);
    f |= usb_crc5(f) << 11;                       /* ready before the first word goes out: */
    USB->TXATTR = WIRE_A_NOCRC;                   /* at full speed a word lasts 64 cycles */
    wire_putn(USB, 16, 0x80u | ((uint32_t)pid << 8));
    wire_putn(USB, 16, f);
    usb__eop(USB);
    tx_end();
}

/* Receive one packet (PID first) within the timeout. Returns its length or -1. */
static int rx_packet(uint8_t *buf, uint32_t max)
{
    uint32_t n = 0;
    block_timeout(80u * BIT_TICKS);
    for (;;) {
        uint32_t x = wire_get(USB);
        if (block_timed_out()) { block_timeout(0); return -1; }
        if (usb_rx_word(x, buf, max, &n)) break;
    }
    block_timeout(0);
    return (int)n;
}

#ifdef USB_FULL_SPEED
/* Full speed: a start-of-frame token (PID 0xA5, 11-bit frame number, CRC5)
 * exactly every 1 ms (the spec allows +/- 0.5 us). Before each transaction:
 * if the next SOF is due within 100 us, wait for its exact cycle and send it
 * first, so no transaction overlaps it. last_keepalive = next SOF time. */
static uint32_t frame;

static void keepalive(void)
{
    if ((int32_t)(last_keepalive - now()) < (int32_t)PG_US_TO_TICKS(100)) {
        wire_flush(USB);
        uint32_t f = frame++ & 0x7FFu;
        f |= usb_crc5(f) << 11;
        wait_until(last_keepalive);
        USB->TXATTR = WIRE_A_NOCRC;
        wire_putn(USB, 16, 0x80u | (0xA5u << 8));
        wire_putn(USB, 16, f);
        usb__eop(USB);
        tx_end();
        last_keepalive += PG_US_TO_TICKS(1000);
    }
}
#else
static void keepalive(void)                      /* low-speed keep-alive: an EOP each frame */
{
    if (now() - last_keepalive >= PG_US_TO_TICKS(1000)) {
        wire_flush(USB);
        usb__eop(USB);
        tx_end();
        last_keepalive = now();
    }
}
#endif

static int expect_ack(void)
{
    uint8_t p[4];
    int n = rx_packet(p, sizeof p);
    if (n == 1 && p[0] == USB_PID_ACK) return 1;
    errors++;
    return 0;
}

static int setup(uint32_t addr, const uint8_t req[8])
{
    keepalive();
    tx_token(USB_PID_SETUP, addr, 0);
    usb_send_data(USB, USB_PID_DATA0, req, 8, 0);
    tx_end();
    return expect_ack();
}

/* IN transaction: returns the payload length, -2 for a NAK, -1 on error */
static int in_tx(uint32_t addr, uint32_t ep, int toggle, uint8_t *data)
{
    uint8_t p[12];
    keepalive();
    wire_flush(USB);
    USB->RXMATCH = USB_MATCH_DATA;
    tx_token(USB_PID_IN, addr, ep);
    usb_queue_handshake(USB, USB_PID_ACK);       /* goes out after the device's DATA */
    int n = rx_packet(p, sizeof p);
    if (n == 1 && p[0] == USB_PID_NAK) { wire_tx_discard(USB); naks++; return -2; }
    if (n < 3 || p[0] != (toggle ? USB_PID_DATA1 : USB_PID_DATA0) || USB->RXCRC != 0x800Du) {
        wire_tx_discard(USB);
        errors++;
        return -1;
    }
    for (int i = 0; i < n - 3; i++) data[i] = p[1 + i];
    wire_flush(USB);                              /* our ACK is out */
    return n - 3;
}

static int out_status(uint32_t addr)
{
    keepalive();
    tx_token(USB_PID_OUT, addr, 0);
    usb_send_data(USB, USB_PID_DATA1, 0, 0, 0);
    tx_end();
    return expect_ack();
}

/* Control read: SETUP, IN data stages (DATA1, DATA0, ...) until a short packet, OUT status */
static int control_read(uint32_t addr, const uint8_t req[8], uint8_t *buf, int max)
{
    if (!setup(addr, req)) return -1;
    int got = 0, toggle = 1;
    for (;;) {
        int n = in_tx(addr, 0, toggle, buf + got);
        if (n == -2) continue;                    /* NAK: ask again */
        if (n < 0) return -1;
        got += n;
        toggle ^= 1;
        if (n < 8 || got >= max) break;
    }
    return out_status(addr) ? got : -1;
}

/* Control write without data: SETUP, then an IN status stage (DATA1, empty) */
static int control_write(uint32_t addr, const uint8_t req[8])
{
    uint8_t d[8];
    if (!setup(addr, req)) return -1;
    for (;;) {
        int n = in_tx(addr, 0, 1, d);
        if (n == -2) continue;
        return n == 0 ? 0 : -1;
    }
}

int main(void)
{
    static uint8_t buf[64];
    uint32_t reset_us = PG_SHARED[0] ? PG_SHARED[0] : 10000u;
    uint32_t recover_us = PG_SHARED[1] ? 100u : 10000u;

    USB_INIT(USB, PIN_UIO(4), PIN_UIO(5));
    /* bus reset: SE0 for reset_us (32-bit words of SE0), then the idle J */
    USB->TXATTR = WIRE_A_SE0 | WIRE_A_NOSTUFF | WIRE_A_NOCRC;
    for (uint32_t b = PG_US_TO_TICKS(reset_us) / BIT_TICKS; b; b -= b > 32u ? 32u : b)
        wire_putn(USB, b > 32u ? 32 : (int)b, 0);
    wire_flush(USB);
    USB->TXATTR = 0;
    delay_ticks(PG_US_TO_TICKS(recover_us));
#ifdef USB_FULL_SPEED
    last_keepalive = now() + PG_US_TO_TICKS(10);    /* first SOF */
#else
    last_keepalive = now();
#endif

    static const uint8_t get_dev[8]  = { 0x80, 6, 0, 1, 0, 0, 18, 0 };
    static const uint8_t set_addr[8] = { 0x00, 5, 3, 0, 0, 0, 0, 0 };
    static const uint8_t get_cfg9[8] = { 0x80, 6, 0, 2, 0, 0, 9, 0 };
    static const uint8_t set_cfg[8]  = { 0x00, 9, 1, 0, 0, 0, 0, 0 };
    uint8_t get_cfg[8] = { 0x80, 6, 0, 2, 0, 0, 0, 0 };

    if (control_read(0, get_dev, buf, 18) == 18)
        PG_SHARED[0] = buf[8] | (buf[9] << 8) | ((uint32_t)buf[10] << 16) | ((uint32_t)buf[11] << 24);
    control_write(0, set_addr);
    delay_ticks(PG_US_TO_TICKS(recover_us / 5u));  /* SET_ADDRESS recovery (2 ms) */
    if (control_read(3, get_cfg9, buf, 9) == 9) {
        uint32_t total = buf[2] | (buf[3] << 8);
        PG_SHARED[1] = total;
        get_cfg[6] = (uint8_t)(total < sizeof buf ? total : sizeof buf);
        if (control_read(3, get_cfg, buf, get_cfg[6]) >= 18)
            PG_SHARED[2] = buf[9 + 5] | (buf[9 + 6] << 8) | ((uint32_t)buf[9 + 7] << 16);
    }
    control_write(3, set_cfg);

    /* poll the interrupt endpoint until two non-empty reports arrived */
    int toggle = 0, reports = 0;
    while (reports < 2 && errors < 8u) {
        int n = in_tx(3, 1, toggle, buf);
        if (n == -2) { delay_ticks(PG_US_TO_TICKS(100)); keepalive(); continue; }
        if (n < 0) continue;
        toggle ^= 1;
        if (n == 8 && buf[2]) {
            PG_SHARED[3 + reports++] = buf[0] | (buf[1] << 8) | ((uint32_t)buf[2] << 16) | ((uint32_t)buf[3] << 24);
        }
    }
#ifdef USB_FULL_SPEED
    for (uint32_t t0 = now(); now() - t0 < PG_US_TO_TICKS(3000); ) keepalive();   /* idle: SOFs go on */
#endif
    PG_SHARED[5] = naks;
    PG_SHARED[6] = errors;
    PG_SHARED[7] = 0x600D05B0u;
    host_irq(1);
    pg_park();
}
