/*
 * Copyright (c) 2026 Tyler Hong
 * SPDX-License-Identifier: Apache-2.0
 *
 * usb_hid: a USB low-speed HID boot keyboard in plain C on Wire Engine 1,
 * with a UART receiver on Wire Engine 0 at the same time: it types "Hi",
 * then every character that arrives on the serial line (a serial-to-USB
 * keyboard). The engine does NRZI, bit stuffing, CRC16 and the end of
 * packet; this file is the whole device: control endpoint 0 (enumeration)
 * and interrupt endpoint 1. One loop sleeps in wait_events() until either
 * engine has received something.
 *
 * Timing: USB gives a device 7.5 bit times (250 cycles at 50 MHz) to answer
 * a packet. C does not have to be that fast: every answer (ACK after a DATA
 * packet, DATA after an IN token) is queued in advance and the engine sends
 * it by itself a fixed turnaround after the matching packet (RXMATCH).
 *
 * Pins: D- uio[4] (P4), D+ uio[5] (P5), 1.5 kOhm pull-up on D- (low speed);
 * UART RX on ui[0] (P16), 8N1, 115200 baud (UART_BAUD).
 * VID 0x1209 / PID 0x0001 are the pid.codes test IDs.
 * Results (PG_SHARED): [1] device address, [2] configuration, [3] reports
 * sent, [4] bus resets seen, [5] packets with a bad CRC, [6] UART characters.
 */

#include "protocol.h"

#define USB  W1
#define UART W0
#ifndef UART_BAUD
#define UART_BAUD 115200u            /* compile-time: the divider is computed by the compiler */
#endif

static const uint8_t dev_desc[18] = {
    18, 1, 0x10, 0x01, 0, 0, 0, 8,          /* USB 1.1, class per interface, EP0 size 8 */
    0x09, 0x12, 0x01, 0x00, 0x00, 0x01,     /* VID 0x1209, PID 0x0001, bcdDevice 1.00 */
    0, 0, 0, 1                              /* no strings, 1 configuration */
};

static const uint8_t report_desc[63] = {
    0x05, 0x01, 0x09, 0x06, 0xA1, 0x01, 0x05, 0x07, 0x19, 0xE0, 0x29, 0xE7, 0x15, 0x00, 0x25, 0x01,
    0x75, 0x01, 0x95, 0x08, 0x81, 0x02, 0x95, 0x01, 0x75, 0x08, 0x81, 0x01, 0x95, 0x05, 0x75, 0x01,
    0x05, 0x08, 0x19, 0x01, 0x29, 0x05, 0x91, 0x02, 0x95, 0x01, 0x75, 0x03, 0x91, 0x01, 0x95, 0x06,
    0x75, 0x08, 0x15, 0x00, 0x25, 0x65, 0x05, 0x07, 0x19, 0x00, 0x29, 0x65, 0x81, 0x00, 0xC0
};

static const uint8_t conf_desc[34] = {
    9, 2, 34, 0, 1, 1, 0, 0x80, 50,         /* 1 interface, bus powered, 100 mA */
    9, 4, 0, 0, 1, 3, 1, 1, 0,              /* HID, boot, keyboard, 1 endpoint */
    9, 0x21, 0x11, 0x01, 0, 1, 0x22, sizeof report_desc, 0,
    7, 5, 0x81, 3, 8, 0, 10                 /* EP1 IN, interrupt, 8 bytes, 10 ms */
};

/* Key queue: (modifier, keycode) pairs; each is typed as a press then a release */
static uint8_t keyq[32][2];
static uint32_t kq_head, kq_tail;
static int released = 1;                      /* the last report the host took was a release */
static int queued_press;                      /* the report waiting for the host is a key press */

static void type_char(char c)
{
    uint8_t mod = 0, key = 0;
    if (c >= 'a' && c <= 'z')      key = (uint8_t)(0x04 + c - 'a');
    else if (c >= 'A' && c <= 'Z') { key = (uint8_t)(0x04 + c - 'A'); mod = 0x02; }
    else if (c >= '1' && c <= '9') key = (uint8_t)(0x1E + c - '1');
    else if (c == '0')             key = 0x27;
    else if (c == ' ')             key = 0x2C;
    else if (c == '\n' || c == '\r') key = 0x28;
    if (!key || kq_head - kq_tail >= 32u) return;
    keyq[kq_head % 32u][0] = mod;
    keyq[kq_head % 32u][1] = key;
    kq_head++;
}

static uint8_t addr, new_addr, config, ep1_toggle;
static uint32_t nreport;
static uint8_t report[8];

/* control transfer state */
static const uint8_t *in_ptr;   /* data stage still to send (IN) */
static uint32_t in_left;
static uint8_t in_toggle;
static int in_pending, zlp_needed;
enum { CTL_IDLE, CTL_SETUP, CTL_IN_DATA, CTL_IN_STATUS, CTL_OUT_STATUS } ctl;

/* Drop an IN answer that was queued but not asked for, or wait until the
 * last answer is out. */
static void settle_tx(void)
{
    if (in_pending) {
        wire_tx_discard(USB);
        in_pending = 0;
    } else {
        wire_flush(USB);
    }
}

static void queue_ack_for_data(void)
{
    USB->RXMATCH = USB_MATCH_DATA;
    usb_queue_handshake(USB, USB_PID_ACK);
}

static void queue_in(const uint8_t *d, uint32_t n, uint8_t toggle)
{
    wire_flush(USB);                         /* the previous answer has gone out */
    USB->RXMATCH = USB_MATCH_IN;
    usb_send_data(USB, toggle ? USB_PID_DATA1 : USB_PID_DATA0, d, n, 1);
    in_pending = 1;
}

static void queue_next_in_chunk(void)
{
    uint32_t n = in_left < 8u ? in_left : 8u;
    queue_in(in_ptr, n, in_toggle);
    zlp_needed = 0;
}

static void setup_request(const uint8_t *s)
{
    uint8_t type = s[0], req = s[1];
    uint16_t value = s[2] | (s[3] << 8), length = s[6] | (s[7] << 8);
    const uint8_t *d = 0;
    uint32_t n = 0;

    in_toggle = 1;
    if (type == 0x80 && req == 6) {                         /* GET_DESCRIPTOR */
        if ((value >> 8) == 1)      { d = dev_desc;  n = sizeof dev_desc; }
        else if ((value >> 8) == 2) { d = conf_desc; n = sizeof conf_desc; }
    } else if (type == 0x81 && req == 6 && (value >> 8) == 0x22) {
        d = report_desc; n = sizeof report_desc;            /* HID report descriptor */
    } else if (type == 0x00 && req == 5) {                  /* SET_ADDRESS */
        new_addr = (uint8_t)(value & 0x7F);
    } else if (type == 0x00 && req == 9) {                  /* SET_CONFIGURATION */
        config = (uint8_t)value;
    }
    /* anything else (e.g. HID SET_IDLE): accept with an empty status stage */

    if (type & 0x80) {                                      /* device-to-host: data stage */
        if (n > length) n = length;
        in_ptr = d; in_left = n;
        ctl = CTL_IN_DATA;
        queue_next_in_chunk();
    } else {                                                /* no data stage: ZLP status */
        ctl = CTL_IN_STATUS;
        queue_in(0, 0, 1);
    }
}

static int token_ok(const uint8_t *p, uint8_t *ep)
{
    uint32_t bits = p[2] | ((uint32_t)p[3] << 8);
    if (usb_crc5(bits & 0x7FFu) != (bits >> 11)) return 0;
    *ep = (uint8_t)((bits >> 7) & 0xF);
    return (bits & 0x7F) == addr;
}

/* Next report: a press for the next queued key, or a release after a press.
 * The key leaves the queue only when the host takes the report (it may be
 * discarded if a control transfer comes first). */
static void next_report(void)
{
    for (int i = 0; i < 8; i++) report[i] = 0;
    queued_press = released && kq_tail != kq_head;
    if (queued_press) {
        report[0] = keyq[kq_tail % 32u][0];
        report[2] = keyq[kq_tail % 32u][1];
    }
    queue_in(report, 8, ep1_toggle);
}

static void report_taken(void)
{
    if (queued_press) { kq_tail++; released = 0; }
    else              released = 1;
}

static void handle(const uint8_t *p, uint32_t n)
{
    uint8_t ep, pid;

    if (n < 2 || p[0] != 0x80) return;
    pid = p[1];
    switch (pid) {
    case USB_PID_SETUP:
        if (n == 4 && token_ok(p, &ep) && ep == 0) {
            settle_tx();
            ctl = CTL_SETUP;
            queue_ack_for_data();
        }
        break;
    case USB_PID_OUT:
        if (n == 4 && token_ok(p, &ep) && ep == 0) {
            settle_tx();
            ctl = CTL_OUT_STATUS;
            queue_ack_for_data();
        }
        break;
    case USB_PID_IN:
        break;                                      /* answered by the queued DATA packet */
    case USB_PID_DATA0:
    case USB_PID_DATA1:
        if (USB->RXCRC != 0x800Du) { PG_SHARED[5]++; break; }
        if (ctl == CTL_SETUP && n == 12) {
            wire_flush(USB);                        /* our ACK is out */
            setup_request(p + 2);
        } else if (ctl == CTL_OUT_STATUS) {
            ctl = CTL_IDLE;
        }
        break;
    case USB_PID_ACK:                               /* the host took our DATA packet */
        in_pending = 0;
        if (ctl == CTL_IN_DATA) {
            uint32_t sent = in_left < 8u ? in_left : 8u;
            in_ptr += sent; in_left -= sent; in_toggle ^= 1;
            if (in_left || (sent == 8 && zlp_needed)) queue_next_in_chunk();
            else ctl = CTL_OUT_STATUS;              /* host will send OUT + ZLP */
        } else if (ctl == CTL_IN_STATUS) {
            ctl = CTL_IDLE;
            if (new_addr) { addr = new_addr; new_addr = 0; PG_SHARED[1] = addr; }
            PG_SHARED[2] = config;
            if (config) next_report();
        } else if (config) {                        /* interrupt endpoint report taken */
            ep1_toggle ^= 1;
            report_taken();
            PG_SHARED[3] = ++nreport;
            next_report();
        }
        break;
    default:
        break;
    }
}

int main(void)
{
    uint8_t pkt[16];
    uint32_t n = 0;

    for (int i = 1; i < 7; i++) PG_SHARED[i] = 0;
    type_char('H');
    type_char('i');
    wire_config(UART, WIRE_UART_8N1(PIN_UO(1), PIN_UI(0), UART_BAUD));
    usb_ls_init(USB, PIN_UIO(4), PIN_UIO(5));
    host_irq(1);

    for (;;) {
        uint32_t ev = wait_events(EV_WIRE(0) | EV_WIRE(1));   /* sleep until either engine has data */
        if (ev & EV_WIRE(0)) {
            int c = uart_getc(UART);
            if (c >= 0) { type_char((char)c); PG_SHARED[6]++; }
        }
        if (!(ev & EV_WIRE(1))) continue;
        uint32_t w = wire_get(USB);
        if (w != WIRE_EV_END) {
            if (n < sizeof pkt) pkt[n] = (uint8_t)w;
            n++;
            continue;
        }
        if (n == 0) {                               /* SE0 without a packet: keep-alive or reset */
            uint32_t t0 = now();
            while ((PG_GPIO_IN & 0x30u) == 0 && now() - t0 < PG_US_TO_TICKS(100)) { }
            if (now() - t0 > PG_US_TO_TICKS(3)) {   /* reset (2.5 us minimum) */
                wire_flush(USB);
                USB->CTRL &= ~WIRE_EN;
                usb_ls_init(USB, PIN_UIO(4), PIN_UIO(5));
                addr = new_addr = config = 0; ep1_toggle = 0; nreport = 0; released = 1;
                ctl = CTL_IDLE; in_pending = 0;
                PG_SHARED[4]++;
            }
        } else {
            handle(pkt, n);
        }
        n = 0;
    }
}
