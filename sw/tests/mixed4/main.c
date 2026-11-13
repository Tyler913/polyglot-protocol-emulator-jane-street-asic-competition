/*
 * Copyright (c) 2026 Tyler Hong
 * SPDX-License-Identifier: Apache-2.0
 *
 * mixed4: four different protocols at the same time, one per Wire Engine,
 * as four cooperative tasks (pt.h) called round-robin from main(). Used by
 * test/test_mixed.py and by the 4 KB capacity study.
 *
 *   W0 (full) CAN 2.0A 500 kbit/s: send a frame, receive it back, check ACK + CRC
 *              TXD uio[0], RXD ui[2]
 *   W1 (full) SPI controller mode 0, 5 MHz: write 16 bytes to an SPI RAM, read back
 *              SCK uo[2], MOSI uo[3], CS uo[4] (GPIO), MISO ui[1]
 *   W2 (lite) UART 8N1 1 Mbaud: echo every byte + 1
 *              TX uo[1], RX ui[0]
 *   W3 (lite) I2C controller 400 kHz: page write to a 24C02-style EEPROM,
 *              ACK polling, random read
 *              SDA uio[1], SCL uio[2] (open drain)
 *
 * Host parameter (PG_SHARED): [0] number of bytes to echo on the UART.
 * Results: [1] 0x600D0004 when all tasks are done; [2] bytes echoed, [3] UART
 * framing errors; [4] SPI result, [5] I2C result (0x600D0000 | error count),
 * [6] I2C polls, [7] CAN frame acknowledged with a good CRC;
 * [8..11] SPI read back, [12..13] I2C read back;
 * [16 + 2k], [17 + 2k]: start and end time (NOW) of task k = CAN, SPI, UART, I2C.
 */

#include "protocol.h"
#include "pt.h"

#define CAN  W0
#define SPI  W1
#define UART W2
#define I2C  W3

#define CS_PIN PIN_UO(4)
#define N_SPI  16
#define N_I2C  8
#define EEPROM 0x50
#define WADDR  0x10

static inline int tx_idle(pg_wire_t *w) { return !(w->STATUS & WIRE_TX_ACTIVE); }

static void mark(int k, int end) { PG_SHARED[16 + 2 * k + end] = now(); }

/* ---- CAN ---------------------------------------------------------------- */

static int can_task(pt_t *pt)
{
    static const uint8_t msg[4] = {0xDE, 0xAD, 0xBE, 0xEF};
    static uint32_t words[4], nw, x, t;
    static can_frame_t f;

    PT_BEGIN(pt);
    mark(0, 0);
    can_init(CAN, PIN_UIO(0), PIN_UI(2), 500000);
    t = now() + PG_US_TO_TICKS(30);                 /* bus idle (11 recessive bits) */
    PT_WAIT(pt, time_reached(t));
    can_send(CAN, 0x123, msg, 4);
    nw = 0;
    for (;;) {                                      /* our own frame comes back on RX */
        PT_WAIT(pt, wire_rx_count(CAN) > 0);
        x = wire_get(CAN);
        if (WIRE_IS_END(x)) break;
        if (nw < 4) words[nw] = x;
        nw++;
    }
    PG_SHARED[7] = can_parse(words, nw, x, &f) == 0 && f.id == 0x123 && f.acked && f.crc_ok;
    mark(0, 1);
    PT_END(pt);
}

/* ---- SPI RAM ------------------------------------------------------------ */

static uint8_t spi_buf[2 + N_SPI], spi_back[2 + N_SPI];

static int spi_frame(pt_t *pt, uint8_t *rx)          /* one CS frame of 2 + N_SPI bytes */
{
    static int i;
    PT_BEGIN(pt);
    pin_write(CS_PIN, 0);
    for (i = 0; i < 2 + N_SPI; i++) {
        wire_put(SPI, spi_buf[i]);
        PT_WAIT(pt, wire_rx_count(SPI) > 0);
        uint8_t b = (uint8_t)wire_get(SPI);
        if (rx) rx[i] = b;
    }
    PT_WAIT(pt, tx_idle(SPI));
    pin_write(CS_PIN, 1);
    PT_END(pt);
}

static int spi_task(pt_t *pt)
{
    static pt_t sub;
    PT_BEGIN(pt);
    mark(1, 0);
    pin_output(CS_PIN);
    pin_write(CS_PIN, 1);
    wire_config(SPI, WIRE_SPI_CTRL(PIN_UO(2), PIN_UO(3), PIN_UI(1), 0, 5000000));
    spi_buf[0] = 0x02;                              /* WRITE, address, data */
    spi_buf[1] = 0x40;
    for (int i = 0; i < N_SPI; i++) spi_buf[2 + i] = (uint8_t)(0x3C + 17 * i);
    PT_SPAWN(pt, &sub, spi_frame(&sub, 0));
    spi_buf[0] = 0x03;                              /* READ: data bytes are ignored */
    PT_SPAWN(pt, &sub, spi_frame(&sub, spi_back));
    {
        uint32_t bad = 0;
        for (int i = 2; i < 2 + N_SPI; i++) bad += spi_back[i] != spi_buf[i];
        for (int i = 0; i < N_SPI / 4; i++)
            PG_SHARED[8 + i] = spi_back[2 + 4 * i] | (spi_back[3 + 4 * i] << 8) |
                               ((uint32_t)spi_back[4 + 4 * i] << 16) | ((uint32_t)spi_back[5 + 4 * i] << 24);
        PG_SHARED[4] = 0x600D0000u | bad;
    }
    mark(1, 1);
    PT_END(pt);
}

/* ---- UART echo ---------------------------------------------------------- */

static int uart_task(pt_t *pt)
{
    static uint32_t n, total;
    PT_BEGIN(pt);
    mark(2, 0);
    total = PG_SHARED[0];
    wire_config(UART, WIRE_UART_8N1(PIN_UO(1), PIN_UI(0), 1000000));
    for (n = 0; n < total; ) {
        PT_WAIT(pt, wire_rx_count(UART) > 0);
        int c = uart_getc(UART);
        if (c < 0) { PG_SHARED[3]++; continue; }
        uart_putc(UART, (uint8_t)(c + 1));
        PG_SHARED[2] = ++n;
    }
    PT_WAIT(pt, tx_idle(UART));
    mark(2, 1);
    PT_END(pt);
}

/* ---- I2C EEPROM --------------------------------------------------------- */
/* START/STOP and byte transfers from protocol.h, rewritten as sub-tasks that
 * wait instead of blocking (wire_flush, delay_ticks and SCL polling). */

static i2c_t bus;
static uint32_t i2c_t0;

static int i2c_start_t(pt_t *pt)
{
    PT_BEGIN(pt);
    if (bus.w->CTRL & WIRE_TX) {                    /* repeated START */
        PT_WAIT(pt, tx_idle(bus.w));
        i2c__pins(0, 1, &bus);
        i2c__engine(&bus, 0);
        i2c_t0 = now() + bus.quarter;
        PT_WAIT(pt, time_reached(i2c_t0));
        i2c__pins(0, 0, &bus);
        PT_WAIT(pt, pin_read(bus.scl));
        i2c_t0 = now() + 2u * bus.quarter;
        PT_WAIT(pt, time_reached(i2c_t0));
    }
    i2c__pins(1, 0, &bus);
    i2c_t0 = now() + 2u * bus.quarter;
    PT_WAIT(pt, time_reached(i2c_t0));
    i2c__pins(1, 1, &bus);
    i2c_t0 = now() + bus.quarter;
    PT_WAIT(pt, time_reached(i2c_t0));
    i2c__engine(&bus, 1);
    PT_END(pt);
}

static int i2c_stop_t(pt_t *pt)
{
    PT_BEGIN(pt);
    PT_WAIT(pt, tx_idle(bus.w));
    i2c__pins(1, 1, &bus);
    i2c__engine(&bus, 0);
    i2c_t0 = now() + bus.quarter;
    PT_WAIT(pt, time_reached(i2c_t0));
    i2c__pins(1, 0, &bus);
    PT_WAIT(pt, pin_read(bus.scl));
    i2c_t0 = now() + 2u * bus.quarter;
    PT_WAIT(pt, time_reached(i2c_t0));
    i2c__pins(0, 0, &bus);
    i2c_t0 = now() + 2u * bus.quarter;
    PT_WAIT(pt, time_reached(i2c_t0));
    PT_END(pt);
}

/* One byte: 8 data bits + the ACK slot. Write: tx is the byte, ack_bit = 1
 * (release SDA); i2c_acked tells whether the target acknowledged. Read:
 * tx = 0xFF, ack_bit = 0 to acknowledge, 1 for the last byte; the byte is in i2c_rx. */
static uint8_t i2c_rx, i2c_acked;

static int i2c_byte_t(pt_t *pt, uint8_t tx, uint32_t ack_bit)
{
    PT_BEGIN(pt);
    wire_putn(bus.w, 8, tx);
    wire_putn(bus.w, 1, ack_bit);
    PT_WAIT(pt, wire_rx_count(bus.w) >= 2);
    i2c_rx = (uint8_t)wire_get(bus.w);
    i2c_acked = (wire_get(bus.w) & 1u) == 0u;
    PT_END(pt);
}

static int i2c_task(pt_t *pt)
{
    static pt_t sub;
    static int i, ok;
    static uint32_t polls, bad;
    static uint8_t data[N_I2C], back[N_I2C];

    PT_BEGIN(pt);
    mark(3, 0);
    i2c_init(&bus, I2C, PIN_UIO(1), PIN_UIO(2), 400000);
    for (i = 0; i < N_I2C; i++) data[i] = (uint8_t)(0xA0 + 7 * i);
    ok = 1;

    /* page write */
    PT_SPAWN(pt, &sub, i2c_start_t(&sub));
    PT_SPAWN(pt, &sub, i2c_byte_t(&sub, EEPROM << 1, 1));
    ok &= i2c_acked;
    PT_SPAWN(pt, &sub, i2c_byte_t(&sub, WADDR, 1));
    ok &= i2c_acked;
    for (i = 0; i < N_I2C; i++) {
        PT_SPAWN(pt, &sub, i2c_byte_t(&sub, data[i], 1));
        ok &= i2c_acked;
    }
    PT_SPAWN(pt, &sub, i2c_stop_t(&sub));

    /* ACK polling while the EEPROM writes */
    for (polls = 1; ; polls++) {
        PT_SPAWN(pt, &sub, i2c_start_t(&sub));
        PT_SPAWN(pt, &sub, i2c_byte_t(&sub, EEPROM << 1, 1));
        if (i2c_acked) break;
        PT_SPAWN(pt, &sub, i2c_stop_t(&sub));
    }
    PG_SHARED[6] = polls;

    /* random read */
    PT_SPAWN(pt, &sub, i2c_byte_t(&sub, WADDR, 1));
    ok &= i2c_acked;
    PT_SPAWN(pt, &sub, i2c_start_t(&sub));
    PT_SPAWN(pt, &sub, i2c_byte_t(&sub, (EEPROM << 1) | 1, 1));
    ok &= i2c_acked;
    for (i = 0; i < N_I2C; i++) {
        PT_SPAWN(pt, &sub, i2c_byte_t(&sub, 0xFF, i < N_I2C - 1 ? 0u : 1u));
        back[i] = i2c_rx;
    }
    PT_SPAWN(pt, &sub, i2c_stop_t(&sub));

    bad = ok ? 0 : 0x80;
    for (i = 0; i < N_I2C; i++) bad += back[i] != data[i];
    PG_SHARED[12] = back[0] | (back[1] << 8) | ((uint32_t)back[2] << 16) | ((uint32_t)back[3] << 24);
    PG_SHARED[13] = back[4] | (back[5] << 8) | ((uint32_t)back[6] << 16) | ((uint32_t)back[7] << 24);
    PG_SHARED[5] = 0x600D0000u | bad;
    mark(3, 1);
    PT_END(pt);
}

/* ---- main --------------------------------------------------------------- */

int main(void)
{
    static pt_t pt[4];
    for (int i = 1; i < 32; i++) PG_SHARED[i] = 0;
    host_irq(0);
    for (;;) {
        int done = can_task(&pt[0]) & spi_task(&pt[1]) & uart_task(&pt[2]) & i2c_task(&pt[3]);
        if (done) break;
    }
    PG_SHARED[1] = 0x600D0004u;
    host_irq(1);
    pg_park();
}
