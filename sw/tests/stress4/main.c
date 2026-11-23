/*
 * Copyright (c) 2026 Tyler Hong
 * SPDX-License-Identifier: Apache-2.0
 *
 * stress4: the four protocols of mixed4 (CAN, SPI, UART, I2C), one per Wire
 * Engine, but with host-chosen rates and sustained traffic instead of one
 * transaction each, to find how fast each one can run while all four share
 * the CPU. Four cooperative tasks (pt.h) called round-robin from main().
 *
 *   W0 (full) CAN 2.0A: send N frames of 8 bytes back to back; each must come
 *              back on RX with the node's ACK and a good CRC
 *              TXD uio[0], RXD ui[2]
 *   W1 (full) SPI controller mode 0: R rounds of (write 32 bytes to an SPI
 *              RAM, read them back, compare), up to 4 bytes in flight
 *              SCK uo[2], MOSI uo[3], CS uo[4] (GPIO), MISO ui[1]
 *   W2 (lite) UART 8N1: echo every byte + 1 (the host sends a long stream)
 *              TX uo[1], RX ui[0]
 *   W3 (lite) I2C controller: R rounds of (8-byte page write, ACK polling,
 *              random read, compare) on a 24C02-style EEPROM
 *              SDA uio[1], SCL uio[2] (open drain)
 *
 * Parameters (PG_SHARED, from the host): [0] UART baud, [1] UART bytes
 * (bit 31: through DMA channels 0 (RX) and 1 (TX) and 64-word rings),
 * [2] SPI Hz, [3] SPI rounds, [4] I2C Hz, [5] I2C rounds, [6] CAN bit/s,
 * [7] CAN frames. A count of 0 skips that task.
 * Results: [8] 0x600D0004 when all tasks are done, [9] UART bytes echoed,
 * [10] UART framing errors, [11] UART RX FIFO overflowed, [12] SPI bytes
 * wrong, [13] I2C bytes wrong (+0x80 per missing ACK), [14] CAN frames good,
 * [15] main-loop passes, [16 + 2k], [17 + 2k]: start and end time (NOW) of
 * task k = CAN, SPI, UART, I2C; [24] CAN RX FIFO overflowed, [25] CAN words of
 * the current frame | frame << 8, [26] longest main-loop pass in cycles,
 * [27..30] longest single step of task k (CAN, SPI, UART, I2C).
 * Each step handles at most 8 UART bytes (bounded steps keep the others served);
 * the CAN task empties its FIFO each time it runs.
 */

#define CAN_STD_ONLY
#include "protocol.h"
#include "pt.h"

#define CAN  W0
#define SPI  W1
#define UART W2
#define I2C  W3

#define CS_PIN PIN_UO(4)
#define N_SPI  32
#define N_I2C  8
#define EEPROM 0x50

static inline int tx_idle(pg_wire_t *w) { return !(w->STATUS & WIRE_TX_ACTIVE); }

static void mark(int k, int end) { PG_SHARED[16 + 2 * k + end] = now(); }

/* ---- CAN ---------------------------------------------------------------- */

static int can_task(pt_t *pt)
{
    static uint8_t msg[8];
    static uint32_t words[6], nw, x, t, k;
    static can_frame_t f;

    PT_BEGIN(pt);
    mark(0, 0);
    can_init(CAN, PIN_UIO(0), PIN_UI(2), PG_SHARED[6]);
    t = now() + 11u * (PG_F_CPU / PG_SHARED[6]);    /* bus idle (11 recessive bits) */
    PT_WAIT(pt, time_reached(t));
    for (k = 0; k < PG_SHARED[7]; k++) {
        for (int i = 0; i < 8; i++) msg[i] = (uint8_t)(k * 8u + (uint32_t)i);
        can_send(CAN, 0x100u + (k & 0xFFu), msg, 8);
        nw = 0;
        for (x = 0; !WIRE_IS_END(x); ) {            /* our own frame comes back on RX */
            PT_WAIT(pt, wire_rx_count(CAN) > 0);
            while (wire_rx_count(CAN) > 0) {        /* drain the FIFO every pass */
                x = wire_get(CAN);
                if (WIRE_IS_END(x)) break;
                if (nw < 6) words[nw] = x;
                nw++;
            }
        }
        if (can_parse(words, nw, x, &f) == 0 && f.id == 0x100u + (k & 0xFFu) && f.acked && f.crc_ok &&
            f.dlc == 8 && f.data[7] == msg[7])
            PG_SHARED[14]++;
    }
    mark(0, 1);
    PT_END(pt);
}

/* ---- SPI RAM ------------------------------------------------------------ */

static uint8_t spi_buf[2 + N_SPI], spi_back[2 + N_SPI];

static int spi_frame(pt_t *pt, uint8_t *rx)          /* one CS frame, up to 4 bytes in flight */
{
    static int sent, got;
    PT_BEGIN(pt);
    pin_write(CS_PIN, 0);
    for (sent = got = 0; got < 2 + N_SPI; ) {
        while (sent < 2 + N_SPI && sent - got < 4) wire_put(SPI, spi_buf[sent++]);
        PT_WAIT(pt, wire_rx_count(SPI) > 0);
        while (wire_rx_count(SPI) > 0) {
            uint8_t b = (uint8_t)wire_get(SPI);
            if (rx) rx[got] = b;
            got++;
        }
    }
    PT_WAIT(pt, tx_idle(SPI));
    pin_write(CS_PIN, 1);
    PT_END(pt);
}

static int spi_task(pt_t *pt)
{
    static pt_t sub;
    static uint32_t r;
    PT_BEGIN(pt);
    mark(1, 0);
    pin_output(CS_PIN);
    pin_write(CS_PIN, 1);
    /* above F/8 the MISO path (pad, synchronizer, device) needs a later sample point */
    wire_config(SPI, WIRE_SPI_CTRL_DLY(PIN_UO(2), PIN_UO(3), PIN_UI(1), 0, PG_SHARED[2],
                                       PG_SHARED[2] > PG_F_CPU / 8u ? 3u : 0u));
    for (r = 0; r < PG_SHARED[3]; r++) {
        spi_buf[0] = 0x02;                          /* WRITE, address, data */
        spi_buf[1] = (uint8_t)(r * 32u);
        for (int i = 0; i < N_SPI; i++) spi_buf[2 + i] = (uint8_t)(r * 7u + 0x3Cu + 17u * (uint32_t)i);
        PT_SPAWN(pt, &sub, spi_frame(&sub, 0));
        spi_buf[0] = 0x03;                          /* READ: data bytes are ignored */
        PT_SPAWN(pt, &sub, spi_frame(&sub, spi_back));
        {
            uint32_t bad = 0;
            for (int i = 2; i < 2 + N_SPI; i++) bad += spi_back[i] != spi_buf[i];
            PG_SHARED[12] += bad;
        }
    }
    mark(1, 1);
    PT_END(pt);
}

/* ---- UART echo ---------------------------------------------------------- */

static uint32_t rx_ring[64], tx_ring[64];

static int uart_task(pt_t *pt)
{
    static uint32_t n, total, use_dma, f;
    PT_BEGIN(pt);
    mark(2, 0);
    total = PG_SHARED[1] & 0xFFFFu;
    use_dma = PG_SHARED[1] >> 31;
    wire_config(UART, WIRE_UART_8N1(PIN_UO(1), PIN_UI(0), PG_SHARED[0]));
    if (use_dma) {
        dma_rx(DMA0, UART, rx_ring, 64);
        dma_tx(DMA1, UART, tx_ring, 64);
    }
    for (n = 0; n < total; ) {
        if (use_dma) {
            PT_WAIT(pt, dma_count(DMA0) > 0 && dma_room(DMA1) > 0);
            for (uint32_t m = 0; m < 8u && n < total && dma_count(DMA0) > 0 && dma_room(DMA1) > 0; m++) {
                f = dma_get(DMA0);
                if ((f & 0x201u) != 0x200u) { PG_SHARED[10]++; continue; }
                dma_put(DMA1, 0x200u | ((((f >> 1) + 1u) & 0xFFu) << 1));
                n++;
            }
        } else {
            PT_WAIT(pt, wire_rx_count(UART) > 0);
            for (uint32_t m = 0; m < 8u && n < total && wire_rx_count(UART) > 0; m++) {
                f = wire_get(UART);
                if ((f & 0x201u) != 0x200u) { PG_SHARED[10]++; continue; }
                wire_put(UART, 0x200u | ((((f >> 1) + 1u) & 0xFFu) << 1));
                n++;
            }
        }
        PG_SHARED[9] = n;
    }
    if (use_dma) PT_WAIT(pt, (DMA1->STATUS & DMA_EMPTY) != 0u);
    PT_WAIT(pt, tx_idle(UART));
    PG_SHARED[11] = (UART->STATUS & WIRE_RX_OVERFLOW) ? 1u : 0u;
    mark(2, 1);
    PT_END(pt);
}

/* ---- I2C EEPROM (sub-tasks as in mixed4) --------------------------------- */

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
    static int i;
    static uint32_t r, a;
    static uint8_t data[N_I2C];

    PT_BEGIN(pt);
    mark(3, 0);
    i2c_init(&bus, I2C, PIN_UIO(1), PIN_UIO(2), PG_SHARED[4]);
    for (r = 0; r < PG_SHARED[5]; r++) {
        a = (r * N_I2C) & 0xF8u;
        for (i = 0; i < N_I2C; i++) data[i] = (uint8_t)(0xA0u + r + 7u * (uint32_t)i);
        /* page write */
        PT_SPAWN(pt, &sub, i2c_start_t(&sub));
        PT_SPAWN(pt, &sub, i2c_byte_t(&sub, EEPROM << 1, 1));
        if (!i2c_acked) PG_SHARED[13] += 0x80;
        PT_SPAWN(pt, &sub, i2c_byte_t(&sub, (uint8_t)a, 1));
        for (i = 0; i < N_I2C; i++)
            PT_SPAWN(pt, &sub, i2c_byte_t(&sub, data[i], 1));
        PT_SPAWN(pt, &sub, i2c_stop_t(&sub));
        /* ACK polling while the EEPROM writes */
        for (;;) {
            PT_SPAWN(pt, &sub, i2c_start_t(&sub));
            PT_SPAWN(pt, &sub, i2c_byte_t(&sub, EEPROM << 1, 1));
            if (i2c_acked) break;
            PT_SPAWN(pt, &sub, i2c_stop_t(&sub));
        }
        /* random read */
        PT_SPAWN(pt, &sub, i2c_byte_t(&sub, (uint8_t)a, 1));
        PT_SPAWN(pt, &sub, i2c_start_t(&sub));
        PT_SPAWN(pt, &sub, i2c_byte_t(&sub, (EEPROM << 1) | 1, 1));
        if (!i2c_acked) PG_SHARED[13] += 0x80;
        for (i = 0; i < N_I2C; i++) {
            PT_SPAWN(pt, &sub, i2c_byte_t(&sub, 0xFF, i < N_I2C - 1 ? 0u : 1u));
            PG_SHARED[13] += i2c_rx != data[i];
        }
        PT_SPAWN(pt, &sub, i2c_stop_t(&sub));
    }
    mark(3, 1);
    PT_END(pt);
}

/* ---- main --------------------------------------------------------------- */

/* run one task and keep its longest single step (after setup) in PG_SHARED[27 + k] */
static int timed(int k, int (*task)(pt_t *), pt_t *pt, uint32_t passes)
{
    uint32_t t = now();
    int r = task(pt);
    t = now() - t;
    if (passes > 8u && t > PG_SHARED[27 + k]) PG_SHARED[27 + k] = t;
    return r;
}

int main(void)
{
    static pt_t pt[4];
    uint32_t passes = 0, t0 = now(), worst = 0;
    for (int i = 8; i < 32; i++) PG_SHARED[i] = 0;
    host_irq(0);
    for (;;) {
        int done = 1;
        if (PG_SHARED[7]) done &= timed(0, can_task, &pt[0], passes);
        if (PG_SHARED[3]) done &= timed(1, spi_task, &pt[1], passes);
        if (PG_SHARED[1] & 0xFFFFu) done &= timed(2, uart_task, &pt[2], passes);
        if (PG_SHARED[5]) done &= timed(3, i2c_task, &pt[3], passes);
        passes++;
        uint32_t t1 = now();
        if (passes > 8u && t1 - t0 > worst) worst = t1 - t0;   /* longest pass after setup */
        t0 = t1;
        if (CAN->STATUS & WIRE_RX_OVERFLOW) PG_SHARED[24] |= 1u;
        if (done) break;
    }
    PG_SHARED[15] = passes;
    PG_SHARED[26] = worst;
    PG_SHARED[8] = 0x600D0004u;
    host_irq(1);
    pg_park();
}
