/*
 * Copyright (c) 2026 Tyler Hong
 * SPDX-License-Identifier: Apache-2.0
 *
 * protocol.h: the whole programming interface of the Polyglot chip.
 *
 * Version 2 (W3): cycle counter, GPIO, timed GPIO (cycle-exact edges), edge
 * capture, blocking-access timeout, and the Wire Engines (v2: per-word
 * length, RX sample delay, open-drain clock with clock stretching both ways,
 * late data launch, frame gates with events, arbitration detect). UART, SPI
 * and I2C are not hardware blocks: they are generic engine parameters and a
 * few lines of C, defined at the end of this file.
 *
 * Build with stock GCC, for example:
 *   riscv-none-elf-gcc -march=rv32ec -mabi=ilp32e -Os -ffreestanding -nostdlib
 */

#ifndef POLYGLOT_PROTOCOL_H
#define POLYGLOT_PROTOCOL_H

#include <stdint.h>

/* ---- Clock ------------------------------------------------------------ */

#ifndef PG_F_CPU
#define PG_F_CPU 50000000u          /* system clock in Hz */
#endif

#define PG_US_TO_TICKS(us)  ((uint32_t)((uint64_t)(us) * PG_F_CPU / 1000000u))
#define PG_NS_TO_TICKS(ns)  ((uint32_t)((uint64_t)(ns) * PG_F_CPU / 1000000000u))

/* Cycles per bit for a bit rate in Hz, rounded to nearest. */
#define PG_BIT_TICKS(hz)    ((uint32_t)((PG_F_CPU + (hz) / 2u) / (hz)))

/* ---- Pin numbering ------------------------------------------------------ */

/* 0..7: uio[0..7] (bidirectional), 8..13: uo[0..5] (outputs), 16..20: ui[0..4] (inputs) */
#define PIN_UIO(n)  (n)
#define PIN_UO(n)   (8 + (n))
#define PIN_UI(n)   (16 + (n))

/* ---- System registers --------------------------------------------------- */

#define PG_SYS_BASE 0x80000000u
#define PG_REG(off) (*(volatile uint32_t *)(PG_SYS_BASE + (off)))

#define PG_ID          PG_REG(0x00)
#define PG_NOW         PG_REG(0x04)
#define PG_GPIO_OUT    PG_REG(0x10)
#define PG_GPIO_OE     PG_REG(0x14)
#define PG_GPIO_IN     PG_REG(0x18)
#define PG_HOST_MBOX   PG_REG(0x20)
#define PG_HOST_IRQ    PG_REG(0x24)
#define PG_TG_TIME     PG_REG(0x30)
#define PG_TG_PUSH     PG_REG(0x34)
#define PG_TG_STATUS   PG_REG(0x38)
#define PG_TG_WAIT     PG_REG(0x3C)
#define PG_CAP_CFG     PG_REG(0x40)
#define PG_CAP_TIME    PG_REG(0x44)
#define PG_PARK        PG_REG(0x48)
#define PG_BLK_TIMEOUT PG_REG(0xF0)
#define PG_BLK_FLAGS   PG_REG(0xF4)

/* Last 256 bytes of SRAM: shared with the host (parameters in, results out).
 * The stack starts below it. */
#define PG_SHARED   ((volatile uint32_t *)0x00000F00u)

/* ---- Time ---------------------------------------------------------------- */

/* Current cycle count (wraps every 2^32 cycles). */
static inline uint32_t now(void) { return PG_NOW; }

/* True once the cycle counter has reached t (wrap-safe for spans < 2^31). */
static inline int time_reached(uint32_t t) { return (int32_t)(now() - t) >= 0; }

/* Busy-wait until cycle t. */
static inline void wait_until(uint32_t t) { while (!time_reached(t)) { } }

static inline void delay_ticks(uint32_t n) { wait_until(now() + n); }

/* ---- GPIO ------------------------------------------------------------------ */

static inline void pin_output(int pin) { if (pin < 8) PG_GPIO_OE |= 1u << pin; }
static inline void pin_input(int pin)  { if (pin < 8) PG_GPIO_OE &= ~(1u << pin); }

static inline void pin_write(int pin, int v)
{
    uint32_t m = 1u << pin;
    PG_GPIO_OUT = v ? (PG_GPIO_OUT | m) : (PG_GPIO_OUT & ~m);
}

static inline void pin_toggle(int pin) { PG_GPIO_OUT ^= 1u << pin; }

static inline int pin_read(int pin) { return (PG_GPIO_IN >> pin) & 1u; }

/* ---- Timed GPIO -------------------------------------------------------------
 * The hardware applies each change in exactly the cycle in which now() == t,
 * however long the C code took to get there. Software only has to schedule
 * ahead. Changes queue up (4 deep); a call blocks while the queue is full.
 * Schedule in time order. A change whose time has passed is applied at once
 * and sets the LATE flag. Pins 0..13 only.
 */

/* At cycle t, set every pin in mask to the matching bit of value (one cycle, all pins together). */
static inline void pins_out_at(uint32_t mask, uint32_t value, uint32_t t)
{
    PG_TG_TIME = t;
    PG_TG_PUSH = ((mask & 0x3FFFu) << 16) | (value & 0x3FFFu);
}

static inline void pin_out_at(int pin, int v, uint32_t t)
{
    pins_out_at(1u << pin, v ? 1u << pin : 0u, t);
}

/* At cycle t, enable (oe = 1) or release (oe = 0) the driver of a uio pin. */
static inline void pin_oe_at(int pin, int oe, uint32_t t)
{
    PG_TG_TIME = t;
    PG_TG_PUSH = 0x80000000u | (1u << (16 + pin)) | (oe ? 1u << pin : 0u);
}

/* Block until every scheduled change has happened; returns now(). */
static inline uint32_t timed_flush(void) { return PG_TG_WAIT; }

/* True if a change happened later than scheduled since the last call. */
static inline int timed_late(void)
{
    int late = (PG_TG_STATUS >> 8) & 1u;
    PG_TG_STATUS = 1u << 8;
    return late;
}

/* ---- Edge capture --------------------------------------------------------- */

#define EDGE_RISE 1u
#define EDGE_FALL 2u
#define EDGE_BOTH 3u

/* Arm the capture channel: the next matching edge on pin is timestamped. */
static inline void capture_arm(int pin, uint32_t edge) { PG_CAP_CFG = (uint32_t)pin | (edge << 8); }

/* True if an edge has been captured and not read yet. */
static inline int capture_ready(void) { return (PG_TG_STATUS >> 16) & 1u; }

/* Block until an edge is captured; return the cycle in which the pin changed. Re-arms. */
static inline uint32_t capture_wait(void) { return PG_CAP_TIME; }

/* ---- Blocking accesses ------------------------------------------------------
 * FIFO and capture accesses stall the CPU until they can complete. With a
 * timeout set, a stalled access gives up after that many cycles: a write is
 * dropped, a read returns 0xFFFFFFFF and block_timed_out() becomes true.
 */

static inline void block_timeout(uint32_t ticks) { PG_BLK_TIMEOUT = ticks; }   /* 0 = forever */

static inline int block_timed_out(void)
{
    int f = PG_BLK_FLAGS & 1u;
    PG_BLK_FLAGS = 1u;
    return f;
}

/* Wait for an edge on pin, at most timeout ticks (0 = forever).
 * Returns 0 and stores the edge time in *t, or -1 on timeout. */
static inline int pin_wait_edge(int pin, uint32_t edge, uint32_t timeout, uint32_t *t)
{
    capture_arm(pin, edge);
    block_timeout(timeout);
    uint32_t v = capture_wait();
    int to = block_timed_out();
    block_timeout(0);
    if (to) return -1;
    *t = v;
    return 0;
}

/* ---- Host interface --------------------------------------------------------- */

static inline void host_irq(int v) { PG_HOST_IRQ = v ? 1u : 0u; }

/* Stop using the bus (the host can then read results from SRAM). Never returns. */
static inline void __attribute__((noreturn)) pg_park(void)
{
    block_timeout(0);
    for (;;) (void)PG_PARK;
}

/* ---- Wire Engines -----------------------------------------------------------
 * A Wire Engine is a protocol-agnostic serial engine: a TX FIFO feeding a
 * shift register, an RX shift register feeding an RX FIFO, and a bit clock
 * (internal divider or an external clock pin). Words are 1..32 bits.
 */

typedef struct {
    volatile uint32_t TX;       /* push a word (blocks while full) */
    volatile uint32_t RX;       /* pop a word (blocks while empty) */
    volatile uint32_t STATUS;
    volatile uint32_t CTRL;
    volatile uint32_t DIV;      /* bit period in cycles (>= 2) */
    volatile uint32_t PINS;
    volatile uint32_t FLUSH;    /* read blocks until everything is sent */
    uint32_t _pad[25];
    volatile uint32_t TXN[32];  /* TXN[n-1] = x pushes an n-bit word */
} pg_wire_t;

#define W0 ((pg_wire_t *)0x80000100u)
#define W1 ((pg_wire_t *)0x80000200u)

/* CTRL bits */
#define WIRE_EN          (1u << 0)
#define WIRE_TX          (1u << 1)
#define WIRE_RX          (1u << 2)
#define WIRE_MSB_FIRST   (1u << 3)
#define WIRE_IDLE_HIGH   (1u << 4)
#define WIRE_RX_ASYNC    (1u << 5)    /* RX frame starts when the line leaves idle */
#define WIRE_CLK_EXT     (1u << 6)    /* bit clock = edges on the CLK pin */
#define WIRE_CLK_OUT     (1u << 7)    /* drive the bit clock on the CLK pin */
#define WIRE_CPOL        (1u << 8)
#define WIRE_CPHA        (1u << 9)
#define WIRE_OE_ALWAYS   (0u << 10)
#define WIRE_OE_FRAME    (1u << 10)   /* drive only while a word is shifting */
#define WIRE_OE_OPEN_DRAIN (2u << 10)
#define WIRE_OE_NEVER    (3u << 10)
#define WIRE_RX_DELAY(k) (((uint32_t)(k) & 7u) << 12)   /* sample k cycles late (internal clock) */
#define WIRE_TX_LATE     (1u << 15)   /* data changes a quarter bit after the bit start */
#define WIRE_LEN(n)      ((((uint32_t)(n) - 1u) & 31u) << 16)
#define WIRE_CLK_OD      (1u << 21)   /* clock pin open drain */
#define WIRE_CLK_WAIT    (1u << 22)   /* wait while a target holds the clock low */
#define WIRE_GATE_LOW    (1u << 23)   /* frame while the GATE pin is low (chip select) */
#define WIRE_GATE_I2C    (2u << 23)   /* frame from START to STOP */
#define WIRE_EVENTS      (1u << 25)   /* frame start/end markers in the RX FIFO */
#define WIRE_STRETCH     (1u << 26)   /* external clock: hold it low while TX is empty at a word boundary */
#define WIRE_ARB_STOP    (1u << 27)   /* stop driving after a lost arbitration */

#define WIRE_PINS(tx, rx, clk)  ((uint32_t)(tx) | ((uint32_t)(rx) << 8) | ((uint32_t)(clk) << 16))
#define WIRE_GATE_PIN(g)        ((uint32_t)(g) << 24)

/* RX FIFO markers (WIRE_EVENTS); data words never have bit 31 set unless LEN is 32 */
#define WIRE_EV_START  0x80000001u
#define WIRE_EV_END    0x80000002u

/* STATUS fields */
#define WIRE_TX_LEVEL(s)  ((s) & 0xFu)
#define WIRE_RX_LEVEL(s)  (((s) >> 8) & 0xFu)
#define WIRE_TX_ACTIVE    (1u << 16)
#define WIRE_RX_BUSY      (1u << 17)
#define WIRE_FRAME_ACTIVE (1u << 18)
#define WIRE_STRETCHING   (1u << 19)
#define WIRE_RX_OVERFLOW  (1u << 24)
#define WIRE_ARB_LOST     (1u << 25)

typedef struct {
    uint32_t ctrl;
    uint32_t div;
    uint32_t pins;
} wire_cfg_t;

/* Disable (clears FIFOs and state), set the bit clock and pins, then enable. */
static inline void wire_config(pg_wire_t *w, wire_cfg_t c)
{
    w->CTRL = 0;
    w->DIV  = c.div;
    w->PINS = c.pins;
    w->CTRL = c.ctrl;
}

static inline void     wire_put(pg_wire_t *w, uint32_t x) { w->TX = x; }
static inline void     wire_putn(pg_wire_t *w, int n, uint32_t x) { w->TXN[n - 1] = x; }
static inline uint32_t wire_get(pg_wire_t *w)             { return w->RX; }
static inline int      wire_rx_count(pg_wire_t *w)        { return (int)WIRE_RX_LEVEL(w->STATUS); }
static inline void     wire_flush(pg_wire_t *w)           { (void)w->FLUSH; }

/* ---- UART: just parameters ------------------------------------------------------
 * A UART frame is a 10-bit word sent LSB first: start bit 0, 8 data bits,
 * stop bit 1. The line idles high, and RX starts on the falling start edge.
 */

#define WIRE_UART_8N1(tx_pin, rx_pin, baud) ((wire_cfg_t){                      \
    WIRE_EN | WIRE_TX | WIRE_RX | WIRE_IDLE_HIGH | WIRE_RX_ASYNC | WIRE_LEN(10), \
    PG_BIT_TICKS(baud), WIRE_PINS(tx_pin, rx_pin, 0) })

static inline void uart_putc(pg_wire_t *w, uint8_t b) { w->TX = 0x200u | ((uint32_t)b << 1); }

static inline void uart_puts(pg_wire_t *w, const char *s) { while (*s) uart_putc(w, (uint8_t)*s++); }

/* Next received byte (blocks), or -1 if the frame was bad (start bit not 0,
 * stop bit not 1) or a blocking timeout hit. */
static inline int uart_getc(pg_wire_t *w)
{
    uint32_t f = w->RX;
    if ((f & 0x201u) != 0x200u) return -1;
    return (int)((f >> 1) & 0xFFu);
}

/* ---- SPI controller: just parameters ----------------------------------------
 * 8-bit words, MSB first, internal bit clock driven on SCK. Mode 0..3 sets
 * CPOL/CPHA. RX samples MISO in step with each bit. Chip select is a GPIO.
 */

#define WIRE_SPI_MODE(m)  (((m) & 2u ? WIRE_CPOL : 0u) | ((m) & 1u ? WIRE_CPHA : 0u))

#define WIRE_SPI_CTRL(sck_pin, mosi_pin, miso_pin, mode, hz) \
    WIRE_SPI_CTRL_DLY(sck_pin, mosi_pin, miso_pin, mode, hz, 0)

/* Above about F/8 the MISO input path (synchronizer + device delay) needs a
 * later sample point: rx_delay cycles (0..7), e.g. 3 at F/4. */
#define WIRE_SPI_CTRL_DLY(sck_pin, mosi_pin, miso_pin, mode, hz, rx_delay) ((wire_cfg_t){ \
    WIRE_EN | WIRE_TX | WIRE_RX | WIRE_MSB_FIRST | WIRE_CLK_OUT | WIRE_SPI_MODE(mode) | WIRE_LEN(8) | \
    WIRE_RX_DELAY(rx_delay), PG_BIT_TICKS(hz), WIRE_PINS(mosi_pin, miso_pin, sck_pin) })

static inline void spi_select(int cs_pin) { pin_write(cs_pin, 0); }

/* Wait until the last bit is out (SCK back at idle), then raise CS. */
static inline void spi_deselect(pg_wire_t *w, int cs_pin) { wire_flush(w); pin_write(cs_pin, 1); }

/* Full-duplex transfer of one byte. */
static inline uint8_t spi_xfer(pg_wire_t *w, uint8_t b) { w->TX = b; return (uint8_t)w->RX; }

/* Full-duplex transfer of n bytes, keeping the engine busy between bytes.
 * tx == 0 sends 0xFF; rx == 0 discards what is received. */
static inline void spi_transfer(pg_wire_t *w, const uint8_t *tx, uint8_t *rx, uint32_t n)
{
    uint32_t sent = 0, got = 0;
    while (got < n) {
        if (sent < n && sent - got < 2u) {      /* stay ahead, but never overflow RX */
            w->TX = tx ? tx[sent] : 0xFFu;
            sent++;
        } else {
            uint8_t r = (uint8_t)w->RX;
            if (rx) rx[got] = r;
            got++;
        }
    }
}

/* ---- SPI target (external clock) ---------------------------------------------
 * The controller's SCK clocks the engine; chip select (active low) frames the
 * transfer: a frame start restarts the bit count, and with WIRE_EVENTS the RX
 * FIFO gets WIRE_EV_START / WIRE_EV_END around each frame's bytes. MISO is
 * driven only while CS is low (OE_FRAME). Queue the first reply byte before
 * the controller starts clocking.
 */

#define WIRE_SPI_TARGET(sck_pin, mosi_pin, miso_pin, cs_pin, mode) ((wire_cfg_t){       \
    WIRE_EN | WIRE_TX | WIRE_RX | WIRE_MSB_FIRST | WIRE_CLK_EXT | WIRE_SPI_MODE(mode) | WIRE_LEN(8) | \
    WIRE_GATE_LOW | WIRE_EVENTS | WIRE_OE_FRAME, \
    2u, WIRE_PINS(miso_pin, mosi_pin, sck_pin) | WIRE_GATE_PIN(cs_pin) })

/* ---- I2C controller: parameters plus START/STOP in C --------------------------
 * SDA is the engine's TX and RX pin (open drain), SCL its clock pin (open
 * drain, waits while a target stretches it). Each byte is an 8-bit word
 * followed by a 1-bit word for the ACK slot. SDA changes a quarter bit after
 * SCL falls (TX_LATE). START and STOP are made with GPIO while the engine
 * lets go of the pins. Both pins must be uio pins with external pull-ups.
 */

#define WIRE_I2C_CTRL_BITS (WIRE_EN | WIRE_RX | WIRE_MSB_FIRST | WIRE_IDLE_HIGH | WIRE_OE_OPEN_DRAIN | \
                            WIRE_CLK_OD | WIRE_CLK_WAIT | WIRE_TX_LATE | WIRE_LEN(8))

typedef struct {
    pg_wire_t *w;
    int sda, scl;
    uint32_t quarter;   /* quarter bit in cycles, for START/STOP timing */
} i2c_t;

static inline void i2c__pins(int sda_low, int scl_low, const i2c_t *b)
{
    uint32_t m = (1u << b->sda) | (1u << b->scl);
    PG_GPIO_OUT &= ~m;                                  /* open drain: 0 when driven */
    PG_GPIO_OE = (PG_GPIO_OE & ~m) | (sda_low ? 1u << b->sda : 0u) | (scl_low ? 1u << b->scl : 0u);
}

static inline void i2c__engine(const i2c_t *b, int on)   /* engine owns SDA/SCL when on */
{
    b->w->CTRL = WIRE_I2C_CTRL_BITS | (on ? WIRE_TX | WIRE_CLK_OUT : 0u);
}

static inline void i2c_init(i2c_t *b, pg_wire_t *w, int sda_pin, int scl_pin, uint32_t hz)
{
    b->w = w; b->sda = sda_pin; b->scl = scl_pin;
    b->quarter = PG_BIT_TICKS(hz) / 4u;
    i2c__pins(0, 0, b);                                  /* bus released */
    w->CTRL = 0;
    w->DIV  = PG_BIT_TICKS(hz);
    w->PINS = WIRE_PINS(sda_pin, sda_pin, scl_pin);
    i2c__engine(b, 0);
}

/* START, or repeated START after a transfer. */
static inline void i2c_start(const i2c_t *b)
{
    if (b->w->CTRL & WIRE_TX) {         /* repeated START: the engine holds SCL low */
        wire_flush(b->w);
        i2c__pins(0, 1, b);             /* GPIO takes over: SCL low, SDA released */
        i2c__engine(b, 0);
        delay_ticks(b->quarter);
        i2c__pins(0, 0, b);             /* SCL up */
        while (!pin_read(b->scl)) { }   /* a target may still stretch it */
        delay_ticks(2u * b->quarter);
    }
    i2c__pins(1, 0, b);                 /* SDA falls while SCL is high: START */
    delay_ticks(2u * b->quarter);
    i2c__pins(1, 1, b);                 /* SCL low */
    delay_ticks(b->quarter);
    i2c__engine(b, 1);                  /* engine holds SCL low, releases SDA */
}

static inline void i2c_stop(const i2c_t *b)
{
    wire_flush(b->w);
    i2c__pins(1, 1, b);                 /* SDA and SCL low */
    i2c__engine(b, 0);
    delay_ticks(b->quarter);
    i2c__pins(1, 0, b);                 /* SCL up */
    while (!pin_read(b->scl)) { }
    delay_ticks(2u * b->quarter);
    i2c__pins(0, 0, b);                 /* SDA rises while SCL is high: STOP */
    delay_ticks(2u * b->quarter);
}

/* Send a byte; returns 1 if the target acknowledged. */
static inline int i2c_write(const i2c_t *b, uint8_t x)
{
    wire_putn(b->w, 8, x);
    wire_putn(b->w, 1, 1u);             /* release SDA for the ACK */
    (void)wire_get(b->w);               /* our own byte, read back */
    return (wire_get(b->w) & 1u) == 0u;
}

/* Receive a byte; ack = 1 to acknowledge (more to come), 0 for the last byte. */
static inline uint8_t i2c_read(const i2c_t *b, int ack)
{
    wire_putn(b->w, 8, 0xFFu);          /* release SDA, the target drives */
    wire_putn(b->w, 1, ack ? 0u : 1u);
    uint8_t x = (uint8_t)wire_get(b->w);
    (void)wire_get(b->w);
    return x;
}

/* ---- I2C target: parameters --------------------------------------------------
 * SCL is the external clock (sampled on rising edges); START/STOP frame the
 * transfer and appear as WIRE_EV_START / WIRE_EV_END in the RX FIFO. At each
 * word boundary with nothing queued the engine stretches SCL, so C decides
 * every ACK and every data byte at its own pace. Per byte: queue 0xFF (8 bits)
 * to receive or the data byte to send, then the ACK slot as a 1-bit word
 * (0 = ACK, 1 = release). The RX FIFO returns every word, including the
 * 1-bit ACK slots.
 */

#define WIRE_I2C_TARGET(sda_pin, scl_pin) ((wire_cfg_t){                         \
    WIRE_EN | WIRE_TX | WIRE_RX | WIRE_MSB_FIRST | WIRE_IDLE_HIGH | WIRE_OE_OPEN_DRAIN | \
    WIRE_CLK_EXT | WIRE_GATE_I2C | WIRE_EVENTS | WIRE_STRETCH | WIRE_LEN(8),        \
    2u, WIRE_PINS(sda_pin, sda_pin, scl_pin) })

#endif /* POLYGLOT_PROTOCOL_H */
