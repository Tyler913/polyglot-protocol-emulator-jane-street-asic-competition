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

/* Time conversions. With a whole-MHz clock, PG_US_TO_TICKS is one 32-bit multiply, so a
 * run-time argument does not pull libgcc's 64-bit divide (about 1.4 KB) into the image.
 * PG_NS_TO_TICKS is meant for constants. */
#define PG_US_TO_TICKS(us)  ((PG_F_CPU % 1000000u == 0u)                              \
    ? (uint32_t)(us) * (PG_F_CPU / 1000000u)                                         \
    : (uint32_t)((uint64_t)(us) * PG_F_CPU / 1000000u))
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
#define PG_EVWAIT      PG_REG(0xF8)

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

/* ---- Waiting for several things at once ---------------------------------------
 * wait_events(mask) sleeps (the CPU's bus access stalls) until one of the
 * events in mask is pending and returns the pending ones: EV_WIRE(n) = engine
 * n has received data, EV_CAPTURE = an edge was captured. */
#define EV_WIRE(n)  (1u << (n))
#define EV_CAPTURE  (1u << 8)

static inline uint32_t wait_events(uint32_t mask)
{
    PG_EVWAIT = mask;
    return PG_EVWAIT;
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
    volatile uint32_t CTRL2;    /* fractional bit period, RX resync, line code, stuffing */
    volatile uint32_t TXATTR;   /* attributes of the next pushed words */
    volatile uint32_t CRCCFG;
    volatile uint32_t CRCINIT;
    volatile uint32_t RXCRC;
    volatile uint32_t TURN;     /* auto-reply delay after a received packet, cycles */
    volatile uint32_t CRCPOLY;
    volatile uint32_t RXMATCH;  /* auto-reply only for packets whose word k matches */
    uint32_t _pad[17];
    volatile uint32_t TXN[32];  /* TXN[n-1] = x pushes an n-bit word */
} pg_wire_t;

#define W0 ((pg_wire_t *)0x80000100u)
#define W1 ((pg_wire_t *)0x80000200u)
/* W2 and W3 are lite engines: no line codes, bit stuffing, differential output,
 * CRC, TX attributes or auto-reply (USB, 10BASE-T and CAN need W0 or W1). */
#define W2 ((pg_wire_t *)0x80000300u)
#define W3 ((pg_wire_t *)0x80000400u)

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

/* CTRL2 bits */
#define WIRE_FRAC(f)     ((uint32_t)(f) & 0xFFu)   /* bit period = DIV + f/256 cycles */
#define WIRE_RESYNC      (1u << 8)                 /* async RX realigns on every edge */
#define WIRE_NRZI        (1u << 9)                 /* line code: a 0 bit toggles the line */
#define WIRE_MANCHESTER  (2u << 9)                 /* line code (TX): ~bit then bit in each bit time (IEEE 802.3) */
#define WIRE_STUFF(n)    (((uint32_t)(n) & 7u) << 11)  /* insert/remove a bit after n equal bits */
#define WIRE_STUFF_EQUAL (1u << 14)                /* runs of equal bits (CAN); default: runs of ones (USB) */
#define WIRE_DIFF        (1u << 15)                /* CLK pin = complement of TX; SE0 = RX and GATE pins low */
#define WIRE_RX_PACKET   (1u << 16)                /* async RX continues word after word until SE0 */
#define WIRE_TAIL(n)     (((uint32_t)(n) & 3u) << 17)  /* drive n idle bits after the last word */
#define WIRE_EOF_RUN(n)  (((uint32_t)(n) & 7u) << 19)  /* RX_PACKET also ends after n idle bits (CAN) */

/* TXATTR bits (apply to the words pushed after the write) */
#define WIRE_A_NOSTUFF   (1u << 0)
#define WIRE_A_NOCRC     (1u << 1)
#define WIRE_A_CRC_INIT  (1u << 2)
#define WIRE_A_CRC_OUT   (1u << 3)                 /* append the CRC after this word */
#define WIRE_A_SE0       (1u << 4)                 /* both lines low for this word */
#define WIRE_A_AFTER_RX  (1u << 5)                 /* start TURN cycles after the next received packet */

/* RXMATCH: only packets whose word k satisfies (word & mask) == value trigger AFTER_RX words */
#define WIRE_MATCH(k, value, mask) ((((uint32_t)(k) & 15u) << 16) | (((uint32_t)(mask) & 0xFFu) << 8) | ((uint32_t)(value) & 0xFFu))

/* CRC unit: width 1..32, polynomial without the top bit; TX appends it MSB first */
static inline void wire_crc(pg_wire_t *w, uint32_t poly, int width, uint32_t init, int xorout, int rx_skip)
{
    w->CRCPOLY = poly;
    w->CRCINIT = init;
    w->CRCCFG  = ((uint32_t)(width - 1) & 31u) | (xorout ? 1u << 5 : 0u) | (((uint32_t)rx_skip & 31u) << 8);
}

/* Bit period for a rate in Hz in 1/256 cycles, rounded: DIV = x >> 8, DIV_FRAC = x & 0xFF.
 * Constants fold at compile time; a run-time rate uses 32-bit long division (two base-16
 * digits) instead of a 64-bit divide, which would add about 1.4 KB of libgcc to the image. */
static inline uint32_t pg_bit_fix8(uint32_t hz)
{
    uint32_t q = PG_F_CPU / hz, r = PG_F_CPU % hz;
    for (int i = 0; i < 2; i++) {      /* r < hz <= 2^27, so r << 4 cannot overflow */
        r <<= 4;
        q = (q << 4) | (r / hz);
        r %= hz;
    }
    return q + (2u * r >= hz);
}
#define PG_BIT_FIX8_CONST(hz) ((uint32_t)(((uint64_t)PG_F_CPU * 256u + (hz) / 2u) / (hz)))
#define PG_BIT_FIX8(hz) (__builtin_constant_p(hz) ? PG_BIT_FIX8_CONST(hz) : pg_bit_fix8(hz))

/* RX FIFO markers (WIRE_EVENTS); data words never have bit 31 set unless LEN is 32.
 * After a packet whose length is not a whole number of words, the end marker
 * carries the number of bits in the last (partial) word: WIRE_END_BITS(w). */
#define WIRE_EV_START  0x80000001u
#define WIRE_EV_END    0x80000002u
#define WIRE_IS_END(w)   (((w) & 0xFFFFE0FFu) == WIRE_EV_END)
#define WIRE_END_BITS(w) (((w) >> 8) & 31u)

/* STATUS fields */
#define WIRE_TX_LEVEL(s)  ((s) & 0xFu)
#define WIRE_RX_LEVEL(s)  (((s) >> 8) & 0xFu)
#define WIRE_TX_ACTIVE    (1u << 16)
#define WIRE_RX_BUSY      (1u << 17)
#define WIRE_FRAME_ACTIVE (1u << 18)
#define WIRE_STRETCHING   (1u << 19)
#define WIRE_RX_OVERFLOW  (1u << 24)
#define WIRE_ARB_LOST     (1u << 25)
#define WIRE_STUFF_ERROR  (1u << 26)

typedef struct {
    uint32_t ctrl;
    uint32_t div;
    uint32_t pins;
    uint32_t ctrl2;
} wire_cfg_t;

/* Disable (clears FIFOs and state), set the bit clock and pins, then enable. */
static inline void wire_config(pg_wire_t *w, wire_cfg_t c)
{
    w->CTRL  = 0;
    w->DIV   = c.div;
    w->CTRL2 = c.ctrl2;
    w->PINS  = c.pins;
    w->CTRL  = c.ctrl;
}

static inline void     wire_put(pg_wire_t *w, uint32_t x) { w->TX = x; }
static inline void     wire_putn(pg_wire_t *w, int n, uint32_t x) { w->TXN[n - 1] = x; }
static inline uint32_t wire_get(pg_wire_t *w)             { return w->RX; }
static inline int      wire_rx_count(pg_wire_t *w)        { return (int)WIRE_RX_LEVEL(w->STATUS); }
static inline void     wire_flush(pg_wire_t *w)           { (void)w->FLUSH; }
static inline void     wire_tx_discard(pg_wire_t *w)      { w->FLUSH = 1u; }   /* drop queued TX words */

/* ---- UART: just parameters ------------------------------------------------------
 * A UART frame is a 10-bit word sent LSB first: start bit 0, 8 data bits,
 * stop bit 1. The line idles high, and RX starts on the falling start edge.
 * The bit period is exact to 1/256 cycle (fractional divider) and RX
 * realigns on every edge, so odd rates like 3 Mbaud at 50 MHz work.
 */

#define WIRE_UART_8N1(tx_pin, rx_pin, baud) WIRE_UART_8N1_FIX8(tx_pin, rx_pin, PG_BIT_FIX8(baud))

/* The same with the bit period already in 1/256 cycles (PG_BIT_FIX8), e.g. from the host. */
#define WIRE_UART_8N1_FIX8(tx_pin, rx_pin, fix8) ((wire_cfg_t){                 \
    WIRE_EN | WIRE_TX | WIRE_RX | WIRE_IDLE_HIGH | WIRE_RX_ASYNC | WIRE_LEN(10), \
    (fix8) >> 8, WIRE_PINS(tx_pin, rx_pin, 0), WIRE_FRAC(fix8) | WIRE_RESYNC })

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
    WIRE_RX_DELAY(rx_delay), PG_BIT_TICKS(hz), WIRE_PINS(mosi_pin, miso_pin, sck_pin), 0u })

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
    2u, WIRE_PINS(miso_pin, mosi_pin, sck_pin) | WIRE_GATE_PIN(cs_pin), 0u })

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
    2u, WIRE_PINS(sda_pin, sda_pin, scl_pin), 0u })

/* ---- USB low speed (1.5 Mbit/s): line layer ------------------------------------
 * NRZI, stuffing after six ones, CRC16 and the SE0 end of packet are done by
 * the engine; packets, PIDs and the device state machine are C. D- is the
 * engine's TX/RX pin, D+ its CLK (complement) and GATE (SE0 detect) pin.
 * Idle J = D- high (low-speed pull-up on D-). The handshake for a received
 * packet can be queued in advance and goes out by itself after USB_TURN
 * cycles, well inside the 7.5 bit times the spec allows.
 */

#define USB_PID_OUT   0xE1u
#define USB_PID_IN    0x69u
#define USB_PID_SETUP 0x2Du
#define USB_PID_DATA0 0xC3u
#define USB_PID_DATA1 0x4Bu
#define USB_PID_ACK   0xD2u
#define USB_PID_NAK   0x5Au
#define USB_PID_STALL 0x1Eu

#ifndef USB_TURN
#define USB_TURN (PG_BIT_FIX8(1500000) * 9u / 512u)   /* 4.5 bit times after SE0 is seen */
#endif

static inline void usb_ls_init(pg_wire_t *w, int dm_pin, int dp_pin)
{
    w->CTRL    = 0;
    w->DIV     = PG_BIT_FIX8(1500000) >> 8;
    w->CTRL2   = WIRE_FRAC(PG_BIT_FIX8(1500000)) | WIRE_RESYNC | WIRE_NRZI | WIRE_STUFF(6) |
                 WIRE_DIFF | WIRE_RX_PACKET | WIRE_TAIL(1);      /* EOP: SE0 SE0, then J driven by TAIL */
    w->PINS    = WIRE_PINS(dm_pin, dm_pin, dp_pin) | WIRE_GATE_PIN(dp_pin);
    wire_crc(w, 0x8005u, 16, 0xFFFFu, 1, 16);       /* CRC16; RX skips SYNC and PID */
    w->TURN    = USB_TURN;
    w->TXATTR  = 0;
    w->CTRL    = WIRE_EN | WIRE_TX | WIRE_RX | WIRE_IDLE_HIGH | WIRE_RX_ASYNC | WIRE_OE_FRAME |
                 WIRE_EVENTS | WIRE_LEN(8);
}

static inline void usb__eop(pg_wire_t *w)
{
    w->TXATTR = WIRE_A_SE0 | WIRE_A_NOSTUFF | WIRE_A_NOCRC;
    wire_putn(w, 2, 0);                              /* SE0 for two bit times, then J (TAIL_BITS) */
}

/* DATA0/DATA1 packet (0..8 bytes); the engine appends CRC16. SYNC+PID go out
 * as one 16-bit word and the payload as words of up to 4 bytes, so a full
 * packet takes at most 4 FIFO words. after_rx: wait for a matching received
 * packet first (see RXMATCH). */
static inline void usb_send_data(pg_wire_t *w, uint8_t pid, const uint8_t *data, uint32_t n, int after_rx)
{
    w->TXATTR = WIRE_A_NOCRC | WIRE_A_CRC_INIT | (n ? 0u : WIRE_A_CRC_OUT) | (after_rx ? WIRE_A_AFTER_RX : 0u);
    wire_putn(w, 16, 0x80u | ((uint32_t)pid << 8));  /* SYNC, PID */
    for (uint32_t i = 0; i < n; ) {
        uint32_t k = n - i < 4u ? n - i : 4u, x = 0;
        for (uint32_t j = 0; j < k; j++) x |= (uint32_t)data[i + j] << (8 * j);
        i += k;
        w->TXATTR = (i == n) ? WIRE_A_CRC_OUT : 0u;
        wire_putn(w, (int)(8 * k), x);
    }
    usb__eop(w);
}

/* Handshake (ACK/NAK/STALL) sent by the engine after the next matching received packet. */
static inline void usb_queue_handshake(pg_wire_t *w, uint8_t pid)
{
    w->TXATTR = WIRE_A_NOCRC | WIRE_A_AFTER_RX;
    wire_putn(w, 16, 0x80u | ((uint32_t)pid << 8));
    usb__eop(w);
}

/* RXMATCH presets: reply only to a DATA packet, or only to an IN token */
#define USB_MATCH_DATA  WIRE_MATCH(1, 0x03, 0x03)   /* DATA0/1/2/MDATA: PID bits 1:0 = 11 */
#define USB_MATCH_IN    WIRE_MATCH(1, USB_PID_IN, 0xFF)

/* CRC5 of a token's 11 address/endpoint bits, as the 5-bit field sent after them */
static inline uint32_t usb_crc5(uint32_t bits11)
{
    uint32_t c = 0x1F;
    for (int i = 0; i < 11; i++) {
        uint32_t fb = ((bits11 >> i) & 1u) ^ ((c >> 4) & 1u);
        c = (c << 1) & 0x1Fu;
        if (fb) c ^= 0x05u;
    }
    c ^= 0x1Fu;
    uint32_t r = 0;                                  /* sent MSB first: reverse into LSB-first order */
    for (int i = 0; i < 5; i++) r |= ((c >> (4 - i)) & 1u) << i;
    return r;
}

/* ---- 10BASE-T Ethernet (TX): line layer -------------------------------------
 * Manchester code, differential TD+ (TX pin) / TD- (CLK pin), and the CRC-32
 * frame check sequence are done by the engine. Needs a clock that is a
 * multiple of 20 MHz (build with PG_F_CPU = 40 MHz: 4 cycles per bit).
 * The frame (destination ... payload, at least 60 bytes) is C data; the
 * engine adds preamble/SFD (pushed as raw words) and the FCS. Link pulses
 * (NLP, one 100 ns pulse every 16 ms) can be sent with timed GPIO.
 */

static inline void eth_init(pg_wire_t *w, int tdp_pin, int tdn_pin)
{
    w->CTRL  = 0;
    w->DIV   = PG_F_CPU / 10000000u;
    w->CTRL2 = WIRE_MANCHESTER | WIRE_DIFF | WIRE_TAIL(2);   /* TP_IDL: TD+ high after the frame */
    w->PINS  = WIRE_PINS(tdp_pin, tdp_pin, tdn_pin) | WIRE_GATE_PIN(tdn_pin);
    wire_crc(w, 0x04C11DB7u, 32, 0xFFFFFFFFu, 1, 0);
    w->TXATTR = 0;
    w->CTRL  = WIRE_EN | WIRE_TX | WIRE_OE_FRAME | WIRE_IDLE_HIGH | WIRE_LEN(32);
}

static inline void eth_send(pg_wire_t *w, const uint8_t *f, uint32_t n)
{
    w->TXATTR = WIRE_A_NOCRC;
    wire_putn(w, 32, 0x55555555u);                   /* preamble */
    wire_putn(w, 32, 0xD5555555u);                   /* preamble + SFD */
    for (uint32_t i = 0; i < n; ) {
        uint32_t left = n - i;
        uint32_t attr = (i == 0 ? WIRE_A_CRC_INIT : 0u);
        if (left >= 4u) {
            w->TXATTR = attr | (left == 4u ? WIRE_A_CRC_OUT : 0u);
            wire_putn(w, 32, f[i] | ((uint32_t)f[i + 1] << 8) | ((uint32_t)f[i + 2] << 16) | ((uint32_t)f[i + 3] << 24));
            i += 4;
        } else {
            w->TXATTR = attr | (left == 1u ? WIRE_A_CRC_OUT : 0u);
            wire_putn(w, 8, f[i]);
            i += 1;
        }
    }
}

/* ---- SWD host (ARM Serial Wire Debug): parameters plus turnarounds in C --------
 * SWCLK is the internal bit clock in SPI mode 0: SWDIO changes while SWCLK is
 * low and the target samples on the rising edge; RX samples in the same step,
 * which reads what the target drove after the previous rising edge. SWDIO is
 * both the TX and the RX pin, LSB first. The host drives it only in its own
 * phases: at each turnaround the code waits for the last bit, then rewrites
 * CTRL (with EN still set nothing is cleared) to switch the TX pin driver off
 * and RX on, or back. The clock just stops in between, which SWD allows.
 * Each phase is one push with its own length (TXN): request 8, turnaround +
 * ACK 4 (5 for a write: one more turnaround), data 32, parity 1 (+1).
 */

#define SWD_OK     1
#define SWD_WAIT   2
#define SWD_FAULT  4
#define SWD_NOACK  7      /* nobody answered (the pull-up reads 111) */
#define SWD_PARERR 8      /* read data parity error (returned by swd_xfer) */

#define WIRE_SWD_HOST  (WIRE_EN | WIRE_TX | WIRE_CLK_OUT | WIRE_OE_ALWAYS)
#define WIRE_SWD_TARGET (WIRE_EN | WIRE_TX | WIRE_RX | WIRE_CLK_OUT | WIRE_OE_NEVER)
#define WIRE_SWD(swclk_pin, swdio_pin, hz) ((wire_cfg_t){ \
    WIRE_SWD_HOST, PG_BIT_TICKS(hz), WIRE_PINS(swdio_pin, swdio_pin, swclk_pin), 0u })

/* Request byte: start, APnDP, RnW, A[3:2], even parity, stop 0, park 1. */
#define SWD_REQ(ap, rnw, a) (0x81u | ((uint32_t)(ap) << 1) | ((uint32_t)(rnw) << 2) | \
    (((uint32_t)(a) & 0xCu) << 1) |                                                 \
    ((((ap) ^ (rnw) ^ ((a) >> 2) ^ ((a) >> 3)) & 1u) << 5))

static inline uint32_t pg_parity32(uint32_t x)
{
    x ^= x >> 16;
    x ^= x >> 8;
    x ^= x >> 4;
    return (0x6996u >> (x & 15u)) & 1u;
}

/* Turnaround: wait for the last bit, then hand SWDIO over (host = 1: we drive it). */
static inline void swd_turn(pg_wire_t *w, int host)
{
    wire_flush(w);
    w->CTRL = host ? WIRE_SWD_HOST : WIRE_SWD_TARGET;
}

/* Line reset: 56 cycles with SWDIO high. */
static inline void swd_line_reset(pg_wire_t *w)
{
    wire_putn(w, 32, 0xFFFFFFFFu);
    wire_putn(w, 24, 0xFFFFFFu);
}

/* Switch an SWJ-DP from JTAG to SWD, line reset, two idle cycles. Read
 * DPIDR next: until then the DP answers nothing else. */
static inline void swd_connect(pg_wire_t *w)
{
    swd_line_reset(w);
    wire_putn(w, 16, 0xE79Eu);
    swd_line_reset(w);
    wire_putn(w, 2, 0u);
}

/* One transfer: req = SWD_REQ(ap, rnw, addr). A read stores the data in *data.
 * Returns the ACK (SWD_OK, SWD_WAIT, SWD_FAULT, SWD_NOACK) or SWD_PARERR. */
static inline int swd_xfer(pg_wire_t *w, uint32_t req, uint32_t *data)
{
    int rnw = (int)(req >> 2) & 1;
    wire_putn(w, 8, req);
    swd_turn(w, 0);
    wire_putn(w, rnw ? 4 : 5, 0u);              /* turnaround, ACK (, turnaround) */
    int ack = (int)(wire_get(w) >> 1) & 7;
    if (rnw) {
        if (ack == SWD_OK) {
            wire_putn(w, 32, 0u);
            wire_putn(w, 2, 0u);                /* parity, turnaround */
            uint32_t d = wire_get(w);
            if ((wire_get(w) & 1u) != pg_parity32(d)) ack = SWD_PARERR;
            *data = d;
        } else {
            wire_putn(w, 1, 0u);                /* turnaround */
            (void)wire_get(w);
        }
        swd_turn(w, 1);
    } else {
        swd_turn(w, 1);
        if (ack == SWD_OK) {
            wire_putn(w, 32, *data);
            wire_putn(w, 1, pg_parity32(*data));
        }
    }
    return ack;
}

/* ---- JTAG (IEEE 1149.1) host: parameters plus TMS in C ----------------------
 * TCK is the internal bit clock in SPI mode 0: TDI changes while TCK is low,
 * the TAPs sample TMS and TDI on the rising edge and change TDO on the falling
 * edge, and RX samples TDO in step with the bit. TDI = TX, TDO = RX, LSB first.
 * TMS is a GPIO pin that the code changes only while the clock is stopped
 * (after wire_flush), so it always meets setup and hold. Moves through the TAP
 * state machine are 1-bit pushes; shifts are one push of up to 32 bits, with
 * the last bit pushed on its own so TMS can go high with it.
 */

#define WIRE_JTAG(tck_pin, tdi_pin, tdo_pin, hz) ((wire_cfg_t){ \
    WIRE_EN | WIRE_TX | WIRE_RX | WIRE_CLK_OUT, PG_BIT_TICKS(hz), WIRE_PINS(tdi_pin, tdo_pin, tck_pin), 0u })

/* TMS sequences (LSB first) */
#define JTAG_RESET     0x1Fu, 6   /* any state -> Test-Logic-Reset -> Run-Test/Idle */
#define JTAG_TO_SHIFT_DR 0x1u, 3  /* Run-Test/Idle -> Shift-DR */
#define JTAG_TO_SHIFT_IR 0x3u, 4  /* Run-Test/Idle -> Shift-IR */
#define JTAG_TO_IDLE   0x1u, 2    /* Exit1-xR -> Update-xR -> Run-Test/Idle */

/* Clock n TMS bits (LSB first) with TDI low. */
static inline void jtag_tms(pg_wire_t *w, int tms_pin, uint32_t bits, int n)
{
    for (int i = 0; i < n; i++) {
        wire_flush(w);
        pin_write(tms_pin, (int)(bits >> i) & 1);
        wire_putn(w, 1, 0u);
        (void)wire_get(w);
    }
    wire_flush(w);
}

/* In Shift-xR: shift n (1..32) bits of tdi in, LSB first, and return the n
 * bits that come out of TDO. last = 1: TMS goes high with the last bit, which
 * moves to Exit1-xR. */
static inline uint32_t jtag_shift(pg_wire_t *w, int tms_pin, uint32_t tdi, int n, int last)
{
    uint32_t r = 0;
    int k = last ? n - 1 : n;               /* bits with TMS low */
    wire_flush(w);
    pin_write(tms_pin, 0);
    if (k) wire_putn(w, k, tdi);
    if (last) {
        wire_flush(w);
        pin_write(tms_pin, 1);
        wire_putn(w, 1, tdi >> k);
    }
    if (k) r = wire_get(w);
    if (last) r |= (wire_get(w) & 1u) << k;
    wire_flush(w);
    return r;
}

/* ---- CAN 2.0A (11-bit identifiers): frame layer ----------------------------
 * NRZ with stuffing after five equal bits, CRC-15 and arbitration by
 * read-back are done by the engine; frames are built and parsed in C. TX goes
 * to a transceiver's TXD (push-pull, recessive = 1), RX comes from RXD. Put
 * TXD on a uio pin: it stays high-impedance (recessive through the
 * transceiver's pull-up) until the program runs, while uo pins are 0
 * (dominant) after reset. The
 * engine compares every stuffed bit it sends with RXD and stops driving if a
 * recessive bit reads dominant (lost arbitration). The receiver also sees
 * the chip's own frames, which gives the ACK bit.
 */

/* fix8: the bit period in 1/256 cycles (PG_BIT_FIX8(bitrate)) */
static inline void can_init_fix8(pg_wire_t *w, int txd_pin, int rxd_pin, uint32_t fix8)
{
    w->CTRL   = 0;
    w->DIV    = fix8 >> 8;
    w->CTRL2  = WIRE_FRAC(fix8) | WIRE_RESYNC | WIRE_STUFF(5) | WIRE_STUFF_EQUAL |
                WIRE_RX_PACKET | WIRE_EOF_RUN(7);
    w->PINS   = WIRE_PINS(txd_pin, rxd_pin, 0);
    wire_crc(w, 0x4599u, 15, 0, 0, 0);
    w->TXATTR = 0;
    w->CTRL   = WIRE_EN | WIRE_TX | WIRE_RX | WIRE_MSB_FIRST | WIRE_IDLE_HIGH | WIRE_RX_ASYNC |
                WIRE_ARB_STOP | WIRE_EVENTS | WIRE_LEN(32);
}

static inline void can_init(pg_wire_t *w, int txd_pin, int rxd_pin, uint32_t bitrate)
{
    can_init_fix8(w, txd_pin, rxd_pin, PG_BIT_FIX8(bitrate));
}

/* Queue a data frame (0..8 bytes). The engine sends it at once; it needs
 * at most 4 FIFO words. */
static inline void can_send(pg_wire_t *w, uint32_t id, const uint8_t *d, uint32_t n)
{
    w->TXATTR = WIRE_A_CRC_INIT | (n ? 0u : WIRE_A_CRC_OUT);
    wire_putn(w, 19, ((id & 0x7FFu) << 7) | (n & 15u));     /* SOF, ID, RTR, IDE, r0, DLC */
    for (uint32_t i = 0; i < n; ) {
        uint32_t k = n - i < 4u ? n - i : 4u, x = 0;
        for (uint32_t j = 0; j < k; j++) x = (x << 8) | d[i + j];
        i += k;
        w->TXATTR = (i == n) ? WIRE_A_CRC_OUT : 0u;
        wire_putn(w, (int)(8 * k), x);
    }
    w->TXATTR = WIRE_A_NOSTUFF | WIRE_A_NOCRC;
    wire_putn(w, 13, 0x1FFFu);                     /* CRC delim, ACK slot, ACK delim, EOF, intermission */
}

typedef struct {
    uint32_t id;
    uint8_t  dlc;
    uint8_t  data[8];
    uint8_t  crc_ok;
    uint8_t  acked;
} can_frame_t;

/* Parse a received frame: words[] holds the first destuffed data words (MSB first,
 * 32 bits each, at most 4 kept), nw how many data words arrived in all, end_marker
 * the RX FIFO end marker. Returns 0, or -1 if the frame was too short. Use it to
 * receive without blocking: collect words while wire_rx_count() > 0, then parse. */
static inline int can_parse(const uint32_t *words, uint32_t nw, uint32_t end_marker, can_frame_t *f)
{
    /* Bit i of the frame (SOF = bit 0) is bit 31 - (i & 31) of words[i >> 5]. Fields are
     * pulled out with shifts, and the CRC runs over a shifted copy of each word, so a
     * frame takes well under 1000 cycles (cooperative tasks rely on short steps). */
    uint32_t nbits = (nw > 0 ? nw - (WIRE_END_BITS(end_marker) ? 1u : 0u) : 0u) * 32u +
                     WIRE_END_BITS(end_marker);
    if (nbits < 19u) return -1;
    uint32_t dlc = (words[0] >> 13) & 15u;          /* bits 15..18 */
    if (dlc > 8) dlc = 8;
    uint32_t end = 19u + 8u * dlc;                 /* CRC field starts here */
    if (nbits < end + 15u + 2u) return -1;
    uint32_t crc = 0;                              /* CRC-15 over SOF..data, then the CRC: residual 0 */
    for (uint32_t k = 0, n = end + 15u; n; k++) {
        uint32_t x = words[k], m = n < 32u ? n : 32u;
        n -= m;
        while (m--) {
            uint32_t fb = ((x >> 31) ^ (crc >> 14)) & 1u;
            x <<= 1;
            crc = (crc << 1) & 0x7FFFu;
            if (fb) crc ^= 0x4599u;
        }
    }
    f->id = (words[0] >> 20) & 0x7FFu;             /* bits 1..11 */
    f->dlc = (uint8_t)dlc;
    for (uint32_t b = 0; b < dlc; b++) {
        uint32_t pos = 19u + 8u * b, sh = pos & 31u, v = words[pos >> 5] << sh;
        if (sh > 24u) v |= words[(pos >> 5) + 1u] >> (32u - sh);
        f->data[b] = (uint8_t)(v >> 24);
    }
    f->crc_ok = (crc == 0);
    f->acked  = (uint8_t)!((words[(end + 16u) >> 5] >> (31u - ((end + 16u) & 31u))) & 1u);  /* ACK slot */
    return 0;
}

/* Receive one frame (blocks). Returns 0, or -1 if the frame was too short. */
static inline int can_recv(pg_wire_t *w, can_frame_t *f)
{
    uint32_t words[4], nw = 0, w32;
    for (;;) {                                      /* destuffed bits, MSB first, 32 per word */
        w32 = wire_get(w);
        if (WIRE_IS_END(w32)) break;
        if (nw < 4) words[nw] = w32;
        nw++;
    }
    return can_parse(words, nw, w32, f);
}

#endif /* POLYGLOT_PROTOCOL_H */
