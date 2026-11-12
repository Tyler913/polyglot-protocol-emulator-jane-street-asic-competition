/*
 * Copyright (c) 2026 Tyler Hong
 * SPDX-License-Identifier: Apache-2.0
 *
 * protocol.h: the whole programming interface of the Polyglot chip.
 *
 * Version 1 (W2): cycle counter, GPIO, timed GPIO (cycle-exact edges), edge
 * capture, blocking-access timeout, and the Wire Engines. UART and SPI are
 * not hardware blocks: they are a handful of generic engine parameters,
 * defined at the end of this file.
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
#define WIRE_LEN(n)      ((((uint32_t)(n) - 1u) & 31u) << 16)

#define WIRE_PINS(tx, rx, clk)  ((uint32_t)(tx) | ((uint32_t)(rx) << 8) | ((uint32_t)(clk) << 16))

/* STATUS fields */
#define WIRE_TX_LEVEL(s)  ((s) & 0xFu)
#define WIRE_RX_LEVEL(s)  (((s) >> 8) & 0xFu)
#define WIRE_TX_ACTIVE    (1u << 16)
#define WIRE_RX_BUSY      (1u << 17)
#define WIRE_RX_OVERFLOW  (1u << 24)

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

#define WIRE_SPI_CTRL(sck_pin, mosi_pin, miso_pin, mode, hz) ((wire_cfg_t){       \
    WIRE_EN | WIRE_TX | WIRE_RX | WIRE_MSB_FIRST | WIRE_CLK_OUT | WIRE_SPI_MODE(mode) | WIRE_LEN(8), \
    PG_BIT_TICKS(hz), WIRE_PINS(mosi_pin, miso_pin, sck_pin) })

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

/* ---- SPI target (external clock), basic --------------------------------------
 * The controller's SCK clocks the engine. No chip-select framing yet (W3):
 * enable the engine before the first SCK edge. TX words must be queued before
 * the controller clocks them out.
 */

#define WIRE_SPI_TARGET(sck_pin, mosi_pin, miso_pin, mode) ((wire_cfg_t){          \
    WIRE_EN | WIRE_TX | WIRE_RX | WIRE_MSB_FIRST | WIRE_CLK_EXT | WIRE_SPI_MODE(mode) | WIRE_LEN(8), \
    2u, WIRE_PINS(miso_pin, mosi_pin, sck_pin) })

#endif /* POLYGLOT_PROTOCOL_H */
