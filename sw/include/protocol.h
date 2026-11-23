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
#define PG_SHARED   ((volatile uint32_t *)0x00002F00u)

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
#define EV_DMA(k)   (1u << (9 + (k)))   /* DMA channel k (IO -> SRAM) has words */

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
    volatile uint32_t ACKBIT;   /* auto-reply at RX bit N if the RX CRC checks (full engines) */
    volatile uint32_t DIV2;     /* second bit period for FAST words (full engines) */
    volatile uint32_t PHASE;    /* sample points at DIV and DIV2 (0 = mid-bit) */
    volatile uint32_t FDCTL;    /* CAN FD RX unit (W0) */
    uint32_t _pad[13];
    volatile uint32_t TXN[32];  /* TXN[n-1] = x pushes an n-bit word */
} pg_wire_t;

#define W0 ((pg_wire_t *)0x80000100u)
#define W1 ((pg_wire_t *)0x80000200u)
/* W2 and W3 are lite engines: no line codes, bit stuffing, differential output,
 * CRC, TX attributes or auto-reply (USB, 10BASE-T and CAN need W0 or W1). */
#define W2 ((pg_wire_t *)0x80000300u)
#define W3 ((pg_wire_t *)0x80000400u)

/* ---- DMA: SRAM ring buffers <-> IO registers ---------------------------------
 * Two channels. Each moves 32-bit words between a ring in SRAM and one IO
 * register: a Wire Engine's RX or TX register, GPIO_IN, GPIO_OUT, ... A word
 * moves when its pace allows: DMA_PACE_RX (the engine has received data),
 * DMA_PACE_TX (its TX FIFO has room), DMA_PACE_TIMER (every PERIOD cycles,
 * exactly), DMA_PACE_FREE. The ring is [START, END); the DMA owns HEAD, C owns
 * TAIL. IO -> SRAM: words from TAIL to HEAD are C's to read; the DMA stops one
 * word before TAIL. SRAM -> IO: C writes words from TAIL on and advances TAIL;
 * the DMA sends while HEAD != TAIL. A word the DMA moves takes one cycle of the
 * IO bus (the CPU's access waits) and one SRAM cycle. Event EV_DMA(k).
 */
typedef struct {
    volatile uint32_t CTRL;
    volatile uint32_t REG;      /* IO register offset (pointer & 0xFFF) */
    volatile uint32_t START;    /* ring, byte addresses in SRAM */
    volatile uint32_t END;
    volatile uint32_t HEAD;
    volatile uint32_t TAIL;
    volatile uint32_t PERIOD;   /* timer pace, cycles */
    volatile uint32_t STATUS;
} pg_dma_t;

#define DMA0 ((pg_dma_t *)0x80000500u)
#define DMA1 ((pg_dma_t *)0x80000520u)
#define DMA_EN          1u
#define DMA_TO_IO       2u
#define DMA_PACE_TIMER  (0u << 2)
#define DMA_PACE_RX     (1u << 2)
#define DMA_PACE_TX     (2u << 2)
#define DMA_PACE_FREE   (3u << 2)
#define DMA_ENGINE(n)   (((uint32_t)(n) & 3u) << 4)
#define DMA_STREAM      (1u << 6)  /* timer pace: a full IO->SRAM / empty SRAM->IO ring is a miss */
#define DMA_MISSED      1u      /* STATUS: a timer tick found the channel not ready */
#define DMA_BUSY        2u
#define DMA_EMPTY       4u
#define DMA_FULL        8u
#define DMA_REG_OF(p)   ((uint32_t)(uintptr_t)(p) & 0xFFFu)
#define DMA_ENGINE_OF(w) ((((uint32_t)(uintptr_t)(w) >> 8) - 1u) & 3u)

/* Start a channel: ring = buf[0..n), register reg, ctrl = DMA_TO_IO? | pace | engine */
static inline void dma_start(pg_dma_t *d, uint32_t ctrl, volatile uint32_t *reg, uint32_t *buf, uint32_t n,
                             uint32_t period)
{
    uint32_t a = (uint32_t)(uintptr_t)buf;
    d->CTRL   = 0;
    d->REG    = DMA_REG_OF(reg);
    d->START  = a;
    d->END    = a + 4u * n;
    d->HEAD   = a;
    d->TAIL   = a;
    d->PERIOD = period;
    d->STATUS = DMA_MISSED;
    d->CTRL   = ctrl | DMA_EN;
}

static inline void dma_stop(pg_dma_t *d) { d->CTRL = 0; }

/* Receive every word of engine w into the ring buf[0..n) */
static inline void dma_rx(pg_dma_t *d, pg_wire_t *w, uint32_t *buf, uint32_t n)
{
    dma_start(d, DMA_PACE_RX | DMA_ENGINE(DMA_ENGINE_OF(w)), &w->RX, buf, n, 1);
}

/* Words waiting in an IO -> SRAM ring */
static inline uint32_t dma_count(pg_dma_t *d)
{
    uint32_t h = d->HEAD, t = d->TAIL;
    return (h >= t ? h - t : h + (d->END - d->START) - t) >> 2;
}

/* Next word of an IO -> SRAM ring (call only if dma_count() > 0) */
static inline uint32_t dma_get(pg_dma_t *d)
{
    uint32_t t = d->TAIL, x = *(volatile uint32_t *)(uintptr_t)t;
    t += 4u;
    if (t == d->END) t = d->START;
    d->TAIL = t;
    return x;
}

/* Send words through engine w's TX register (CTRL.LEN bits each) from the ring buf[0..n) */
static inline void dma_tx(pg_dma_t *d, pg_wire_t *w, uint32_t *buf, uint32_t n)
{
    dma_start(d, DMA_TO_IO | DMA_PACE_TX | DMA_ENGINE(DMA_ENGINE_OF(w)), &w->TX, buf, n, 1);
}

/* Room in an SRAM -> IO ring */
static inline uint32_t dma_room(pg_dma_t *d)
{
    uint32_t h = d->HEAD, t = d->TAIL;
    return ((h > t ? h - t : h + (d->END - d->START) - t) >> 2) - 1u;
}

/* Append a word to an SRAM -> IO ring (call only if dma_room() > 0) */
static inline void dma_put(pg_dma_t *d, uint32_t x)
{
    uint32_t t = d->TAIL;
    *(volatile uint32_t *)(uintptr_t)t = x;
    t += 4u;
    if (t == d->END) t = d->START;
    d->TAIL = t;
}

/* Logic analyzer: sample GPIO_IN every period cycles into buf[0..n), n - 1 samples, then stop */
static inline void dma_capture(pg_dma_t *d, uint32_t *buf, uint32_t n, uint32_t period)
{
    dma_start(d, DMA_PACE_TIMER, &PG_GPIO_IN, buf, n, period);
}

/* The same as a continuous stream into the ring buf[0..n): C reads it (dma_get) while it
 * fills; a sample that finds the ring full is lost and sets DMA_MISSED. */
static inline void dma_capture_stream(pg_dma_t *d, uint32_t *buf, uint32_t n, uint32_t period)
{
    dma_start(d, DMA_PACE_TIMER | DMA_STREAM, &PG_GPIO_IN, buf, n, period);
}

/* Pattern generator: write buf[0..count) to GPIO_OUT, one word every period cycles */
static inline void dma_pattern(pg_dma_t *d, uint32_t *buf, uint32_t n, uint32_t count, uint32_t period)
{
    dma_start(d, DMA_TO_IO | DMA_PACE_TIMER, &PG_GPIO_OUT, buf, n, period);
    d->TAIL = (uint32_t)(uintptr_t)(buf + count);    /* count < n words queued */
}


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
#define WIRE_MANCHESTER  (2u << 9)                 /* line code: ~bit then bit in each bit time (IEEE 802.3);
                                                      RX (async, engine 1) decodes by edge timing: clock recovery */
#define WIRE_STUFF(n)    (((uint32_t)(n) & 7u) << 11)  /* insert/remove a bit after n equal bits */
#define WIRE_STUFF_EQUAL (1u << 14)                /* runs of equal bits (CAN); default: runs of ones (USB) */
#define WIRE_DIFF        (1u << 15)                /* CLK pin = complement of TX; SE0 = RX and GATE pins low */
#define WIRE_RX_PACKET   (1u << 16)                /* async RX continues word after word until SE0 */
#define WIRE_TAIL(n)     (((uint32_t)(n) & 3u) << 17)  /* drive n idle bits after the last word */
#define WIRE_EOF_RUN(n)  (((uint32_t)(n) & 7u) << 19)  /* RX_PACKET also ends after n idle bits (CAN) */
#define WIRE_MSTART      (1u << 22)                /* Manchester RX: the first edge starts a bit (not mid-bit) */
#define WIRE_BIPHASE     (3u << 9)                 /* line code: biphase mark (S/PDIF, AES3, USB PD, LTC);
                                                      RX (async, engine 1) decodes by edge timing */
#define WIRE_BIPHASE_DM  (1u << 23)                /* with WIRE_BIPHASE: differential Manchester */
#define WIRE_RSW_SP      (1u << 24)                /* FAST rate changes at the sample point of the last bit (CAN FD) */

/* TXATTR bits (apply to the words pushed after the write) */
#define WIRE_A_NOSTUFF   (1u << 0)
#define WIRE_A_NOCRC     (1u << 1)
#define WIRE_A_CRC_INIT  (1u << 2)
#define WIRE_A_CRC_OUT   (1u << 3)                 /* append the CRC after this word */
#define WIRE_A_SE0       (1u << 4)                 /* both lines low for this word */
#define WIRE_A_AFTER_RX  (1u << 5)                 /* start TURN cycles after the next received packet */
#define WIRE_A_FAST      (1u << 6)                 /* bit period DIV2 */
#define WIRE_A_PP        (1u << 7)                 /* open-drain mode: drive this word push-pull (I3C) */
#define WIRE_A_RAW       (1u << 8)                 /* no line code: NRZ levels */

/* DIV2: second bit period (DIV2 >> 8 cycles + fraction) from a PG_BIT_FIX8 value */
#define WIRE_DIV2_FIX8(x) ((((uint32_t)(x) >> 8) & 0xFFFFu) | (((uint32_t)(x) & 0xFFu) << 16))
/* PHASE: sample point as cycles before the end of the bit, at DIV and at DIV2 */
#define WIRE_PHASE(n, f)  (((uint32_t)(n) & 0xFFFFu) | ((uint32_t)(f) << 16))

/* RXMATCH: only packets whose word k satisfies (word & mask) == value trigger AFTER_RX words */
#define WIRE_MATCH(k, value, mask) ((((uint32_t)(k) & 15u) << 16) | (((uint32_t)(mask) & 0xFFu) << 8) | ((uint32_t)(value) & 0xFFu))

/* ACKBIT: AFTER_RX words start TURN cycles after RX decodes bit n of the packet
 * (1 = its first bit, stuff bits not counted), if the RX CRC over bits 1..n-1 is zero.
 * WIRE_ACK_FD (W0 with FDCTL on): at the CRC delimiter of a CAN FD frame that checks. */
#define WIRE_ACK_AT(n)   (((uint32_t)(n) & 0x3FFu) | (1u << 16))
#define WIRE_ACK_FD      (1u << 16)

/* CRC unit: width 1..32, polynomial without the top bit; TX appends it MSB first */
static inline void wire_crc(pg_wire_t *w, uint32_t poly, int width, uint32_t init, int xorout, int rx_skip)
{
    w->CRCPOLY = poly;
    w->CRCINIT = init;
    w->CRCCFG  = ((uint32_t)(width - 1) & 31u) | (xorout ? 1u << 5 : 0u) | (((uint32_t)rx_skip & 31u) << 8);
}

/* SYNC hunt (engine 1, async RX): deliver only the bits after the 8-bit
 * pattern (in bit order: LSB first = first received bit in bit 0). Call after wire_crc. */
static inline void wire_sync(pg_wire_t *w, uint8_t pattern)
{
    w->CRCCFG = (w->CRCCFG & 0xFFFFu) | ((uint32_t)pattern << 16) | (1u << 24);
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
#define WIRE_IS_END(w)   (((w) & 0xFFE0E0FFu) == WIRE_EV_END)   /* bits 20..16: CAN FD unit status */
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

/* ---- Asynchronous serial with any frame format ------------------------------
 * The engine shifts whole frames, so the data bits (5..9), parity and stop
 * bits are all chosen in C. An inverted line (idle low, e.g. SBUS, SDI-12)
 * just clears WIRE_IDLE_HIGH: RX then starts on a rising edge and words
 * arrive inverted, which ser_decode undoes; ser_encode inverts TX words.
 */
#define SER_NONE 0u
#define SER_EVEN 1u
#define SER_ODD  2u

typedef struct { uint8_t data, parity, stop, invert; } ser_fmt_t;

#define SER_FMT(data, parity, stop, invert) ((ser_fmt_t){ (data), (parity), (stop), (invert) })

static inline uint32_t ser_bits(ser_fmt_t f) { return 1u + f.data + (f.parity ? 1u : 0u) + f.stop; }

static inline uint32_t ser_par(uint32_t x)        /* 1 if x has an odd number of ones */
{
    x ^= x >> 16; x ^= x >> 8; x ^= x >> 4; x ^= x >> 2; x ^= x >> 1;
    return x & 1u;
}

/* Frame word, start bit first (LSB) */
static inline uint32_t ser_encode(ser_fmt_t f, uint32_t v)
{
    uint32_t n = f.data;
    v &= (1u << n) - 1u;
    uint32_t w = v << 1;
    if (f.parity) w |= (ser_par(v) ^ (f.parity == SER_ODD)) << (++n);
    w |= ((1u << f.stop) - 1u) << (n + 1u);
    return f.invert ? w ^ ((1u << ser_bits(f)) - 1u) : w;
}

/* Data of a received frame word, or -1 (start, parity or stop bit wrong;
 * a BREAK, all space, also gives -1: see ser_is_break). */
static inline int ser_decode(ser_fmt_t f, uint32_t w)
{
    uint32_t nb = ser_bits(f), p = 1u + f.data;
    if (f.invert) w ^= (1u << nb) - 1u;
    uint32_t v = (w >> 1) & ((1u << f.data) - 1u);
    if (w & 1u) return -1;
    if (f.parity) {
        if (((w >> p) & 1u) != (ser_par(v) ^ (f.parity == SER_ODD))) return -1;
        p++;
    }
    if (((w >> p) & ((1u << f.stop) - 1u)) != (1u << f.stop) - 1u) return -1;
    return (int)v;
}

/* A frame word that is all space (logic 0), stop bits included: a BREAK */
static inline int ser_is_break(ser_fmt_t f, uint32_t w)
{
    return ((f.invert ? ~w : w) & ((1u << ser_bits(f)) - 1u)) == 0u;
}

static inline void ser_init(pg_wire_t *w, int tx_pin, int rx_pin, uint32_t baud, ser_fmt_t f)
{
    uint32_t fix8 = PG_BIT_FIX8(baud);
    wire_config(w, (wire_cfg_t){ WIRE_EN | WIRE_TX | WIRE_RX | WIRE_RX_ASYNC | WIRE_LEN(ser_bits(f)) |
                                 (f.invert ? 0u : WIRE_IDLE_HIGH),
                                 fix8 >> 8, WIRE_PINS(tx_pin, rx_pin, 0), WIRE_FRAC(fix8) | WIRE_RESYNC });
}

static inline void ser_putc(pg_wire_t *w, ser_fmt_t f, uint32_t v) { wire_putn(w, (int)ser_bits(f), ser_encode(f, v)); }

static inline int ser_getc(pg_wire_t *w, ser_fmt_t f) { return ser_decode(f, w->RX); }

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

/* ---- USB low speed (1.5 Mbit/s) and full speed (12 Mbit/s): line layer ----------
 * NRZI, stuffing after six ones, CRC16 and the SE0 end of packet are done by
 * the engine; packets, PIDs and the device state machine are C. The line of
 * the J state is the engine's TX/RX pin, the other line its CLK (complement)
 * and GATE (SE0 detect) pin: low speed idles with D- high (pull-up on D-),
 * full speed with D+ high (pull-up on D+). The two speeds differ only in
 * these pins and the bit period. The handshake for a received packet can be
 * queued in advance and goes out by itself after USB_TURN(rate) cycles,
 * well inside the 7.5 bit times the spec allows. USB runs on W1 (it needs
 * the SYNC hunt, which only engine 1 has). RX: the engine's SYNC hunt
 * consumes the SYNC byte and packets arrive as 16-bit words, PID first (at
 * full speed a byte every 32 cycles is too fast to take one by one), see
 * usb_rx_word(). Full speed needs a clock
 * that is a multiple of 12 MHz (48 or 60 MHz: 4 or 5 cycles per bit).
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
#define USB_TURN(rate) (PG_BIT_FIX8(rate) * 9u / 512u)  /* 4.5 bit times after SE0 is seen */
#endif

/* j_pin: the line that is high in the J (idle) state, k_pin: the other one */
static inline void usb_line_init(pg_wire_t *w, int j_pin, int k_pin, uint32_t fix8, uint32_t turn)
{
    w->CTRL    = 0;
    w->DIV     = fix8 >> 8;
    w->CTRL2   = WIRE_FRAC(fix8) | WIRE_RESYNC | WIRE_NRZI | WIRE_STUFF(6) |
                 WIRE_DIFF | WIRE_RX_PACKET | WIRE_TAIL(1);      /* EOP: SE0 SE0, then J driven by TAIL */
    w->PINS    = WIRE_PINS(j_pin, j_pin, k_pin) | WIRE_GATE_PIN(k_pin);
    wire_crc(w, 0x8005u, 16, 0xFFFFu, 1, 8);        /* CRC16; RX skips the PID */
    wire_sync(w, 0x80u);                             /* RX starts after SYNC */
    w->TURN    = turn;
    w->TXATTR  = 0;
    w->CTRL    = WIRE_EN | WIRE_TX | WIRE_RX | WIRE_IDLE_HIGH | WIRE_RX_ASYNC | WIRE_OE_FRAME |
                 WIRE_EVENTS | WIRE_LEN(16);
}

/* Add one RX word to a packet buffer. Returns 1 when the packet ended
 * (*n = its length in bytes, PID first), 0 otherwise. */
static inline int usb_rx_word(uint32_t x, uint8_t *pkt, uint32_t max, uint32_t *n)
{
    if (WIRE_IS_END(x)) {
        if (WIRE_END_BITS(x) == 8u) *n -= 1u;       /* odd length: the last word held one byte */
        return 1;
    }
    if (*n + 2u <= max) { pkt[*n] = (uint8_t)x; pkt[*n + 1u] = (uint8_t)(x >> 8); }
    *n += 2u;
    return 0;
}

static inline void usb_ls_init(pg_wire_t *w, int dm_pin, int dp_pin)
{
    usb_line_init(w, dm_pin, dp_pin, PG_BIT_FIX8(1500000), USB_TURN(1500000));
}

static inline void usb_fs_init(pg_wire_t *w, int dm_pin, int dp_pin)
{
    usb_line_init(w, dp_pin, dm_pin, PG_BIT_FIX8(12000000), USB_TURN(12000000));
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
#define USB_MATCH_DATA  WIRE_MATCH(0, 0x03, 0x03)   /* DATA0/1/2/MDATA: PID bits 1:0 = 11 */
#define USB_MATCH_IN    WIRE_MATCH(0, USB_PID_IN, 0xFF)

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

/* Normal link pulse (NLP): TD+ high for one bit time (100 ns), as a 1-bit NRZ
 * word on the differential pair; send one every 16 ms (+/- 8 ms) while idle,
 * or a link partner reports no link. Call between frames. */
static inline void eth_link_pulse(pg_wire_t *w)
{
    wire_flush(w);
    uint32_t c2 = w->CTRL2;
    w->CTRL2 = c2 & ~WIRE_MANCHESTER & ~WIRE_TAIL(3);
    w->TXATTR = WIRE_A_NOCRC | WIRE_A_NOSTUFF;
    wire_putn(w, 1, 1u);
    wire_flush(w);
    w->CTRL2 = c2;
    w->TXATTR = 0;
}

/* 10BASE-T receive: RD (one comparator output, idle low) on engine 1 (W1).
 * The engine decodes Manchester by edge timing, hunts for the SFD (0xD5),
 * delivers the frame as 16-bit words and runs the CRC-32 over it, FCS
 * included; a good frame leaves the residual ETH_CRC_RESIDUAL. Independent
 * jitter on every edge up to +/-5 ns (50 MHz) or +/-8 ns (60 MHz) by design
 * (RTL sim); set DIV = clock / 10 MHz, an integer. */
#define ETH_CRC_RESIDUAL 0xC704DD7Bu

static inline void eth_rx_init(pg_wire_t *w, int rd_pin)
{
    w->CTRL  = 0;
    w->DIV   = PG_F_CPU / 10000000u;
    w->CTRL2 = WIRE_MANCHESTER | WIRE_RX_PACKET;
    w->PINS  = WIRE_PINS(0, rd_pin, 0);
    wire_crc(w, 0x04C11DB7u, 32, 0xFFFFFFFFu, 0, 0);
    wire_sync(w, 0xD5u);
    w->CTRL  = WIRE_EN | WIRE_RX | WIRE_RX_ASYNC | WIRE_EVENTS | WIRE_LEN(16);
}

/* Receive one frame into f (at most max bytes, FCS included; 16-bit aligned).
 * Returns its length, or -1 if the FCS is wrong, -2 if it did not fit or had
 * a partial byte. Blocks until a frame ends. */
static inline int eth_recv(pg_wire_t *w, uint8_t *f, uint32_t max)
{
    uint16_t *h = (uint16_t *)f;
    uint32_t n = 0, x;
    int bad = 0;
    while (!WIRE_IS_END(x = w->RX)) {
        if (n + 2u <= max) h[n >> 1] = (uint16_t)x; else bad = 1;
        n += 2u;
    }
    uint32_t crc = w->RXCRC, part = WIRE_END_BITS(x);
    if (part) n -= (part == 8u) ? 1u : 2u;
    if (bad || (part & 7u)) return -2;
    return crc == ETH_CRC_RESIDUAL ? (int)n : -1;
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

/* ---- CAN 2.0A/2.0B: frame layer --------------------------------------------
 * NRZ with stuffing after five equal bits, CRC-15 and arbitration by
 * read-back are done by the engine; frames are built and parsed in C. TX goes
 * to a transceiver's TXD (push-pull, recessive = 1), RX comes from RXD. Put
 * TXD on a uio pin: it stays high-impedance (recessive through the
 * transceiver's pull-up) until the program runs, while uo pins are 0
 * (dominant) after reset. The
 * engine compares every stuffed bit it sends with RXD and stops driving if a
 * recessive bit reads dominant (lost arbitration). The receiver also sees
 * the chip's own frames, which gives the ACK bit.
 * Identifiers with CAN_EFF set are 29-bit extended identifiers (CAN 2.0B, used
 * by SAE J1939); can_send and can_parse handle both formats. To receive
 * extended frames with ACK, use can_init_b / can_recv_b (20-bit RX words).
 * Define CAN_STD_ONLY before including this header to leave out the
 * extended-frame parsing where memory is tight.
 */

#define CAN_EFF 0x80000000u    /* extended (29-bit) identifier flag, as in Linux SocketCAN */

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
 * at most 4 FIFO words (5 for an extended frame with more than 4 bytes: the
 * last write waits for room). */
static inline void can_send(pg_wire_t *w, uint32_t id, const uint8_t *d, uint32_t n)
{
#ifndef CAN_STD_ONLY
    if (id & CAN_EFF) {
        w->TXATTR = WIRE_A_CRC_INIT;                /* SOF, base ID, SRR, IDE, extended ID */
        wire_putn(w, 32, (((id >> 18) & 0x7FFu) << 20) | (3u << 18) | (id & 0x3FFFFu));
        w->TXATTR = n ? 0u : WIRE_A_CRC_OUT;
        wire_putn(w, 7, n & 15u);                   /* RTR, r1, r0, DLC */
    } else
#endif
    {
        w->TXATTR = WIRE_A_CRC_INIT | (n ? 0u : WIRE_A_CRC_OUT);
        wire_putn(w, 19, ((id & 0x7FFu) << 7) | (n & 15u)); /* SOF, ID, RTR, IDE, r0, DLC */
    }
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
static inline int can_parse_bits(const uint32_t *words, uint32_t nbits, can_frame_t *f);

static inline int can_parse(const uint32_t *words, uint32_t nw, uint32_t end_marker, can_frame_t *f)
{
    /* Bit i of the frame (SOF = bit 0) is bit 31 - (i & 31) of words[i >> 5]. Fields are
     * pulled out with shifts, and the CRC runs over a shifted copy of each word, so a
     * frame takes well under 1000 cycles (cooperative tasks rely on short steps). */
    uint32_t nbits = (nw > 0 ? nw - (WIRE_END_BITS(end_marker) ? 1u : 0u) : 0u) * 32u +
                     WIRE_END_BITS(end_marker);
    return can_parse_bits(words, nbits, f);
}

/* The same on nbits frame bits packed MSB first into 32-bit words. */
static inline int can_parse_bits(const uint32_t *words, uint32_t nbits, can_frame_t *f)
{
    if (nbits < 19u) return -1;
#ifdef CAN_STD_ONLY                                /* 11-bit identifiers only: smaller code */
    uint32_t ide = 0;
#else
    uint32_t ide = (words[0] >> 18) & 1u;          /* bit 13 */
#endif
    uint32_t hdr = ide ? 39u : 19u;                /* SOF .. DLC */
    if (nbits < hdr) return -1;
    uint32_t dlc = (ide ? words[1] >> 25 : words[0] >> 13) & 15u;   /* bits hdr-4 .. hdr-1 */
    if (dlc > 8) dlc = 8;
    uint32_t end = hdr + 8u * dlc;                 /* CRC field starts here */
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
    if (ide) f->id = (f->id << 18) | (words[0] & 0x3FFFFu) | CAN_EFF;   /* + bits 14..31 */
    f->dlc = (uint8_t)dlc;
    for (uint32_t b = 0; b < dlc; b++) {
        uint32_t pos = hdr + 8u * b, sh = pos & 31u, v = words[pos >> 5] << sh;
        if (sh > 24u) v |= words[(pos >> 5) + 1u] >> (32u - sh);
        f->data[b] = (uint8_t)(v >> 24);
    }
    f->crc_ok = (crc == 0);
    f->acked  = (uint8_t)!((words[(end + 16u) >> 5] >> (31u - ((end + 16u) & 31u))) & 1u);  /* ACK slot */
    return 0;
}

/* Receiver ACK, armed from the first 32 bits of a frame (they hold the DLC):
 * a dominant bit queued as an AFTER_RX word goes out at the ACK slot, half a
 * bit after the CRC delimiter is sampled, only if the CRC checks (ACKBIT).
 * Not for our own frame; after a lost arbitration the rest of ours is dropped. */
static inline void can_ack_arm_at(pg_wire_t *w, uint32_t dlc, uint32_t hdr)
{
    uint32_t st = w->STATUS;
    if (dlc > 8u) dlc = 8u;
    if (st & WIRE_TX_ACTIVE) {
        if (!(st & WIRE_ARB_LOST)) return;        /* our own frame */
        wire_tx_discard(w);
    }
    w->TURN   = (w->DIV >> 1) - 4u;               /* delimiter sample -> ACK slot start */
    w->TXATTR = WIRE_A_AFTER_RX | WIRE_A_NOSTUFF | WIRE_A_NOCRC;
    wire_putn(w, 1, 0u);
    w->TXATTR = 0;
    w->ACKBIT = WIRE_ACK_AT(hdr + 16u + 8u * dlc); /* SOF..DLC, data, CRC 15, delimiter */
}

static inline void can_ack_arm(pg_wire_t *w, uint32_t word0)
{
    can_ack_arm_at(w, (word0 >> 13) & 15u, 19u);
}

/* Receive one frame (blocks). ack = 1: acknowledge frames from other nodes
 * (full engines). Returns 0, or -1 if the frame was too short. */
static inline int can_recv_opt(pg_wire_t *w, can_frame_t *f, int ack)
{
    uint32_t words[4], nw = 0, w32;
    for (;;) {                                      /* destuffed bits, MSB first, 32 per word */
        w32 = wire_get(w);
        if (WIRE_IS_END(w32)) break;
        if (ack && nw == 0) can_ack_arm(w, w32);
        if (nw < 4) words[nw] = w32;
        nw++;
    }
    if (ack && WIRE_TX_LEVEL(w->STATUS)) wire_tx_discard(w);   /* CRC bad: the ACK stayed queued */
    return can_parse(words, nw, w32, f);
}

static inline int can_recv_ack(pg_wire_t *w, can_frame_t *f) { return can_recv_opt(w, f, 1); }

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

/* CAN 2.0B receiver: standard and extended frames. RX words are 20 bits long,
 * so the DLC is complete in word 0 (standard, bits 15..18) or word 1
 * (extended, bits 35..38) well before the ACK slot, even for the shortest
 * frames; can_recv_b packs the words into 32-bit ones for can_parse_bits.
 * can_send works unchanged. */
static inline void can_init_b(pg_wire_t *w, int txd_pin, int rxd_pin, uint32_t bitrate)
{
    can_init_fix8(w, txd_pin, rxd_pin, PG_BIT_FIX8(bitrate));
    w->CTRL = (w->CTRL & ~WIRE_LEN(32)) | WIRE_LEN(20);
}

/* Receive one frame (blocks). ack = 1: acknowledge frames from other nodes
 * (full engines). Returns 0, or -1 if the frame was too short. */
static inline int can_recv_b(pg_wire_t *w, can_frame_t *f, int ack)
{
    uint32_t a[4] = {0, 0, 0, 0}, nb = 0, nw = 0, w0 = 0, x;
    for (;;) {
        x = wire_get(w);
        if (WIRE_IS_END(x)) break;
        if (ack) {                                  /* IDE = bit 13 = bit 6 of word 0 */
            if (nw == 0 && !((x >> 6) & 1u)) can_ack_arm_at(w, (x >> 1) & 15u, 19u);
            if (nw == 1 && ((w0 >> 6) & 1u)) can_ack_arm_at(w, (x >> 1) & 15u, 39u);
        }
        if (nw == 0) w0 = x;
        uint32_t k = nb >> 5, s = nb & 31u;         /* append 20 bits MSB first */
        x &= 0xFFFFFu;
        if (k < 4u) {
            if (s <= 12u) a[k] |= x << (12u - s);
            else {
                a[k] |= x >> (s - 12u);
                if (k < 3u) a[k + 1u] |= x << (44u - s);
            }
        }
        nb += 20u;
        nw++;
    }
    if (ack && WIRE_TX_LEVEL(w->STATUS)) wire_tx_discard(w);   /* CRC bad: the ACK stayed queued */
    if (WIRE_END_BITS(x)) nb -= 20u - WIRE_END_BITS(x);       /* partial last word */
    return can_parse_bits(a, nb, f);
}

/* ---- CAN FD (ISO 11898-1:2015) on W0 ------------------------------------------
 * Two bit rates in one frame: TX words with WIRE_A_FAST use DIV2, and with
 * WIRE_RSW_SP the rate changes at the sample point of BRS and of the CRC
 * delimiter. The engine stuffs SOF..data and reads back the arbitration field;
 * C simulates the same stuffing to get the stuff count and the CRC-17/21 (they
 * cover the dynamic stuff bits), and sends the CRC field with its fixed stuff
 * bits as NOSTUFF words. RX: W0's CAN FD unit (FDCTL) follows FDF, BRS and the
 * DLC of every frame, switches the RX bit rate, removes the fixed stuff bits
 * and checks CRC-17/21 and the stuff count; it also follows classic frames
 * (CRC-15) and, like a CAN controller, ACKs every good frame from another node
 * by itself (FDCTL_ACK_EN), so C has no deadline. The frame status comes with
 * the RX end marker (bits 20..16). Classic CAN frames still work on the same
 * bus (can_send; canfd_recv parses both). Sample points: 80 % nominal, 75 %
 * data. RX words are 20 bits long.
 */
#define FDCTL_EN      1u
#define FDCTL_ACK_EN  2u          /* the unit ACKs good frames from other nodes itself */
#define FDCTL_FDF     (1u << 8)
#define FDCTL_BRS     (1u << 9)
#define FDCTL_DLM     (1u << 10)
#define FDCTL_CRC_OK  (1u << 11)
#define FDCTL_SC_OK   (1u << 12)

static const uint8_t canfd_len[16] = {0, 1, 2, 3, 4, 5, 6, 7, 8, 12, 16, 20, 24, 32, 48, 64};

/* DLC for a payload length (rounded up to the next valid length) */
static inline uint32_t canfd_dlc(uint32_t n)
{
    uint32_t d = 0;
    while (d < 15u && canfd_len[d] < n) d++;
    return d;
}

static inline void canfd_init(pg_wire_t *w, int txd_pin, int rxd_pin, uint32_t nominal, uint32_t data)
{
    uint32_t fn = PG_BIT_FIX8(nominal), fd = PG_BIT_FIX8(data);
    can_init_fix8(w, txd_pin, rxd_pin, fn);
    w->DIV2  = WIRE_DIV2_FIX8(fd);
    w->PHASE = WIRE_PHASE((fn >> 8) / 5u, (fd >> 8) / 4u);
    w->CTRL2 = w->CTRL2 | WIRE_RSW_SP;
    w->TURN  = (fn >> 8) / 5u - 4u;                   /* delimiter sample -> ACK slot start */
    w->FDCTL = FDCTL_EN | FDCTL_ACK_EN;
    w->CTRL  = (w->CTRL & ~WIRE_LEN(32)) | WIRE_LEN(20);
}

/* C copy of the engine's stuffing over SOF..data, with CRC-17 and CRC-21 */
typedef struct { uint32_t c17, c21, run, last, dsc, pend; } canfd_tx_t;

static inline void canfd_crc_bit(canfd_tx_t *s, uint32_t b)
{
    uint32_t f17 = (b ^ (s->c17 >> 16)) & 1u, f21 = (b ^ (s->c21 >> 20)) & 1u;
    s->c17 = ((s->c17 << 1) & 0x1FFFFu) ^ (f17 ? 0x1685Bu : 0u);
    s->c21 = ((s->c21 << 1) & 0x1FFFFFu) ^ (f21 ? 0x102899u : 0u);
}

/* Pass 0: account for n bits (MSB first); pass 1: push them as one word.
 * The whole frame is computed before the first word goes out, since the
 * per-bit simulation is slower than a fast data phase. */
static inline void canfd_put(pg_wire_t *w, canfd_tx_t *s, int n, uint32_t x, uint32_t attr, int pass)
{
    if (pass) {
        w->TXATTR = attr;
        wire_putn(w, n, x);
        return;
    }
    for (int i = n - 1; i >= 0; i--) {
        uint32_t b = (x >> i) & 1u;
        if (s->pend) {                                /* the engine's dynamic stuff bit */
            canfd_crc_bit(s, s->last ^ 1u);
            s->dsc++;
            s->last ^= 1u;
            s->run = 1;
            s->pend = 0;
        }
        canfd_crc_bit(s, b);
        s->run = (b == s->last) ? s->run + 1u : 1u;
        s->last = b;
        if (s->run == 5u) s->pend = 1;                /* due before the next bit */
    }
}

/* Queue a frame: n data bytes (0..64, a valid CAN FD length), brs = data phase
 * at the second bit rate. Identifiers with CAN_EFF set are 29-bit. */
static inline void canfd_send(pg_wire_t *w, uint32_t id, const uint8_t *d, uint32_t n, int brs)
{
    canfd_tx_t s;
    s.c17 = 1u << 16;                                 /* ISO: 1 followed by zeros */
    s.c21 = 1u << 20;
    s.run = s.dsc = s.pend = 0;
    s.last = 2;
    uint32_t fast = brs ? WIRE_A_FAST : 0u, b = brs ? 1u : 0u, tw[2], tn[2] = {0, 0};
    for (int pass = 0; pass < 2; pass++) {
        if (id & CAN_EFF) {                           /* SOF, base ID, SRR, IDE, ID extension */
            canfd_put(w, &s, 32, (((id >> 18) & 0x7FFu) << 20) | (3u << 18) | (id & 0x3FFFFu), 0, pass);
            canfd_put(w, &s, 4, 4u | b, 0, pass);    /* RRS, FDF, res, BRS */
        } else {                                      /* SOF, ID, RRS, IDE, FDF, res, BRS */
            canfd_put(w, &s, 17, ((id & 0x7FFu) << 5) | 4u | b, 0, pass);
        }
        canfd_put(w, &s, 5, canfd_dlc(n), fast, pass);   /* ESI, DLC */
        for (uint32_t i = 0; i < n; ) {
            uint32_t k = n - i < 4u ? n - i : 4u, x = 0;
            for (uint32_t j = 0; j < k; j++) x = (x << 8) | d[i + j];
            i += k;
            canfd_put(w, &s, (int)(8 * k), x, fast, pass);
        }
        if (pass) break;
        /* CRC field: stuff count (Gray code + even parity), CRC; a fixed stuff bit
         * before it and after every 4 bits. If the engine stuffs after the last
         * data bit, that stuff bit is the first fixed one (not counted, not in
         * the CRC). At most 33 bits with the delimiter: two words. */
        uint32_t c = s.dsc & 7u, g = c ^ (c >> 1);
        uint32_t sc = (g << 1) | ((g ^ (g >> 1) ^ (g >> 2)) & 1u);
        for (int i = 3; i >= 0; i--) canfd_crc_bit(&s, (sc >> i) & 1u);
        uint32_t wd = n > 16u ? 21u : 17u, crc = wd == 21u ? s.c21 : s.c17;
        uint32_t prev = s.pend ? s.last ^ 1u : s.last, acc = 0, na = 0;
        for (uint32_t i = 0; i < 4u + wd; i++) {
            uint32_t bit = i < 4u ? (sc >> (3u - i)) & 1u : (crc >> (wd + 3u - i)) & 1u;
            if ((i & 3u) == 0 && (i || !s.pend)) {    /* fixed stuff bit */
                prev ^= 1u;
                acc = (acc << 1) | prev;
                na++;
            }
            acc = (acc << 1) | bit;
            prev = bit;
            if (++na >= 31u) {                        /* keep room for one more bit */
                tw[0] = acc;
                tn[0] = na;
                acc = 0;
                na = 0;
            }
        }
        tw[1] = (acc << 1) | 1u;                      /* ... CRC delimiter (rate back at its sample point) */
        tn[1] = na + 1u;
    }
    uint32_t attr = fast | WIRE_A_NOSTUFF | WIRE_A_NOCRC;
    for (int k = 0; k < 2; k++) {
        if (!tn[k]) continue;
        w->TXATTR = attr;
        wire_putn(w, (int)tn[k], tw[k]);
    }
    w->TXATTR = WIRE_A_NOSTUFF | WIRE_A_NOCRC;
    wire_putn(w, 12, 0xFFFu);                         /* ACK slot, ACK delim, EOF, intermission */
    w->TXATTR = 0;
}

typedef struct {
    uint32_t id;
    uint32_t status;     /* FDCTL at the end of the frame (FD frames) */
    uint8_t  len, fd, brs, crc_ok, acked;
    uint8_t  data[64];
} canfd_frame_t;

/* n (1..32) bits from position pos of an MSB-first packed bit buffer */
static inline uint32_t canfd_field(const uint32_t *a, uint32_t pos, uint32_t n)
{
    uint32_t k = pos >> 5, sh = pos & 31u, v = a[k] << sh;
    if (sh + n > 32u) v |= a[k + 1u] >> (32u - sh);
    return v >> (32u - n);
}

/* Receive one frame, classic or FD (blocks; the unit has already ACKed it if
 * good). Returns 0, or -1 if the frame was too short. The 20-bit RX words are
 * packed into 32-bit words as they arrive (no division: RV32EC). */
static inline int canfd__recv(pg_wire_t *w, pg_dma_t *d, canfd_frame_t *f)
{
    uint32_t a[22], nb = 0, nw = 0, x, st;
    for (uint32_t i = 0; i < 22u; i++) a[i] = 0;
    for (;;) {                                        /* destuffed bits, MSB first, 20 per word */
        if (d) {
            while (!dma_count(d)) { }
            x = dma_get(d);
        } else {
            x = wire_get(w);
        }
        if (WIRE_IS_END(x)) break;
        nw++;
        uint32_t k = nb >> 5, s = nb & 31u;           /* append 20 bits MSB first */
        x &= 0xFFFFFu;
        if (k < 21u) {
            if (s <= 12u) a[k] |= x << (12u - s);
            else {
                a[k] |= x >> (s - 12u);
                a[k + 1u] |= x << (44u - s);
            }
            nb += 20u;
        }
    }
    st = (x >> 8) & 0x1F00u;                          /* unit status (FDCTL[12:8] layout) */
    if (WIRE_END_BITS(x) && nb >= 20u) nb -= 20u - WIRE_END_BITS(x);
    if (nb < 20u) return -1;
    uint32_t ide = canfd_field(a, 13, 1), fdf = canfd_field(a, ide ? 33u : 14u, 1);
    uint32_t hdr = fdf ? (ide ? 41u : 22u) : (ide ? 39u : 19u);
    if (nb < hdr) return -1;
    uint32_t dlc = canfd_field(a, hdr - 4u, 4), len = fdf ? canfd_len[dlc] : (dlc > 8u ? 8u : dlc);
    uint32_t end = hdr + 8u * len + (fdf ? (len > 16u ? 25u : 21u) : 15u);   /* CRC delimiter */
    if (nb < end + 2u) return -1;
    f->id = canfd_field(a, 1, 11);
    if (ide) f->id = (f->id << 18) | canfd_field(a, 14, 18) | CAN_EFF;
    f->status = st;
    f->len = (uint8_t)len;
    f->fd  = (uint8_t)fdf;
    f->brs = (uint8_t)(fdf && canfd_field(a, hdr - 6u, 1));
    for (uint32_t i = 0; i < len; i += 4u) {          /* four bytes per field read */
        uint32_t v = canfd_field(a, hdr + 8u * i, 32);
        for (uint32_t j = 0; j < 4u && i + j < len; j++) f->data[i + j] = (uint8_t)(v >> (24u - 8u * j));
    }
    f->crc_ok = (st & (FDCTL_DLM | FDCTL_CRC_OK)) == (FDCTL_DLM | FDCTL_CRC_OK) &&
                (!fdf || (st & FDCTL_SC_OK));
    f->acked = (uint8_t)!canfd_field(a, end + 1u, 1);    /* ACK slot */
    return 0;
}

static inline int canfd_recv(pg_wire_t *w, canfd_frame_t *f) { return canfd__recv(w, 0, f); }

/* The same from a DMA ring that receives W0's words (canfd_rx_dma): frames
 * arriving back to back are kept while C works on an earlier one. */
static inline int canfd_recv_dma(pg_dma_t *d, canfd_frame_t *f) { return canfd__recv(0, d, f); }

static inline void canfd_rx_dma(pg_dma_t *d, pg_wire_t *w, uint32_t *ring, uint32_t n) { dma_rx(d, w, ring, n); }

/* ---- S/PDIF and AES3 (IEC 60958 / AES3 line format) ------------------------
 * Biphase mark (WIRE_BIPHASE) on a full engine: one slot = 1/(64 fs). A
 * subframe is a preamble (sent as 8 RAW half slots at DIV2 = half a slot) and
 * 28 coded slots: 24-bit sample, V, U, C, even parity P. The parity keeps the
 * line low at every subframe end, so the preambles have one polarity.
 * RX (W1 only): the engine decodes by edge timing and starts a 30-bit word at
 * each preamble, type in bits [1:0] (SPDIF_B/M/W), slots 4..31 in bits [29:2].
 */
#define SPDIF_B 1u    /* channel A, first frame of a 192-frame block */
#define SPDIF_M 2u    /* channel A */
#define SPDIF_W 3u    /* channel B */

static inline uint32_t pg_parity(uint32_t x)
{
    x ^= x >> 16;
    x ^= x >> 8;
    x ^= x >> 4;
    x ^= x >> 2;
    x ^= x >> 1;
    return x & 1u;
}

static inline void spdif_tx_init(pg_wire_t *w, int pin, uint32_t fs)
{
    uint32_t fix = PG_BIT_FIX8(fs * 64u);
    w->CTRL   = 0;
    w->DIV    = fix >> 8;
    w->CTRL2  = WIRE_FRAC(fix) | WIRE_BIPHASE;
    w->DIV2   = WIRE_DIV2_FIX8(fix >> 1);
    w->PINS   = WIRE_PINS(pin, 0, 0);
    w->TXATTR = 0;
    w->CTRL   = WIRE_EN | WIRE_TX | WIRE_LEN(28);
}

/* Queue one subframe: pre = SPDIF_B/M/W, 24-bit sample, vuc = V | U << 1 | C << 2 */
static inline void spdif_put(pg_wire_t *w, uint32_t pre, uint32_t sample, uint32_t vuc)
{
    static const uint8_t pat[4] = {0, 0x17, 0x47, 0x27};   /* B, M, W half slots, first in bit 0 */
    uint32_t x = (sample & 0xFFFFFFu) | ((vuc & 7u) << 24);
    x |= pg_parity(x) << 27;
    w->TXATTR = WIRE_A_RAW | WIRE_A_FAST;
    wire_putn(w, 8, pat[pre & 3u]);
    w->TXATTR = 0;
    wire_putn(w, 28, x);
}

static inline void spdif_rx_init(pg_wire_t *w, int pin, uint32_t fs)
{
    uint32_t fix = PG_BIT_FIX8(fs * 64u);
    w->CTRL   = 0;
    w->DIV    = fix >> 8;
    w->CTRL2  = WIRE_BIPHASE | WIRE_RX_PACKET;
    w->PINS   = WIRE_PINS(0, pin, 0);
    w->CRCCFG = 1u << 24;                             /* SYNC_EN: nothing before the first preamble */
    w->CTRL   = WIRE_EN | WIRE_RX | WIRE_RX_ASYNC | WIRE_LEN(30);
}

/* Next subframe word (blocks): type in [1:0], slots 4..31 in [29:2] */
static inline uint32_t spdif_get(pg_wire_t *w) { return wire_get(w); }

/* ---- MIPI I3C SDR controller ------------------------------------------------
 * SDA is the engine's TX/RX pin in open-drain mode, SCL its push-pull clock.
 * The address header and its ACK run open drain at DIV (I2C-like timing);
 * data words run at DIV2 (up to 12.5 MHz) and controller writes are driven
 * push-pull (WIRE_A_PP) as the spec requires: 8 data bits and the T bit
 * (odd parity). In reads the target drives data and T (1 = more data).
 * START, repeated START and STOP are made with GPIO, as for I2C. Both pins must
 * be uio pins with pull-ups. Uses a full engine (W0 or W1).
 */
#define WIRE_I3C_CTRL_BITS (WIRE_EN | WIRE_RX | WIRE_MSB_FIRST | WIRE_IDLE_HIGH | WIRE_OE_OPEN_DRAIN | \
                            WIRE_TX_LATE | WIRE_RX_DELAY(3) | WIRE_LEN(9))

typedef i2c_t i3c_t;

static inline void i3c__engine(const i3c_t *b, int on)
{
    b->w->CTRL = WIRE_I3C_CTRL_BITS | (on ? WIRE_TX | WIRE_CLK_OUT : 0u);
}

/* od_hz: open-drain bit rate (header, ACK); pp_hz: push-pull data rate */
static inline void i3c_init(i3c_t *b, pg_wire_t *w, int sda_pin, int scl_pin, uint32_t od_hz, uint32_t pp_hz)
{
    b->w = w; b->sda = sda_pin; b->scl = scl_pin;
    b->quarter = PG_BIT_TICKS(od_hz) / 4u;
    i2c__pins(0, 0, b);
    w->CTRL   = 0;
    w->DIV    = PG_BIT_TICKS(od_hz);
    w->DIV2   = PG_BIT_TICKS(pp_hz);
    w->PINS   = WIRE_PINS(sda_pin, sda_pin, scl_pin);
    w->TXATTR = 0;
    i3c__engine(b, 0);
}

/* START, or repeated START after a transfer */
static inline void i3c_start(const i3c_t *b)
{
    if (b->w->CTRL & WIRE_TX) {
        wire_flush(b->w);
        i2c__pins(0, 1, b);             /* GPIO: SCL low, SDA released */
        i3c__engine(b, 0);
        delay_ticks(b->quarter);
        i2c__pins(0, 0, b);             /* SCL up */
        delay_ticks(2u * b->quarter);
    }
    i2c__pins(1, 0, b);                 /* SDA falls while SCL is high */
    delay_ticks(2u * b->quarter);
    i2c__pins(1, 1, b);                 /* SCL low */
    delay_ticks(b->quarter);
    i3c__engine(b, 1);
}

static inline void i3c_stop(const i3c_t *b)
{
    wire_flush(b->w);
    i2c__pins(1, 1, b);                 /* SDA and SCL low */
    i3c__engine(b, 0);
    delay_ticks(b->quarter);
    i2c__pins(1, 0, b);                 /* SCL up */
    delay_ticks(2u * b->quarter);
    i2c__pins(0, 0, b);                 /* SDA rises while SCL is high */
    delay_ticks(2u * b->quarter);
}

/* Address header (open drain); returns 1 if a target acknowledged */
static inline int i3c_header(const i3c_t *b, uint32_t addr, int rnw)
{
    wire_putn(b->w, 8, (addr << 1) | (rnw ? 1u : 0u));
    wire_putn(b->w, 1, 1u);             /* ACK slot: released */
    (void)wire_get(b->w);
    return (wire_get(b->w) & 1u) == 0u;
}

/* Data byte from the controller: push-pull, T = odd parity */
static inline void i3c_write(const i3c_t *b, uint8_t x)
{
    b->w->TXATTR = WIRE_A_FAST | WIRE_A_PP;
    wire_putn(b->w, 9, ((uint32_t)x << 1) | (pg_parity(x) ^ 1u));
    b->w->TXATTR = 0;
    (void)wire_get(b->w);
}

/* Data byte from the target; *more = its T bit (0 on the last byte) */
static inline uint8_t i3c_read(const i3c_t *b, int *more)
{
    b->w->TXATTR = WIRE_A_FAST;
    wire_putn(b->w, 9, 0x1FFu);         /* released: the target drives */
    b->w->TXATTR = 0;
    uint32_t x = wire_get(b->w);
    *more = (int)(x & 1u);
    return (uint8_t)(x >> 1);
}

/* Broadcast or direct CCC write: START 7E/W, code, data; returns the 7E ACK */
static inline int i3c_ccc(const i3c_t *b, uint8_t code, const uint8_t *d, uint32_t n, int stop)
{
    i3c_start(b);
    int ok = i3c_header(b, 0x7Eu, 0);
    if (ok) {
        i3c_write(b, code);
        for (uint32_t i = 0; i < n; i++) i3c_write(b, d[i]);
    }
    if (stop || !ok) i3c_stop(b);
    return ok;
}

/* Read up to max bytes from addr (after i3c_start); returns the number read
 * (the target ends with T = 0), or -1 without ACK. Ends with STOP. */
static inline int i3c_read_msg(const i3c_t *b, uint32_t addr, uint8_t *d, uint32_t max)
{
    int more = 1, n = 0;
    if (!i3c_header(b, addr, 1)) {
        i3c_stop(b);
        return -1;
    }
    while (more && (uint32_t)n < max) d[n++] = i3c_read(b, &more);
    i3c_stop(b);
    return n;
}

/* ---- USB Power Delivery (BMC, 4b5b) on W1 -----------------------------------
 * The engine does the biphase mark coding at 300 kbit/s both ways (TX on a
 * full engine, RX on W1 by edge timing); C does the 4b5b symbols, the CRC-32
 * and the message layer. The CC wire carries both directions: TX drives it
 * only while sending (WIRE_OE_FRAME, through the board's BMC driver), RX reads
 * it through a comparator, so RX also sees our own packets. RX hunts for the
 * end of the preamble and the first Sync-1 (SYNC pattern) and delivers 5-bit
 * symbols. Bits and symbols go LSB first, nibbles low first.
 */
#define PD_SYNC1 0x18u
#define PD_SYNC2 0x11u
#define PD_EOP   0x0Du
#define PD_CTRL_GOODCRC 1u
#define PD_CTRL_ACCEPT  3u
#define PD_CTRL_PS_RDY  6u
#define PD_DATA_SRC_CAP 1u
#define PD_DATA_REQUEST 2u
/* header: type, data role, spec revision 3.0, power role, message ID, objects */
#define PD_HEADER(type, drole, prole, id, cnt) ((uint32_t)(type) | ((uint32_t)(drole) << 5) | (2u << 6) | \
                                                ((uint32_t)(prole) << 8) | (((uint32_t)(id) & 7u) << 9) | \
                                                (((uint32_t)(cnt) & 7u) << 12))
#define PD_HDR_TYPE(h) ((h) & 0x1Fu)
#define PD_HDR_ID(h)   (((h) >> 9) & 7u)
#define PD_HDR_CNT(h)  (((h) >> 12) & 7u)

static const uint8_t pd_enc4b5b[16] = {0x1E, 0x09, 0x14, 0x15, 0x0A, 0x0B, 0x0E, 0x0F,
                                       0x12, 0x13, 0x16, 0x17, 0x1A, 0x1B, 0x1C, 0x1D};

static inline uint32_t pd_crc32(uint32_t crc, uint32_t x, int nbytes)   /* CRC-32 (IEEE), LSB first */
{
    for (int i = 0; i < nbytes; i++, x >>= 8) {
        crc ^= x & 0xFFu;
        for (int k = 0; k < 8; k++) crc = (crc >> 1) ^ (0xEDB88320u & (0u - (crc & 1u)));
    }
    return crc;
}

static inline void pd_init(pg_wire_t *w, int tx_pin, int rx_pin)
{
    uint32_t fix = PG_BIT_FIX8(300000);
    w->CTRL   = 0;
    w->DIV    = fix >> 8;
    w->CTRL2  = WIRE_FRAC(fix) | WIRE_BIPHASE | WIRE_RX_PACKET | WIRE_TAIL(1);
    w->PINS   = WIRE_PINS(tx_pin, rx_pin, 0);
    w->CRCCFG = (0xC5u << 16) | (1u << 24);           /* ...1,0,1 of the preamble + Sync-1 */
    w->TXATTR = 0;
    w->CTRL   = WIRE_EN | WIRE_TX | WIRE_RX | WIRE_RX_ASYNC | WIRE_OE_FRAME | WIRE_EVENTS | WIRE_LEN(5);
}

typedef struct { pg_wire_t *w; uint32_t acc, n, lvl; } pd_tx_t;

static inline void pd__sym(pd_tx_t *t, uint32_t s)    /* queue one 5-bit symbol */
{
    t->acc |= s << t->n;
    t->n += 5u;
    for (int i = 0; i < 5; i++, s >>= 1) t->lvl ^= (~s) & 1u;   /* a 0 bit flips the line once, a 1 twice */
    if (t->n == 30u) {
        wire_putn(t->w, 30, t->acc);
        t->acc = t->n = 0;
    }
}

static inline void pd__nibbles(pd_tx_t *t, uint32_t x, int n)
{
    while (n--) {
        pd__sym(t, pd_enc4b5b[x & 15u]);
        x >>= 4;
    }
}

/* Send a message (header + cnt objects) and wait until it has left the wire
 * and our own copy has gone through RX. */
static inline void pd_send(pg_wire_t *w, uint32_t header, const uint32_t *obj, uint32_t cnt)
{
    pd_tx_t t = {w, 0, 0, 0};
    uint32_t crc = pd_crc32(0xFFFFFFFFu, header, 2);
    for (uint32_t i = 0; i < cnt; i++) crc = pd_crc32(crc, obj[i], 4);
    wire_putn(w, 32, 0xAAAAAAAAu);                    /* preamble: 64 bits 0101..., first 0 */
    wire_putn(w, 32, 0xAAAAAAAAu);
    pd__sym(&t, PD_SYNC1); pd__sym(&t, PD_SYNC1); pd__sym(&t, PD_SYNC1); pd__sym(&t, PD_SYNC2);
    pd__nibbles(&t, header, 4);
    for (uint32_t i = 0; i < cnt; i++) pd__nibbles(&t, obj[i], 8);
    pd__nibbles(&t, ~crc, 8);
    pd__sym(&t, PD_EOP);
    if (t.n) wire_putn(w, (int)t.n, t.acc);
    if (!t.lvl) {                                     /* end with an edge: one high bit, then low */
        w->TXATTR = WIRE_A_RAW;
        wire_putn(w, 1, 1u);
        w->TXATTR = 0;
    }
    wire_flush(w);
    uint32_t t0 = now();
    while (!time_reached(t0 + PG_US_TO_TICKS(20))) {  /* drop our own copy */
        if (wire_rx_count(w) && WIRE_IS_END(wire_get(w))) break;
    }
}

/* Receive a message within timeout_us. Returns the header, or -1 (timeout,
 * bad symbol or CRC). obj[] gets up to 7 objects. */
static inline int32_t pd_recv(pg_wire_t *w, uint32_t *obj, uint32_t timeout_us)
{
    static const uint8_t dec[32] = {16, 16, 16, 16, 16, 16, 16, 16, 16, 1, 4, 5, 16, 16, 6, 7,
                                    16, 16, 8, 9, 2, 3, 10, 11, 16, 16, 12, 13, 14, 15, 0, 16};
    uint32_t t0 = now(), k = 0, header = 0, crc = 0xFFFFFFFFu, n = 0, x, nib = 0, cnt = 7, got = 0;
    for (;;) {
        while (!wire_rx_count(w))
            if (time_reached(t0 + PG_US_TO_TICKS(timeout_us))) return -1;
        x = wire_get(w);
        if (WIRE_IS_END(x)) {
            if (got) return -1;                       /* ended early */
            k = 0; continue;
        }
        if (k < 3u) {                                 /* the rest of the SOP: Sync-1, Sync-1, Sync-2 */
            if (x != (k < 2u ? PD_SYNC1 : PD_SYNC2)) k = 99;
            else k++;
            continue;
        }
        if (k == 99) continue;                        /* not a SOP packet: skip to its end */
        uint32_t d = dec[x & 31u];
        if (d > 15u) return -1;
        nib |= d << (4u * (n < 4u ? n : (n - 4u) & 7u));
        n++;
        if (n == 4u) {
            header = nib;
            cnt = PD_HDR_CNT(header);
            crc = pd_crc32(crc, header, 2);
            nib = 0;
            got = 1;
        } else if (n > 4u && ((n - 4u) & 7u) == 0) {
            uint32_t i = (n - 4u) / 8u - 1u;
            if (i < cnt) {
                obj[i] = nib;
                crc = pd_crc32(crc, nib, 4);
            } else {
                if (nib != ~crc) return -1;           /* CRC */
                /* EOP and the end marker follow: drain them */
                for (uint32_t m = 0; m < 4u; m++) {
                    uint32_t t1 = now();
                    while (!wire_rx_count(w) && !time_reached(t1 + PG_US_TO_TICKS(30))) { }
                    if (!wire_rx_count(w) || WIRE_IS_END(wire_get(w))) break;
                }
                return (int32_t)header;
            }
            nib = 0;
        }
    }
}

/* ---- SMPTE linear timecode (LTC) -------------------------------------------
 * 80-bit frames in biphase mark: TX on a full engine (three words), RX on W1
 * (SYNC hunt on the end of the sync word, then 16-bit words: five per frame,
 * the fifth is the sync word 0xBFFC). Fields are BCD, LSB first.
 */
typedef struct { uint8_t h, m, s, f; } ltc_time_t;

/* tx = 1: the engine drives tx_pin (W0/W1); RX on W1 */
static inline void ltc_init(pg_wire_t *w, int tx_pin, int rx_pin, uint32_t fps, int tx)
{
    uint32_t fix = PG_BIT_FIX8(fps * 80u);
    w->CTRL   = 0;
    w->DIV    = fix >> 8;
    w->CTRL2  = WIRE_FRAC(fix) | WIRE_BIPHASE | WIRE_RX_PACKET;
    w->PINS   = WIRE_PINS(tx_pin, rx_pin, 0);
    w->CRCCFG = (0xBFu << 16) | (1u << 24);          /* last 8 bits of the sync word */
    w->TXATTR = 0;
    w->CTRL   = WIRE_EN | (tx ? WIRE_TX : WIRE_RX | WIRE_RX_ASYNC) | WIRE_LEN(16);
}

static inline uint32_t ltc__bcd(uint32_t v) { return (v / 10u) << 4 | (v % 10u); }

static inline void ltc_send(pg_wire_t *w, ltc_time_t t)
{
    uint32_t f = ltc__bcd(t.f), s = ltc__bcd(t.s), m = ltc__bcd(t.m), h = ltc__bcd(t.h);
    uint32_t lo = (f & 15u) | ((f >> 4) << 8) | ((s & 15u) << 16) | ((s >> 4) << 24);
    uint32_t hi = (m & 15u) | ((m >> 4) << 8) | ((h & 15u) << 16) | ((h >> 4) << 24);
    uint32_t sync = 0xBFFCu;                           /* bits 64..79, LSB first */
    /* polarity correction (bit 27): an even number of zeros in the frame */
    if ((80u - (uint32_t)__builtin_popcount(lo) - (uint32_t)__builtin_popcount(hi) -
         (uint32_t)__builtin_popcount(sync)) & 1u) lo |= 1u << 27;
    wire_putn(w, 32, lo);
    wire_putn(w, 32, hi);
    wire_putn(w, 16, sync);
}

/* Decode the five RX words of a frame: 0, or -1 if the fifth is not the sync word */
static inline int ltc_decode(const uint32_t *x, ltc_time_t *t)
{
    if (x[4] != 0xBFFCu) return -1;
    t->f = (uint8_t)((x[0] & 15u) + 10u * ((x[0] >> 8) & 3u));
    t->s = (uint8_t)((x[1] & 15u) + 10u * ((x[1] >> 8) & 7u));
    t->m = (uint8_t)((x[2] & 15u) + 10u * ((x[2] >> 8) & 7u));
    t->h = (uint8_t)((x[3] & 15u) + 10u * ((x[3] >> 8) & 3u));
    return 0;
}

/* ---- 100 Mbit/s Ethernet through an RMII PHY (experimental, pg_rmii) ---------
 * Only in builds with the RMII port. The chip clock must be the PHY's 50 MHz
 * REF_CLK. The hardware sends preamble, SFD and FCS and checks the received
 * FCS; C moves frame words: one every 16 cycles, so these loops are tight.
 * Pins: TXD0 uo[0], TXD1 uo[1], TX_EN uo[2]; RXD0 ui[0], RXD1 ui[1], CRS_DV ui[2].
 */
typedef struct {
    volatile uint32_t TX, RX, RXHEAD, TXLEN, CTRL, STATUS;
} pg_rmii_t;

#define RMII        ((pg_rmii_t *)0x80000500u)
#define RMII_END    (1u << 31)
#define RMII_FCS_OK (1u << 16)
#define RMII_TX_BUSY     1u
#define RMII_TX_UNDERRUN 2u
#define RMII_RX_OVERFLOW 4u

static inline void rmii_init(void) { RMII->CTRL = 1; RMII->STATUS = RMII_TX_UNDERRUN | RMII_RX_OVERFLOW; }

/* Receive a frame into buf (4-byte aligned, max bytes). Returns its length
 * with the FCS, -1 if the FCS is wrong, -2 if it did not fit. Blocks. */
static inline int rmii_recv(uint32_t *buf, uint32_t max)
{
    uint32_t n = 0, h;
    max >>= 2;
    while (!((h = RMII->RXHEAD) & RMII_END)) {
        uint32_t x = RMII->RX;
        if (n < max) buf[n] = x;
        n++;
    }
    (void)RMII->RX;                                  /* pop the end entry */
    if (n > max) return -2;
    return (h & RMII_FCS_OK) ? (int)(h & 0xFFFFu) : -1;
}

/* Send n bytes (without FCS; at least 60 for Ethernet: pad first). */
static inline void rmii_send(const uint32_t *f, uint32_t n)
{
    uint32_t words = (n + 3u) >> 2, i = 0;
    while (RMII->STATUS & RMII_TX_BUSY) { }          /* previous frame and gap */
    RMII->TX = f[i++];                               /* two words queued, then start */
    if (i < words) RMII->TX = f[i++];
    RMII->TXLEN = n;
    while (i < words) RMII->TX = f[i++];
}

#endif /* POLYGLOT_PROTOCOL_H */
