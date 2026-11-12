/*
 * Copyright (c) 2026 Tyler Hong
 * SPDX-License-Identifier: Apache-2.0
 *
 * protocol.h: the whole programming interface of the Polyglot chip.
 *
 * Version 0 (bring-up): system registers, cycle counter and plain GPIO.
 * Wire engines and timed GPIO are added in later versions.
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

/* ---- Pin numbering ------------------------------------------------------ */

/* 0..7: uio[0..7] (bidirectional), 8..13: uo[0..5] (outputs), 16..20: ui[0..4] (inputs) */
#define PIN_UIO(n)  (n)
#define PIN_UO(n)   (8 + (n))
#define PIN_UI(n)   (16 + (n))

/* ---- System registers --------------------------------------------------- */

#define PG_SYS_BASE 0x80000000u
#define PG_REG(off) (*(volatile uint32_t *)(PG_SYS_BASE + (off)))

#define PG_ID        PG_REG(0x00)
#define PG_NOW       PG_REG(0x04)
#define PG_GPIO_OUT  PG_REG(0x10)
#define PG_GPIO_OE   PG_REG(0x14)
#define PG_GPIO_IN   PG_REG(0x18)
#define PG_HOST_MBOX PG_REG(0x20)

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

#endif /* POLYGLOT_PROTOCOL_H */
