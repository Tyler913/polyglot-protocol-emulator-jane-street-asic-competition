/*
 * Copyright (c) 2026 Tyler Hong
 * SPDX-License-Identifier: Apache-2.0
 *
 * hd44780: a character LCD (Hitachi HD44780 and compatibles) on its 4-bit
 * parallel bus, write only (R/W tied low), so the busy flag is never read and
 * each instruction waits its worst-case execution time instead. The bus
 * timing (address setup, enable pulse width, data setup and hold, enable
 * cycle) comes from the timed GPIO queue: each nibble is three scheduled
 * pin changes at exact cycles, whatever the C code around them does.
 *
 * Pins: RS uo[0] (P8), E uo[1] (P9), D4..D7 uo[2..5] (P10..P13).
 * Host parameters: [0] power-on wait in us (0 = 40000, the data sheet value).
 * Writes "Polyglot" on line 1 and "RISC-V C" on line 2.
 * Results (PG_SHARED): [1] 0x600D44C0 when done.
 */

#include "protocol.h"

#define RS   PIN_UO(0)
#define E    PIN_UO(1)
#define D4   PIN_UO(2)
#define MASK ((1u << RS) | (1u << E) | (0xFu << D4))

static uint32_t t;   /* time of the next bus action */

static void nibble(int rs, uint32_t n)
{
    uint32_t v = (rs ? 1u << RS : 0u) | ((n & 0xFu) << D4);
    pins_out_at(MASK, v, t);                                   /* RS, data; E low */
    pins_out_at(MASK, v | (1u << E), t + PG_NS_TO_TICKS(60));  /* tAS >= 40 ns */
    pins_out_at(MASK, v, t + PG_NS_TO_TICKS(360));             /* PWEH >= 230 ns, tDSW >= 80 ns */
    t += PG_NS_TO_TICKS(600);                                  /* tcycE >= 500 ns, tH >= 10 ns */
}

static void lcd_write(int rs, uint8_t b, uint32_t exec_us)
{
    nibble(rs, b >> 4);
    nibble(rs, b);
    t += PG_US_TO_TICKS(exec_us);
}

static void lcd_text(const char *s) { while (*s) lcd_write(1, (uint8_t)*s++, 43); }

int main(void)
{
    uint32_t pon = PG_SHARED[0] ? PG_SHARED[0] : 40000u;
    pins_out_at(MASK, 0, now() + 10);
    t = now() + PG_US_TO_TICKS(pon);
    /* Initialization by instruction, 4-bit interface (data sheet figure 24) */
    nibble(0, 0x3); t += PG_US_TO_TICKS(4100);
    nibble(0, 0x3); t += PG_US_TO_TICKS(100);
    nibble(0, 0x3); t += PG_US_TO_TICKS(37);
    nibble(0, 0x2); t += PG_US_TO_TICKS(37);
    lcd_write(0, 0x28, 37);              /* function set: 4-bit, 2 lines, 5x8 dots */
    lcd_write(0, 0x08, 37);              /* display off */
    lcd_write(0, 0x01, 1520);            /* clear display */
    lcd_write(0, 0x06, 37);              /* entry mode: increment, no shift */
    lcd_write(0, 0x0C, 37);              /* display on, cursor off */
    lcd_text("Polyglot");
    lcd_write(0, 0x80 | 0x40, 37);       /* DDRAM address 0x40: line 2 */
    lcd_text("RISC-V C");
    timed_flush();
    PG_SHARED[1] = 0x600D44C0u;
    host_irq(1);
    pg_park();
}
