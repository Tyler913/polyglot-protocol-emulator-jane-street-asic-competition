/*
 * Copyright (c) 2026 Tyler Hong
 * SPDX-License-Identifier: Apache-2.0
 *
 * midi: a MIDI 1.0 sender (31250 baud, 8N1, current loop driver outside)
 * playing a short phrase with running status: note-on messages for the
 * same channel omit the repeated status byte, and a note-on with velocity
 * 0 ends a note. Timing between notes comes from the NOW counter.
 *
 * Pin: MIDI OUT uo[1] (P9). Results (PG_SHARED): [0] 0x600D0001 when done.
 */

#include "protocol.h"

#define MIDI W1

int main(void)
{
    static const uint8_t notes[4] = { 60, 64, 67, 72 };   /* C major arpeggio */
    wire_config(MIDI, WIRE_UART_8N1(PIN_UO(1), 0, 31250));
    uart_putc(MIDI, 0xC0); uart_putc(MIDI, 0x05);           /* program change: channel 1, program 6 */
    uart_putc(MIDI, 0x90);                                   /* note on, channel 1 (running status from here) */
    for (int i = 0; i < 4; i++) {
        uart_putc(MIDI, notes[i]); uart_putc(MIDI, 100);     /* note on */
        delay_ticks(PG_US_TO_TICKS(2000));
        uart_putc(MIDI, notes[i]); uart_putc(MIDI, 0);       /* note off as velocity 0 */
    }
    uart_putc(MIDI, 0xB0); uart_putc(MIDI, 123); uart_putc(MIDI, 0);   /* all notes off */
    wire_flush(MIDI);
    PG_SHARED[0] = 0x600D0001u;
    host_irq(1);
    pg_park();
}
