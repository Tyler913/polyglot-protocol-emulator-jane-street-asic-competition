/*
 * Copyright (c) 2026 Tyler Hong
 * SPDX-License-Identifier: Apache-2.0
 *
 * pdm_mic: records audio from a PDM MEMS microphone (the kind in phones
 * and laptops). The host drives the microphone's clock; the microphone
 * answers with one bit per clock, the output of its sigma-delta modulator.
 * Here an SPI engine drives the clock (50 MHz / 48 = 1.042 MHz) and samples
 * DATA on rising edges (microphone with SELECT low); C keeps the engine
 * clocking and runs a third-order CIC decimation filter (decimation 64,
 * 16.28 kHz output), all in plain C: about 260 cycles per 32 input bits
 * out of the 1536 available.
 *
 * Pins: CLK uo[2] (P10), DATA ui[4] (P20); uo[3] (P11) is the unused data
 * output of the SPI engine.
 * Host parameters (PG_SHARED): [0] samples to record (1..112).
 * Results: [2] 0x600D0011 when done, [8..] samples, two 16-bit values per
 * word (first in the low half), signed: CIC output - 2^17, divided by 4.
 */

#include "protocol.h"

#define PDM W2

int main(void)
{
    uint32_t want = PG_SHARED[0], got = 0, cnt = 0;
    uint32_t i1 = 0, i2 = 0, i3 = 0, d1 = 0, d2 = 0, d3 = 0;
    if (want < 1u || want > 112u) want = 112u;

    wire_config(PDM, WIRE_SPI_CTRL(PIN_UO(2), PIN_UO(3), PIN_UI(4), 0, 1041667));
    PDM->CTRL = (PDM->CTRL & ~WIRE_LEN(32)) | WIRE_LEN(32);
    host_irq(1);                                   /* ready */
    wire_put(PDM, 0);                              /* keep two words queued: the clock never stops */
    wire_put(PDM, 0);
    while (got < want) {
        uint32_t w = wire_get(PDM);
        wire_put(PDM, 0);
        for (int b = 31; b >= 0; b--) {            /* MSB = first bit */
            i1 += (w >> b) & 1u;                   /* integrators (modulo 2^32) */
            i2 += i1;
            i3 += i2;
            if (++cnt == 64u) {                    /* decimate, then three combs */
                cnt = 0;
                uint32_t c1 = i3 - d1; d1 = i3;
                uint32_t c2 = c1 - d2; d2 = c1;
                uint32_t c3 = c2 - d3; d3 = c2;
                int32_t s = ((int32_t)c3 - (1 << 17)) >> 2;   /* gain 64^3 = 2^18, mid = 2^17 */
                if (got < want) {
                    uint32_t k = 8u + (got >> 1);
                    if (got & 1u) PG_SHARED[k] |= (uint32_t)(s & 0xFFFF) << 16;
                    else PG_SHARED[k] = (uint32_t)(s & 0xFFFF);
                    got++;
                }
            }
        }
    }
    PG_SHARED[2] = 0x600D0011u;
    host_irq(0);
    pg_park();
}
