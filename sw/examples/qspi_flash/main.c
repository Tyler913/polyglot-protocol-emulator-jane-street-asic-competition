/*
 * Copyright (c) 2026 Tyler Hong
 * SPDX-License-Identifier: Apache-2.0
 *
 * qspi_flash: reads a quad-SPI NOR flash (W25Q-style command 0x6B, "Fast
 * Read Quad Output") with all four engines as one 4-bit port. Engine 0 is
 * the SPI controller: it sends the command and the 24-bit address on IO0,
 * releases IO0 for the 8 dummy clocks (the turnaround), then keeps clocking
 * SCK while the flash drives IO0..IO3,
 * a nibble per clock (high nibble first). Engines 1..3 receive IO1..IO3
 * with SCK as their external clock, so all four lanes sample the same
 * edges; they are enabled only after the command phase. C interleaves the
 * four 32-bit lane words into 16 bytes. SPI mode 0 at 5 MHz.
 *
 * Pins: SCK uo[0] (P8), CS# uo[1] (P9), IO0 uio[0] (P0, both ways),
 * IO1..IO3 ui[0..2] (P16..P18).
 * Host parameters (PG_SHARED): [0] flash address, [1] bytes (16..64, a
 * multiple of 16).
 * Results: [2] 0x600D0010 when done, [8..23] the bytes (little endian).
 */

#include "protocol.h"

#define SCK PIN_UO(0)
#define CS  PIN_UO(1)
#define IO0 PIN_UIO(0)

int main(void)
{
    uint32_t addr = PG_SHARED[0], n = PG_SHARED[1];
    static pg_wire_t *const lane[4] = { W0, W1, W2, W3 };
    if (n < 16u || n > 64u) n = 64u;
    n &= ~15u;

    pin_write(CS, 1);
    /* RX sample 3 cycles after mid-bit: SCK leaves through a flip-flop and IO0
     * comes back through the input synchronizer */
    wire_config(W0, WIRE_SPI_CTRL_DLY(SCK, IO0, IO0, 0, 5000000, 3));
    host_irq(1);                                   /* ready */
    pin_write(CS, 0);
    wire_putn(W0, 8, 0x6B);                        /* Fast Read Quad Output */
    wire_putn(W0, 24, addr & 0xFFFFFFu);
    wire_flush(W0);                                /* SCK idle low; drop the RX words */
    while (wire_rx_count(W0)) (void)wire_get(W0);
    /* Turnaround: release IO0 before the dummy clocks, after which the flash drives it */
    W0->CTRL = (W0->CTRL & ~(3u << 10)) | WIRE_OE_NEVER;
    wire_putn(W0, 8, 0);                           /* 8 dummy clocks */
    (void)wire_get(W0);

    /* Data phase: engine 0 samples IO0; engines 1..3 follow SCK on IO1..IO3 */
    for (int k = 1; k < 4; k++) {
        lane[k]->CTRL = 0;
        lane[k]->PINS = WIRE_PINS(0, PIN_UI(k - 1), SCK);
        lane[k]->CTRL = WIRE_EN | WIRE_RX | WIRE_MSB_FIRST | WIRE_CLK_EXT | WIRE_OE_NEVER | WIRE_LEN(32);
    }
    for (uint32_t g = 0; g < n / 16u; g++) {       /* 32 clocks = 16 bytes per group */
        uint32_t w[4], out[4] = {0, 0, 0, 0};
        wire_putn(W0, 32, 0);
        for (int k = 0; k < 4; k++) w[k] = wire_get(lane[k]);
        for (uint32_t c = 0; c < 32u; c++) {       /* clock c: bits 31 - c of each lane */
            uint32_t s = 31u - c;
            uint32_t nib = ((w[3] >> s) & 1u) << 3 | ((w[2] >> s) & 1u) << 2 |
                           ((w[1] >> s) & 1u) << 1 | ((w[0] >> s) & 1u);
            uint32_t byte = c >> 1, sh = 8u * (byte & 3u) + ((c & 1u) ? 0u : 4u);
            out[byte >> 2] |= nib << sh;
        }
        for (int k = 0; k < 4; k++) PG_SHARED[8 + 4 * g + k] = out[k];
    }
    pin_write(CS, 1);
    for (int k = 1; k < 4; k++) lane[k]->CTRL = 0;
    PG_SHARED[2] = 0x600D0010u;
    host_irq(0);
    pg_park();
}
