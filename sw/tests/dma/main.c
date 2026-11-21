/*
 * Copyright (c) 2026 Tyler Hong
 * SPDX-License-Identifier: Apache-2.0
 *
 * dma: the DMA channels without the CPU in the loop. Used by test/test_dma.py.
 *   mode 1: UART RX at 1 Mbaud on Wire Engine 2 into a ring by DMA0 while
 *           the CPU does not touch the engine; then the bytes are reported.
 *   mode 2: UART TX at 1 Mbaud on Wire Engine 2 from a ring by DMA0 (the CPU
 *           only fills the ring); frames must follow each other without gaps.
 *   mode 3: logic analyzer: DMA1 samples GPIO_IN every 10 cycles (N samples)
 *           while DMA0 plays a pattern on GPIO_OUT (uo pins) every 25 cycles.
 * Pins: UART RX ui[1] (P17), UART TX uo[0] (P8); pattern on uo[1..5].
 * Host parameters (PG_SHARED): [0] mode, [1] count.
 * Results: [0] 0x600D0D3A when done, [1] words, [2] DMA STATUS bits,
 * [8..] data (mode 1: bytes packed little-endian; mode 3: samples).
 */

#include "protocol.h"

static uint32_t ring[160];

int main(void)
{
    uint32_t mode = PG_SHARED[0], n = PG_SHARED[1];

    if (mode == 1u) {                                /* RX: n bytes, no CPU until all are in */
        wire_config(W2, WIRE_UART_8N1(PIN_UO(0), PIN_UI(1), 1000000));
        dma_rx(DMA0, W2, ring, 160);
        host_irq(1);
        while (dma_count(DMA0) < n) delay_ticks(PG_US_TO_TICKS(50));
        for (uint32_t i = 0; i < n; i++) {
            uint32_t f = dma_get(DMA0);
            ((volatile uint8_t *)&PG_SHARED[8])[i] = (uint8_t)(f >> 1);
            if ((f & 0x201u) != 0x200u) PG_SHARED[2] |= 0x100u;   /* framing */
        }
        PG_SHARED[1] = n;
        PG_SHARED[2] |= DMA0->STATUS | (W2->STATUS & WIRE_RX_OVERFLOW ? 0x200u : 0u);
    } else if (mode == 2u) {                         /* TX: n bytes 0x30, 0x31, ... */
        wire_config(W2, WIRE_UART_8N1(PIN_UO(0), PIN_UI(1), 1000000));
        dma_tx(DMA0, W2, ring, 160);
        host_irq(1);
        for (uint32_t i = 0; i < n; i++) {
            while (!dma_room(DMA0)) { }
            dma_put(DMA0, 0x200u | ((0x30u + i) << 1));
        }
        while (!(DMA0->STATUS & DMA_EMPTY)) { }
        wire_flush(W2);
        PG_SHARED[1] = n;
    } else {                                         /* capture + pattern */
        static uint32_t pat[40];
        for (uint32_t i = 0; i < 40; i++) pat[i] = (((i * 7u) & 31u) << 9) | (PG_GPIO_OUT & 0x1FFu);
        if (n > 56u) n = 56u;
        host_irq(1);
        dma_capture(DMA1, ring, n + 1u, 10);
        dma_pattern(DMA0, pat, 40, 39, 25);
        while (!(DMA1->STATUS & DMA_FULL) || !(DMA0->STATUS & DMA_EMPTY)) { }
        for (uint32_t i = 0; i < n; i++) PG_SHARED[8 + i] = ring[i];
        PG_SHARED[1] = n;
        PG_SHARED[2] = DMA0->STATUS | (DMA1->STATUS << 4);
    }
    PG_SHARED[0] = 0x600D0D3Au;
    host_irq(0);
    pg_park();
}
