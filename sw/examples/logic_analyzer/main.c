/*
 * Copyright (c) 2026 Tyler Hong
 * SPDX-License-Identifier: Apache-2.0
 *
 * logic_analyzer: the 14 KB SRAM as a capture buffer. DMA channel 1 samples
 * GPIO_IN (the ui and uio pins, and the uo pins as driven) every PERIOD
 * cycles; the host reads the buffer over SPI when the run is done.
 *
 *   mode 0, burst: raw 32-bit samples straight into BUF_WORDS words of SRAM
 *          (about 12.4 KB): 3,099 samples, up to 25 MS/s (PERIOD 2).
 *   mode 1, change log: the DMA fills a 64-word ring; the CPU keeps only
 *          changes, one word each: [31:16] ui[7:0] << 8 | uio[7:0],
 *          [15:0] samples since the previous entry (0xFFFF: no change for
 *          65,535 samples, entry repeats the value). Sparse signals (serial
 *          buses) fit seconds of activity into the same memory. The DMA
 *          runs as a stream: if the CPU falls behind (above about 1.7 MS/s),
 *          samples are lost and the late flag ([10] bit 0) says so.
 *
 * Host parameters: [0] mode, [1] period (cycles, >= 1), [2] samples (mode 0;
 * 0 = all), [3] trigger mask, [4] trigger value (start when (GPIO_IN & mask)
 * == value; mask 0 = at once), [5] mode 1: stop after this many samples.
 * Results: [8] 0x600D10A5 when done, [9] words in the buffer, [10] DMA status
 * (bit 0 = a sample came late: PERIOD too short), [11] buffer address,
 * [12] BUF_WORDS, [13] mode 1: ring overruns (CPU too slow), [14] cycles
 * from trigger to the end.
 */

#include "protocol.h"

#define BUF_WORDS 3100
static uint32_t buf[BUF_WORDS];

static uint32_t compact(uint32_t v) { return (v & 0xFFu) | ((v >> 8) & 0xFF00u); }

int main(void)
{
    uint32_t mode = PG_SHARED[0], period = PG_SHARED[1] ? PG_SHARED[1] : 1u;
    uint32_t n = PG_SHARED[2], mask = PG_SHARED[3], val = PG_SHARED[4], limit = PG_SHARED[5];
    for (int i = 8; i < 16; i++) PG_SHARED[i] = 0;
    PG_SHARED[11] = (uint32_t)(uintptr_t)buf;
    PG_SHARED[12] = BUF_WORDS;
    if (n == 0u || n > BUF_WORDS - 1u) n = BUF_WORDS - 1u;
    host_irq(1);                                       /* armed */

    while ((PG_GPIO_IN & mask) != val) { }            /* trigger */
    uint32_t t0 = now();

    if (mode == 0u) {
        dma_capture(DMA1, buf, n + 1u, period);       /* n samples, then the ring is full */
        while (!(DMA1->STATUS & DMA_FULL)) { }
        PG_SHARED[10] = DMA1->STATUS;
        dma_stop(DMA1);
        PG_SHARED[9] = n;
    } else {
        static uint32_t ring[64];
        uint32_t k = 0, last, run = 0, seen = 1, ti = 1;
        const uint32_t base = (uint32_t)(uintptr_t)ring;
        dma_capture_stream(DMA1, ring, 64, period);   /* a full ring = lost samples */
        while (DMA1->HEAD == base) { }
        last = compact(ring[0]);
        DMA1->TAIL = base + 4u;
        buf[k++] = last << 16;                         /* first entry: the start value */
        while (seen < limit && k < BUF_WORDS) {
            /* the whole batch the DMA has written: read SRAM directly, move TAIL once */
            uint32_t h = (DMA1->HEAD - base) >> 2;
            if (((h - ti) & 63u) >= 60u) PG_SHARED[13]++;   /* ring nearly full: CPU behind */
            while (ti != h && seen < limit) {
                uint32_t v = compact(ring[ti]);
                ti = (ti + 1u) & 63u;
                seen++;
                run++;
                if (v != last || run == 0xFFFFu) {
                    buf[k++] = (v << 16) | run;
                    last = v;
                    run = 0;
                    if (k == BUF_WORDS) break;
                }
            }
            DMA1->TAIL = base + 4u * ti;
        }
        PG_SHARED[10] = DMA1->STATUS;
        dma_stop(DMA1);
        PG_SHARED[9] = k;
    }
    PG_SHARED[14] = now() - t0;
    PG_SHARED[8] = 0x600D10A5u;
    host_irq(0);
    pg_park();
}
