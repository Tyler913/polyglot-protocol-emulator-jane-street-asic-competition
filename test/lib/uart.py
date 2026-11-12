# Copyright (c) 2026 Tyler Hong
# SPDX-License-Identifier: Apache-2.0
"""Independent UART peer model (8N1), written from the UART frame format,
not from the RTL. Time is counted in system clock cycles.

UartMonitor decodes a line by sampling at the middle of each bit and, in
strict mode, also checks that every edge inside a frame falls exactly on a
bit boundary (start edge + k * bit_cycles).
UartDriver sends frames on a ui_in bit.
"""

import cocotb
from cocotb.triggers import ClockCycles, ReadOnly, RisingEdge

from .pins import ui_pins


def bit_cycles(baud, f_clk=50_000_000):
    """Cycles per bit, rounded to nearest (the UART spec only needs ~2%)."""
    return (f_clk + baud // 2) // baud


class UartMonitor:
    def __init__(self, dut, get_line, bit, strict=True):
        self.dut = dut
        self.get_line = get_line  # callable returning the current line level
        self.bit = bit
        self.strict = strict
        self.bytes = []
        self.errors = []
        self._task = None

    def start(self):
        self._task = cocotb.start_soon(self._run())

    def stop(self):
        if self._task:
            self._task.cancel()

    async def _sample(self):
        await RisingEdge(self.dut.clk)
        await ReadOnly()
        return self.get_line()

    async def _run(self):
        clk_n = 0
        last = 0  # a start edge counts only after the line has been seen idle (high)
        while True:
            # Wait for the falling start edge
            while True:
                v = await self._sample()
                clk_n += 1
                if last == 1 and v == 0:
                    break
                last = v
            start = clk_n
            edges = []
            level = 0
            samples = []
            # Walk through the frame cycle by cycle until the middle of the stop bit
            frame_end = start + 9 * self.bit + self.bit // 2
            while clk_n < frame_end:
                v = await self._sample()
                clk_n += 1
                if v != level:
                    edges.append(clk_n - start)
                    level = v
                pos = clk_n - start
                if pos % self.bit == self.bit // 2:
                    samples.append(v)
            if len(samples) != 10:
                self.errors.append(f"internal: {len(samples)} samples")
            if samples[0] != 0:
                self.errors.append(f"glitch: start bit not low at mid-bit (frame at {start})")
            if samples[9] != 1:
                self.errors.append(f"framing error: stop bit low (frame at {start})")
            byte = sum(b << i for i, b in enumerate(samples[1:9]))
            if self.strict:
                bad = [e for e in edges if e % self.bit != 0]
                if bad:
                    self.errors.append(f"byte {byte:#04x}: edges off the bit grid at {bad} (bit = {self.bit})")
            self.bytes.append(byte)
            last = level


class UartDriver:
    def __init__(self, dut, ui_bit, bit):
        self.dut = dut
        self.ui = ui_pins(dut)
        self.ui_bit = ui_bit
        self.bit = bit
        self.ui.set(ui_bit, 1)

    async def send(self, data, gap_bits=0, stop=1):
        for b in data:
            frame = [0] + [(b >> i) & 1 for i in range(8)] + [stop]
            for v in frame:
                self.ui.set(self.ui_bit, v)
                await ClockCycles(self.dut.clk, self.bit)
            self.ui.set(self.ui_bit, 1)
            if gap_bits:
                await ClockCycles(self.dut.clk, gap_bits * self.bit)
