# Copyright (c) 2026 Tyler Hong
# SPDX-License-Identifier: Apache-2.0
"""Independent UART peer model (8N1), written from the UART frame format,
not from the RTL. Time is counted in system clock cycles.

Bit periods may be fractional (e.g. 16.667 cycles at 3 Mbaud, 50 MHz).
UartMonitor decodes a line by sampling at the middle of each bit and, in
strict mode, also checks that every edge inside a frame lies within one
cycle of a bit boundary (start edge + k * bit period).
UartDriver sends frames on a ui_in bit, optionally with a rate error.
"""

import cocotb
from cocotb.triggers import ClockCycles, ReadOnly, RisingEdge

from .pins import ui_pins


def bit_cycles(baud, f_clk=50_000_000):
    """Cycles per bit (exact, may be fractional)."""
    return f_clk / baud


class UartMonitor:
    def __init__(self, dut, get_line, bit, strict=True):
        self.dut = dut
        self.get_line = get_line  # callable returning the current line level
        self.bit = bit
        self.strict = strict
        self.bytes = []
        self.times = []  # start-edge cycle of each byte (counted from start())
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
            points = [round((k + 0.5) * self.bit) for k in range(10)]
            # Walk through the frame cycle by cycle until the middle of the stop bit
            frame_end = start + points[-1]
            while clk_n < frame_end:
                v = await self._sample()
                clk_n += 1
                if v != level:
                    edges.append(clk_n - start)
                    level = v
                if clk_n - start in points:
                    samples.append(v)
            if len(samples) != 10:
                self.errors.append(f"internal: {len(samples)} samples")
            if samples[0] != 0:
                self.errors.append(f"glitch: start bit not low at mid-bit (frame at {start})")
            if samples[9] != 1:
                self.errors.append(f"framing error: stop bit low (frame at {start})")
            byte = sum(b << i for i, b in enumerate(samples[1:9]))
            if self.strict:
                bad = [e for e in edges if abs(e / self.bit - round(e / self.bit)) * self.bit > 1.0]
                if bad:
                    self.errors.append(f"byte {byte:#04x}: edges off the bit grid at {bad} (bit = {self.bit:.3f})")
            self.bytes.append(byte)
            self.times.append(start)
            last = level


class UartDriver:
    def __init__(self, dut, ui_bit, bit, rate_error=0.0):
        self.dut = dut
        self.ui = ui_pins(dut)
        self.ui_bit = ui_bit
        self.bit = bit / (1.0 + rate_error)  # rate_error > 0: sender runs fast
        self.ui.set(ui_bit, 1)
        self.t = 0.0  # fractional cycle position, so long streams keep the exact rate
        self.n = 0

    async def _hold(self, bits):
        self.t += bits * self.bit
        k = round(self.t) - self.n
        self.n += k
        if k:
            await ClockCycles(self.dut.clk, k)

    async def send(self, data, gap_bits=0, stop=1):
        for b in data:
            frame = [0] + [(b >> i) & 1 for i in range(8)] + [stop]
            for v in frame:
                self.ui.set(self.ui_bit, v)
                await self._hold(1)
            self.ui.set(self.ui_bit, 1)
            if gap_bits:
                await self._hold(gap_bits)
