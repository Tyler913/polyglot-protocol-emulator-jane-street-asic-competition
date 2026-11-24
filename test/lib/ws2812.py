# Copyright (c) 2026 Tyler Hong
# SPDX-License-Identifier: Apache-2.0
"""Independent WS2812 receiver model, from the WS2812B datasheet: each bit
is one high pulse; T0H = 0.4 us +/- 0.15 us, T1H = 0.8 us +/- 0.15 us,
bit period 1.25 us +/- 0.6 us; low for > 50 us latches. 24 bits per LED,
G-R-B, MSB first."""

import cocotb
from cocotb.triggers import FallingEdge
from .pins import bit


class Ws2812Model:
    def __init__(self, dut, uo_bit, clk_ns):
        self.dut, self.bit, self.ns = dut, uo_bit, clk_ns
        self.bits = []
        self.frames = []
        self.errors = []

    def start(self):
        self._task = cocotb.start_soon(self._run())

    def stop(self):
        self._task.cancel()

    async def _run(self):
        t = 0
        level, rise, last_rise = 0, None, None
        while True:
            await FallingEdge(self.dut.clk)
            t += 1
            v = bit(self.dut.uo_out, self.bit)
            if v and not level:
                if last_rise is not None and self.bits:
                    period = (t - last_rise) * self.ns
                    if not 650 <= period <= 1850:
                        self.errors.append(f"bit period {period:.0f} ns")
                rise, last_rise = t, t
            elif level and not v:
                high = (t - rise) * self.ns
                if 250 <= high <= 550:
                    self.bits.append(0)
                elif 650 <= high <= 950:
                    self.bits.append(1)
                else:
                    self.errors.append(f"high pulse of {high:.0f} ns")
            elif not v and self.bits and (t - last_rise) * self.ns > 50_000:
                self.frames.append(self.bits)
                self.bits = []
            level = v

    def leds(self, frame):
        return [tuple(sum(frame[24 * i + 8 * c + j] << (7 - j) for j in range(8)) for c in range(3))
                for i in range(len(frame) // 24)]
