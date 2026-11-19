# Copyright (c) 2026 Tyler Hong
# SPDX-License-Identifier: Apache-2.0
"""Independent PDM MEMS microphone model: a second-order sigma-delta
modulator (one bit per microphone clock) fed with a sine; the microphone
with SELECT low changes DATA after each falling clock edge, so it is stable
when the host samples on the rising edge. Not derived from the RTL."""

import math

import cocotb
from cocotb.triggers import ReadOnly, RisingEdge

from .pins import bit, ui_pins


class PdmMic:
    def __init__(self, dut, clk_uo, data_ui, freq, amp, f_sys=50e6):
        self.dut, self.clk, self.data = dut, clk_uo, data_ui
        self.freq, self.amp, self.f_sys = freq, amp, f_sys
        self.ui = ui_pins(dut)
        self.v1 = self.v2 = 0.0
        self.y = 1.0
        self.bits = 0
        self.edges = []

    def start(self):
        self._task = cocotb.start_soon(self._run())

    def stop(self):
        self._task.cancel()

    async def _run(self):
        prev, t = 0, 0
        while True:
            await RisingEdge(self.dut.clk)
            await ReadOnly()
            t += 1
            c = bit(self.dut.uo_out, self.clk)
            fall = prev == 1 and c == 0
            if c and not prev:
                self.edges.append(t)
            prev = c
            if fall:
                x = self.amp * math.sin(2 * math.pi * self.freq * t / self.f_sys)
                self.v1 += x - self.y
                self.v2 += self.v1 - self.y
                self.y = 1.0 if self.v2 >= 0 else -1.0
                self.bits += 1
                await RisingEdge(self.dut.clk)
                t += 1
                self.ui.set(self.data, 1 if self.y > 0 else 0)
