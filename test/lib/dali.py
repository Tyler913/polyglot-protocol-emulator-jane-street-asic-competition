# Copyright (c) 2026 Tyler Hong
# SPDX-License-Identifier: Apache-2.0
"""Independent DALI control gear, written from IEC 62386-101/102 (1200 bit/s
bi-phase, Te = 416.67 us +/- 10 %, bus idle high, a 1 is low then high;
forward frame = start bit + 16 bits MSB first, backward frame = start bit
+ 8 bits, sent 7..22 Te after a query; address byte 0AAAAAAS, S = 0 direct
arc power, S = 1 command; 0xA0 = QUERY ACTUAL LEVEL), not from the RTL.

Event driven (edges of the chip's TX), so the slow bus costs little
simulation time. The bus is the wired AND of the chip's TX and the gear;
the chip's RX pin follows the bus."""

import math

import cocotb
from cocotb.triggers import First, Timer
from cocotb.utils import get_sim_time

from .pins import bit, ui_pins

TE_NS = 1e9 / 2400


class DaliGear:
    def __init__(self, dut, tx_uo, rx_ui, address, reply_te=10):
        self.dut, self.tx, self.rx, self.addr = dut, tx_uo, rx_ui, address
        self.ui = ui_pins(dut)
        self.gear = 1
        self.level = 0
        self.reply_te = reply_te
        self.frames, self.errors = [], []
        self.ui.set(rx_ui, 1)

    def start(self):
        self._tasks = [cocotb.start_soon(self._wire()), cocotb.start_soon(self._run())]

    def stop(self):
        for t in self._tasks:
            t.cancel()

    async def _wire(self):
        """The bus: the chip's RX pin follows chip TX AND gear, at every chip edge."""
        while True:
            self._drive()
            await self.dut.uo_out.value_change

    def _chip(self):
        try:
            return bit(self.dut.uo_out, self.tx)
        except ValueError:
            return 1

    def _drive(self):
        self.ui.set(self.rx, self._chip() & self.gear)

    async def _edges(self):
        """Edges of the chip's TX for one frame: (time_ns, level) until 4 Te of quiet."""
        while self._chip() == 1:
            await self.dut.uo_out.value_change
            self._drive()
        ev = [(get_sim_time("ns"), 0)]
        while True:
            lv = self._chip()
            r = await First(self.dut.uo_out.value_change, Timer(round(4 * TE_NS), unit="ns"))
            self._drive()
            if isinstance(r, Timer):
                return ev
            if self._chip() != lv:
                ev.append((get_sim_time("ns"), self._chip()))

    def _decode(self, ev):
        t0 = ev[0][0]

        def level_at(t):
            v = 1
            for te, lv in ev:
                if te - t0 <= t:
                    v = lv
            return v

        for te, _ in ev:  # every edge on the Te grid, +/- 10 %
            k = (te - t0) / TE_NS
            if abs(k - round(k)) > 0.1:
                self.errors.append(f"edge at {k:.2f} Te, off the grid")
        bits = []
        # the last edge is the last bit's mid-bit transition (a 1) or its end (a 0, back to idle)
        n = math.ceil((ev[-1][0] - t0) / (2 * TE_NS) - 0.25)
        for i in range(n):
            a, b = level_at((2 * i + 0.5) * TE_NS), level_at((2 * i + 1.5) * TE_NS)
            if a == b:
                self.errors.append(f"bit {i}: not bi-phase")
            bits.append(b)
        return bits

    async def _send(self, byte):
        bits = [1] + [(byte >> (7 - i)) & 1 for i in range(8)]
        for b in bits:
            for v in (1 - b, b):
                self.gear = v
                self._drive()
                await Timer(round(TE_NS), unit="ns")
        self.gear = 1
        self._drive()

    async def _run(self):
        # Until the chip configures its TX pin the line is low (reset): wait for 4 Te idle high
        while True:
            while self._chip() == 0:
                await self.dut.uo_out.value_change
                self._drive()
            r = await First(self.dut.uo_out.value_change, Timer(round(4 * TE_NS), unit="ns"))
            self._drive()
            if isinstance(r, Timer) and self._chip() == 1:
                break
        while True:
            bits = self._decode(await self._edges())
            if len(bits) != 17 or bits[0] != 1:
                self.errors.append(f"forward frame of {len(bits)} bits {bits} at {get_sim_time('ns'):.0f} ns")
                continue
            a = sum(b << (15 - i) for i, b in enumerate(bits[1:17]))
            adr, data = a >> 8, a & 0xFF
            self.frames.append((adr, data))
            if (adr >> 1) & 0x3F != self.addr:
                continue
            if adr & 1 == 0:
                self.level = data  # direct arc power
            elif data == 0xA0:
                # the edge search waited 4 Te after the last edge (half a bit + stop)
                await Timer(round((self.reply_te - 4) * TE_NS), unit="ns")
                await self._send(self.level)
