# Copyright (c) 2026 Tyler Hong
# SPDX-License-Identifier: Apache-2.0
"""Independent 10BASE-T auto-negotiation link partner (IEEE 802.3 clause
28), written from the standard, not from the RTL or the C code:

- FLP burst: 17 clock pulses 125 us apart (111..139 us allowed), a data
  pulse 62.5 us after a clock pulse (55.5..69.5 us) for each 1 bit of the
  16-bit Link Code Word (bit 0 first), pulses 100 ns wide; bursts 8..24 ms
  apart (16 ms nominal).
- Arbitration (simplified): three identical words (Acknowledge bit 14
  ignored) = ability match, then our words carry Ack; complete after three
  received words with Ack. The highest common mode: 10BASE-T full duplex
  (bit 6) before half duplex (bit 5).
The chip's bursts are taken from rising edges of TD+ (uio, driven high:
output enable and value), ours go to RD
(ui). The model reacts to pin edges only, so long runs stay fast."""

import random

import cocotb
from cocotb.triggers import Edge, First, Timer
from cocotb.utils import get_sim_time

from .pins import ui_pins

ACK = 1 << 14


def burst_times(word, t0_us=0.0):
    """Pulse start times (us) of one burst."""
    t = []
    for k in range(33):
        if k % 2 == 0 or (word >> (k // 2)) & 1:
            t.append(t0_us + 62.5 * k)
    return t


class AutonegPartner:
    def __init__(self, dut, tdp_uio, rd_ui, ability, seed=1):
        self.dut, self.tdp, self.rd = dut, tdp_uio, rd_ui
        self.ability = 0x0001 | ability
        self.ui = ui_pins(dut)
        self.ui.set(rd_ui, 0)
        self.rng = random.Random(seed)
        self.errors, self.words, self.bursts = [], [], []
        self.match = False
        self.acks = 0
        self.result = None

    def start(self):
        self._tasks = [cocotb.start_soon(self._rx()), cocotb.start_soon(self._tx())]

    def stop(self):
        for t in self._tasks:
            t.cancel()

    def _decode(self, times, widths):
        if any(not (80 <= w <= 120) for w in widths):
            self.errors.append(f"pulse widths {sorted(set(round(w) for w in widths))} ns")
        word, bit, i = 0, 0, 1
        while i < len(times) and bit < 16:
            d = times[i] - times[i - 1]
            if 55.5 <= d <= 69.5:                   # data pulse: 1
                if i + 1 >= len(times) or not (55.5 <= times[i + 1] - times[i] <= 69.5):
                    self.errors.append("data pulse not followed by a clock pulse 62.5 us later")
                    return None
                word |= 1 << bit
                i += 2
            elif 111 <= d <= 139:                    # next clock pulse: 0
                i += 1
            else:
                self.errors.append(f"pulse spacing {d:.1f} us")
                return None
            bit += 1
        if bit != 16 or i != len(times):
            self.errors.append(f"burst of {len(times)} pulses")
            return None
        return word

    async def _rx(self):
        times, widths, rise = [], [], None
        bit_mask = 1 << self.tdp
        prev = 0
        while True:
            await First(Edge(self.dut.uio_out), Edge(self.dut.uio_oe))
            now = get_sim_time("ns") / 1000.0         # us
            v = 1 if int(self.dut.uio_out.value) & int(self.dut.uio_oe.value) & bit_mask else 0
            if v == prev:
                continue
            prev = v
            if v:
                if times and now - times[-1] > 200:   # new burst: finish the last one
                    self._burst_done(times, widths)
                    times, widths = [], []
                rise = now
                times.append(now)
            elif rise is not None:
                widths.append((now - rise) * 1000)

    def _burst_done(self, times, widths):
        if self.bursts and not (8000 <= times[0] - self.bursts[-1] <= 24000):
            self.errors.append(f"burst interval {(times[0] - self.bursts[-1]) / 1000:.2f} ms")
        self.bursts.append(times[0])
        w = self._decode(times, widths)
        if w is None:
            return
        self.words.append(w)
        base = [x & ~ACK for x in self.words[-3:]]
        if len(base) == 3 and len(set(base)) == 1:
            self.match = True
        if self.match and w & ACK:
            self.acks += 1
            if self.acks >= 3 and self.result is None:
                common = self.ability & w
                self.result = 2 if common & 0x40 else 1 if common & 0x20 else 0

    async def _tx(self):
        await Timer(round(self.rng.uniform(3000, 9000) * 1000), unit="ns")
        while True:
            word = self.ability | (ACK if self.match else 0)
            t = 0.0
            for p in burst_times(word):
                if round((p - t) * 1000) > 0:
                    await Timer(round((p - t) * 1000), unit="ns")
                self.ui.set(self.rd, 1)
                await Timer(100, unit="ns")
                self.ui.set(self.rd, 0)
                t = p + 0.1
            await Timer(round((16000 - t) * 1000), unit="ns")
