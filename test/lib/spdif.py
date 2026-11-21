# Copyright (c) 2026 Tyler Hong
# SPDX-License-Identifier: Apache-2.0
"""Independent S/PDIF (IEC 60958, consumer; AES3 has the same line format)
model, written from the standard's description, not from our RTL.

A frame has two subframes (channel A, channel B) of 32 time slots. Slots 0..3
are the preamble, slots 4..31 carry 4 auxiliary bits, 20 audio bits (LSB
first; 24-bit audio uses slots 4..27), V, U, C and P (even parity over slots
4..31). Every slot is biphase-mark coded: a transition at the start of each
slot and another one in its middle for a 1. Preambles break that rule; as
half-slot levels after a low line they are B = 11101000 (channel A, first
frame of a 192-frame block), M = 11100010 (channel A), W = 11100100
(channel B), inverted after a high line.

SpdifSource drives a pin; SpdifSink samples one and decodes subframes from
the edge intervals.
"""

import cocotb
from cocotb.triggers import FallingEdge

from .pins import bit

PRE = {"B": [1, 1, 1, 0, 1, 0, 0, 0], "M": [1, 1, 1, 0, 0, 0, 1, 0], "W": [1, 1, 1, 0, 0, 1, 0, 0]}
TYPE = {"B": 1, "M": 2, "W": 3}


def payload(sample24, v=0, u=0, c=0):
    """Slots 4..31 as a 28-bit value, slot 4 in bit 0, parity included."""
    x = (sample24 & 0xFFFFFF) | (v << 24) | (u << 25) | (c << 26)
    p = bin(x).count("1") & 1
    return x | (p << 27)


def subframe_halves(pre, x28, level):
    """Half-slot levels of one subframe after a line at `level`."""
    out = list(PRE[pre]) if level == 0 else [1 - h for h in PRE[pre]]
    lvl = out[-1]
    for i in range(28):
        lvl ^= 1
        out.append(lvl)
        if (x28 >> i) & 1:
            lvl ^= 1
        out.append(lvl)
    return out


def stream_halves(subframes):
    """subframes: list of (preamble, x28). Returns the half-slot levels."""
    out, lvl = [], 0
    for pre, x in subframes:
        h = subframe_halves(pre, x, lvl)
        out += h
        lvl = h[-1]
    return out


class SpdifSource:
    """Drives ui[pin] with the half-slot levels, half = cycles per half slot
    (fractional; the edges land on the nearest clock)."""

    def __init__(self, dut, ui, pin, half):
        self.dut, self.ui, self.pin, self.half = dut, ui, pin, half

    async def play(self, halves, jitter=None):
        t = 0
        for i, h in enumerate(halves):
            self.ui.set(self.pin, h)
            n = round((i + 1) * self.half) - round(i * self.half)
            if jitter:
                n += jitter(i)
            for _ in range(n):
                await FallingEdge(self.dut.clk)
            t += n
        return t


class SpdifSink:
    """Samples uio_out[pin] (driven when uio_oe is set) every clock and
    decodes subframes: list of (preamble letter, x28, parity ok)."""

    def __init__(self, dut, pin, half):
        self.dut, self.pin, self.half = dut, pin, half
        self.intervals = []
        self.errors = []

    def start(self):
        self._task = cocotb.start_soon(self._run())

    def stop(self):
        self._task.cancel()

    async def _run(self):
        prev, t, last = None, 0, None
        while True:
            await FallingEdge(self.dut.clk)
            t += 1
            lvl = bit(self.dut.uio_out, self.pin) if bit(self.dut.uio_oe, self.pin) else None
            if lvl is None:
                prev = None
                continue
            if prev is not None and lvl != prev:
                if last is not None:
                    self.intervals.append((t - last, prev))
                last = t
            prev = lvl

    def decode(self):
        """Half-slot levels from the measured intervals, then subframes."""
        halves = []
        for n, lvl in self.intervals:
            k = n / self.half
            q = round(k)
            if q < 1 or q > 3 or abs(k - q) > 0.3:
                self.errors.append(f"interval {n} cycles = {k:.2f} half slots")
                q = max(1, min(3, q))
            halves += [lvl] * q
        subs, i = [], 0
        while i + 64 <= len(halves):
            if not (halves[i] == halves[i + 1] == halves[i + 2] != halves[i + 3]):
                i += 1
                continue
            p = halves[i:i + 8]
            if p[0] == 0:
                p = [1 - h for h in p]
            pre = next((k for k, v in PRE.items() if v == p), None)
            if pre is None:
                self.errors.append(f"bad preamble {p}")
                i += 1
                continue
            x = 0
            for s in range(28):
                a, b = halves[i + 8 + 2 * s], halves[i + 9 + 2 * s]
                before = halves[i + 7 + 2 * s]
                if a == before:
                    self.errors.append("no transition at a slot start")
                x |= (a != b) << s
            subs.append((pre, x, bin(x).count("1") % 2 == 0))
            i += 64
        return subs
