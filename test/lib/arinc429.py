# Copyright (c) 2026 Tyler Hong
# SPDX-License-Identifier: Apache-2.0
"""Independent ARINC 429 line model, written from the ARINC 429 word and
signal definition, not from the RTL: 32-bit words at 100 kbit/s (high
speed), bipolar return-to-zero (HI half bit = 1, LO half bit = 0, then NULL),
at least 4 bit times of NULL between words, odd parity in bit 32, label in
bits 1..8 transmitted MSB first. Words are ints with bit 1 in the LSB.

Rx429 watches the chip's DATA A / DATA B outputs (A = HI, B = LO, both low
= NULL; both high is an error) and checks half-bit and bit timing (+/- 2 %),
the gap and the parity. Tx429 drives the receiver's HI and LO outputs."""

import cocotb
from cocotb.triggers import ClockCycles, ReadOnly, RisingEdge

from .pins import bit, ui_pins

BIT = 500  # cycles at 50 MHz


def word(label_octal, sdi, data19, ssm, bad_parity=False):
    lab = int(str(label_octal), 8)
    w = sum(((lab >> (7 - i)) & 1) << i for i in range(8))     # bit 1 = label MSB
    w |= (sdi & 3) << 8 | (data19 & 0x7FFFF) << 10 | (ssm & 3) << 29
    p = 1 - (bin(w).count("1") & 1)                            # odd parity
    return w | ((p ^ (1 if bad_parity else 0)) << 31)


class Rx429:
    def __init__(self, dut, a_uo, b_uo):
        self.dut, self.a, self.b = dut, a_uo, b_uo
        self.words, self.errors, self.starts = [], [], []

    def start(self):
        self._task = cocotb.start_soon(self._run())

    def stop(self):
        self._task.cancel()

    async def _run(self):
        t, prev, cur, n, w, t_on, both = 0, "N", None, 0, 0, 0, 0
        while True:
            await RisingEdge(self.dut.clk)
            await ReadOnly()
            t += 1
            a, b = bit(self.dut.uo_out, self.a), bit(self.dut.uo_out, self.b)
            both = both + 1 if a and b else 0
            if both > 2:
                self.errors.append(f"A and B both high at {t}")
            s = "H" if a and not b else "L" if b and not a else "N" if not a and not b else prev
            if s != prev:
                if s in "HL":                                  # a bit starts
                    if n == 0:
                        if self.starts and t - self.starts[-1] < 36 * BIT:
                            self.errors.append(f"gap before word: {(t - self.starts[-1]) / BIT - 32:.2f} bits")
                        self.starts.append(t)
                    elif abs((t - t_on) - BIT) > 0.02 * BIT:
                        self.errors.append(f"bit period {t - t_on} cycles")
                    t_on = t
                    w |= (1 if s == "H" else 0) << n
                    n += 1
                elif prev in "HL":                             # half bit over
                    if abs((t - t_on) - BIT / 2) > 0.02 * BIT:
                        self.errors.append(f"half bit {t - t_on} cycles")
                    if n == 32:
                        if bin(w).count("1") % 2 != 1:
                            self.errors.append(f"parity {w:#x}")
                        self.words.append(w)
                        n, w = 0, 0
                prev = s


class Tx429:
    def __init__(self, dut, hi_ui, lo_ui):
        self.dut, self.hi, self.lo = dut, hi_ui, lo_ui
        self.ui = ui_pins(dut)
        self.ui.set(hi_ui, 0)
        self.ui.set(lo_ui, 0)

    async def send(self, words, gap_bits=4):
        for w in words:
            for i in range(32):
                pin = self.hi if (w >> i) & 1 else self.lo
                self.ui.set(pin, 1)
                await ClockCycles(self.dut.clk, BIT // 2)
                self.ui.set(pin, 0)
                await ClockCycles(self.dut.clk, BIT // 2)
            await ClockCycles(self.dut.clk, gap_bits * BIT)
