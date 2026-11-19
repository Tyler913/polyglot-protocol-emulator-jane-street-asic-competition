# Copyright (c) 2026 Tyler Hong
# SPDX-License-Identifier: Apache-2.0
"""Independent BiSS-C encoder (unidirectional), written from the published
BiSS-C frame description (iC-Haus; RLS application note), not from the RTL:
MA idles high, SL idles high. A frame starts with the first falling MA edge
after MA was high for the timeout; on the next rising edge the encoder
latches the position and answers, one bit per rising MA edge: Ack
(low, ack_bits long), Start (1), CDS (0), position MSB first, nE, nW, CRC-6
(x^6 + x + 1, init 0, over position + nE + nW, inverted). Then SL stays low
until MA has been high for the timeout, and goes high (ready).
`delay` adds cable/converter delay to SL, in clock cycles."""

import cocotb
from cocotb.triggers import ClockCycles, ReadOnly, RisingEdge

from .pins import bit, ui_pins


def crc6(bits):
    c = 0
    for b in bits:
        fb = b ^ (c >> 5)
        c = (c << 1) & 0x3F
        if fb:
            c ^= 0x03
    return c


def frame_bits(pos, nbits, ne=1, nw=1, ack_bits=2, bad_crc=False):
    data = [(pos >> (nbits - 1 - i)) & 1 for i in range(nbits)] + [ne, nw]
    c = crc6(data) ^ 0x3F ^ (1 if bad_crc else 0)
    return [0] * ack_bits + [1, 0] + data + [(c >> (5 - i)) & 1 for i in range(6)]


class BissEncoder:
    def __init__(self, dut, ma_uo, sl_ui, nbits, timeout=50, delay=7):
        self.dut, self.ma, self.sl = dut, ma_uo, sl_ui
        self.nbits, self.timeout, self.delay = nbits, timeout, delay
        self.ui = ui_pins(dut)
        self.queue = []          # (pos, ne, nw, bad_crc) for the next frames
        self.served = []
        self.errors = []
        self.max_gap = 0
        self.rises = []          # rising MA edges inside frames
        self.ui.set(sl_ui, 1)

    def start(self):
        self._task = cocotb.start_soon(self._run())

    def stop(self):
        self._task.cancel()

    async def _run(self):
        prev, t, high_since, bits, k, pending = 1, 0, 0, None, 0, []
        last_rise = None
        sl = 1
        armed = False
        while True:
            await RisingEdge(self.dut.clk)
            await ReadOnly()
            t += 1
            ma = bit(self.dut.uo_out, self.ma)
            if prev and not ma and bits is None and high_since and t - high_since >= self.timeout:
                armed = True                        # first falling edge after idle: a frame starts
            if ma and not prev:                     # rising MA edge
                if bits is None:
                    if armed and sl == 1 and self.queue:
                        armed = False
                        p = self.queue.pop(0)
                        bits = frame_bits(p[0], self.nbits, p[1], p[2], bad_crc=p[3])
                        self.served.append(p)
                        k = 0
                if bits is not None:
                    if last_rise is not None:
                        self.max_gap = max(self.max_gap, t - last_rise)
                    last_rise = t
                    self.rises.append(t)
                    v = bits[k] if k < len(bits) else 0   # after the CRC: low until timeout
                    k += 1
                    pending.append((t + self.delay, v))
            if ma:
                high_since = high_since or t
            else:
                high_since = 0
            if bits is not None and k >= len(bits) and high_since and t - high_since >= self.timeout:
                bits, last_rise = None, None
                pending.append((t + self.delay, 1))       # ready
            prev = ma
            await RisingEdge(self.dut.clk)
            t += 1
            while pending and pending[0][0] <= t:
                sl = pending.pop(0)[1]
                self.ui.set(self.sl, sl)
            if bits is not None and k < len(bits) and high_since and t - high_since > self.timeout:
                self.errors.append("MA stopped in the middle of a frame")
                bits = None
