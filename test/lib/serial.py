# Copyright (c) 2026 Tyler Hong
# SPDX-License-Identifier: Apache-2.0
"""Independent asynchronous serial (UART-style) frame model with any common
format, written from the frame definition, not from the RTL: one start bit,
5..9 data bits LSB first, optional even/odd parity, 1 or 2 stop bits, and an
optional inverted line (idle low, as SBUS and SDI-12 use).

Fmt("8E2", invert=True) describes SBUS; Fmt("7E1", invert=True) SDI-12.
SerialRx decodes a line by sampling the middle of each bit, checks parity and
stop bits, and records (value, start cycle) per frame; a frame that is all
"space" including the stop bits is recorded as a BREAK (value None).
SerialTx drives frames onto a ui input pin, optionally with a rate error.
Time is counted in system clock cycles; bit periods may be fractional.
"""

import cocotb
from cocotb.triggers import ClockCycles, ReadOnly, RisingEdge


class Fmt:
    def __init__(self, spec="8N1", invert=False):
        self.data = int(spec[0])
        self.parity = spec[1]          # "N", "E" or "O"
        self.stop = int(spec[2])
        self.invert = invert
        assert 5 <= self.data <= 9 and self.parity in "NEO" and self.stop in (1, 2)

    @property
    def bits(self):
        """Bits per frame, start and stop bits included."""
        return 1 + self.data + (self.parity != "N") + self.stop

    def parity_bit(self, v):
        ones = bin(v & ((1 << self.data) - 1)).count("1")
        return ones & 1 if self.parity == "E" else 1 - (ones & 1)

    def logic_bits(self, v):
        """Frame bits in logic levels (1 = mark), start bit first."""
        b = [0] + [(v >> i) & 1 for i in range(self.data)]
        if self.parity != "N":
            b.append(self.parity_bit(v))
        return b + [1] * self.stop

    def line(self, logic):
        return 1 - logic if self.invert else logic


class SerialRx:
    def __init__(self, dut, get_line, bit, fmt, strict=True):
        self.dut, self.get_line, self.bit, self.fmt, self.strict = dut, get_line, bit, fmt, strict
        self.frames = []   # (value or None for a BREAK, start cycle)
        self.ok = []       # per frame: parity and stop bits good
        self.errors = []
        self.t = 0
        self._task = None

    @property
    def values(self):
        return [v for v, _ in self.frames if v is not None]

    def start(self):
        self._task = cocotb.start_soon(self._run())

    def stop(self):
        if self._task:
            self._task.cancel()

    async def _sample(self):
        await RisingEdge(self.dut.clk)
        await ReadOnly()
        self.t += 1
        return self.fmt.line(self.get_line())   # logic level

    async def _run(self):
        last = 0
        while True:
            while True:                           # mark -> space edge after idle mark
                v = await self._sample()
                if last == 1 and v == 0:
                    break
                last = v
            start = self.t
            n = self.fmt.bits
            levels = {}
            edges = []
            prev = 0
            while self.t - start < (n - 0.5) * self.bit:
                v = await self._sample()
                if v != prev:
                    edges.append(self.t - start)
                    prev = v
                for k in range(n):
                    if k not in levels and self.t - start >= round((k + 0.5) * self.bit):
                        levels[k] = v
            bits = [levels[k] for k in range(n)]
            if self.strict:
                for e in edges:
                    k = round(e / self.bit)
                    if abs(e - k * self.bit) > 1.5 and k < n:
                        self.errors.append(f"edge at {e} cycles, off the bit grid")
            last = bits[-1]
            if not any(bits):
                self.frames.append((None, start))        # BREAK (or a long space)
                self.ok.append(False)
                continue
            if bits[0] != 0:
                self.errors.append("start bit not space at its middle")
                continue
            d = self.fmt.data
            val = sum(bits[1 + i] << i for i in range(d))
            p = 1 + d
            good = True
            if self.fmt.parity != "N":
                if bits[p] != self.fmt.parity_bit(val):
                    self.errors.append(f"parity error on {val:#x}")
                    good = False
                p += 1
            if not all(bits[p:]):
                self.errors.append(f"stop bit missing on {val:#x}")
                good = False
            self.frames.append((val, start))
            self.ok.append(good)


class SerialTx:
    """Drives frames on ui_in[pin]. The line is set to idle at once."""

    def __init__(self, dut, ui, pin, bit, fmt):
        self.dut, self.ui, self.pin, self.bit, self.fmt = dut, ui, pin, bit, fmt
        self.ui.set(pin, fmt.line(1))

    async def level(self, logic, cycles):
        self.ui.set(self.pin, self.fmt.line(logic))
        await ClockCycles(self.dut.clk, max(1, round(cycles)))

    async def send(self, values, gap_bits=0, bad_parity=(), rate=1.0):
        """Send frames; gap_bits of idle after each; frames whose index is in
        bad_parity get the wrong parity bit; rate scales the bit period."""
        bit = self.bit * rate
        for i, v in enumerate(values):
            bits = self.fmt.logic_bits(v)
            if i in bad_parity and self.fmt.parity != "N":
                bits[1 + self.fmt.data] ^= 1
            t = 0.0
            for b in bits:
                self.ui.set(self.pin, self.fmt.line(b))
                nxt = t + bit
                await ClockCycles(self.dut.clk, round(nxt) - round(t))
                t = nxt
            if gap_bits:
                await ClockCycles(self.dut.clk, round(gap_bits * bit))

    async def brk(self, bits):
        """Space (logic 0) for the given number of bit times, then mark."""
        await self.level(0, bits * self.bit)
        self.ui.set(self.pin, self.fmt.line(1))
