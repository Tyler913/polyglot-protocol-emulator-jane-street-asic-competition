# Copyright (c) 2026 Tyler Hong
# SPDX-License-Identifier: Apache-2.0
"""Independent HDMI-CEC follower, written from the HDMI 1.4b CEC timing
(start bit: low 3.5-3.9 ms, period 4.3-4.7 ms; data bit: period 2.05-2.75 ms,
low 0.4-0.8 ms = 1, 1.3-1.7 ms = 0; a follower acknowledges a block by
holding the ACK bit low for 1.5 ms; header = initiator << 4 | destination,
blocks of 8 bits + EOM + ACK), not from the RTL.

The line is a wired AND with a pull-up: the chip pulls it low when its
output enable is set (its output value is 0), the follower when it ACKs.
Event driven on uio_oe/uio_out changes."""

import cocotb
from cocotb.triggers import First, Timer
from cocotb.utils import get_sim_time


class CecFollower:
    def __init__(self, dut, pin, address):
        self.dut, self.pin, self.addr = dut, pin, address
        self.low = False  # follower pulling
        self.frames, self.errors = [], []
        self.edges = []  # (time_ms, level) of the resolved line

    def _chip_low(self):
        oe, out = int(self.dut.uio_oe.value), int(self.dut.uio_out.value)
        return (oe >> self.pin) & 1 and not (out >> self.pin) & 1

    def _update(self):
        v = 0 if (self._chip_low() or self.low) else 1
        cur = int(self.dut.uio_in.value)
        self.dut.uio_in.value = (cur & ~(1 << self.pin)) | (v << self.pin)
        if not self.edges or self.edges[-1][1] != v:
            self.edges.append((get_sim_time("ns") / 1e6, v))
        return v

    def start(self):
        self._update()
        self._tasks = [cocotb.start_soon(self._wire())]

    def stop(self):
        for t in self._tasks:
            t.cancel()

    async def _wire(self):
        n_seen = 0
        while True:
            await First(self.dut.uio_oe.value_change, self.dut.uio_out.value_change)
            self._update()
            # a falling edge of the line starts a bit
            if len(self.edges) > n_seen:
                n_seen = len(self.edges)
                if self.edges[-1][1] == 0 and self._ack_due():
                    cocotb.start_soon(self._ack())

    async def _ack(self):
        self.low = True
        self._update()
        await Timer(1500, unit="us")
        self.low = False
        self._update()

    def _ack_due(self):
        """Is the bit that just started an ACK bit of a block addressed to us?"""
        bits, block, header = self._bits_so_far()
        if bits is None:
            return False
        return len(bits) % 10 == 9 and header is not None and (header & 0xF) == self.addr

    def _bits_so_far(self):
        """Decode the current frame from the edges seen up to the last falling edge."""
        falls = [t for t, v in self.edges if v == 0]
        rises = [t for t, v in self.edges if v == 1]
        # find the last start bit
        start = None
        for i, f in enumerate(falls[:-1]):
            r = next((x for x in rises if x > f), None)
            if r is not None and 3.5 <= r - f <= 3.9 and 4.3 <= falls[i + 1] - f <= 4.7:
                start = i
        if start is None:
            return None, None, None
        bits = []
        for i in range(start + 1, len(falls) - 1):
            f, nf = falls[i], falls[i + 1]
            r = next((x for x in rises if x > f), None)
            low = r - f
            bits.append(1 if low < 1.05 else 0)
        header = None
        if len(bits) >= 8:
            header = sum(b << (7 - k) for k, b in enumerate(bits[:8]))
        return bits, None, header

    def decode(self):
        """Whole frames from the recorded edges: list of (bytes, acks)."""
        falls = [t for t, v in self.edges if v == 0]
        rises = [t for t, v in self.edges if v == 1]
        frames, cur = [], None
        for i, f in enumerate(falls):
            r = next((x for x in rises if x > f), None)
            if r is None:
                break
            low = r - f
            period = falls[i + 1] - f if i + 1 < len(falls) else None
            if 3.5 <= low <= 3.9:
                if period is not None and not 4.3 <= period <= 4.7:
                    self.errors.append(f"start bit period {period:.3f} ms")
                cur = []
                frames.append(cur)
                continue
            if cur is None:
                continue
            if period is not None and period < 4.0 and not 2.05 <= period <= 2.75:
                self.errors.append(f"bit period {period:.3f} ms")
            if 0.4 <= low <= 0.8:
                cur.append(1)
            elif 1.3 <= low <= 1.7:
                cur.append(0)
            else:
                self.errors.append(f"low time {low:.3f} ms")
        out = []
        for bits in frames:
            if len(bits) % 10:
                self.errors.append(f"frame of {len(bits)} bits")
                continue
            data, acks = [], []
            for k in range(0, len(bits), 10):
                data.append(sum(b << (7 - j) for j, b in enumerate(bits[k : k + 8])))
                eom, ack = bits[k + 8], bits[k + 9] == 0
                last = k + 10 == len(bits)
                if eom and not last:
                    self.errors.append(f"data after EOM in block {k // 10}")
                if last and not eom and ack:
                    self.errors.append("frame ends without EOM although the block was ACKed")
                acks.append(ack)
            out.append((bytes(data), acks))
        return out
