# Copyright (c) 2026 Tyler Hong
# SPDX-License-Identifier: Apache-2.0
"""Independent 10BASE-T transmit decoder, written from IEEE 802.3 clause 14
(Manchester: in each 100 ns bit the first half is the complement of the bit
and the second half the bit; differential TD+/TD-) and clause 3 (preamble,
SFD 0xD5, FCS = CRC-32, checked with zlib), not from the RTL. After the
last bit the transmitter holds TD+ high (TP_IDL) for tail_bits bit times.
"""

import zlib

import cocotb
from cocotb.triggers import FallingEdge


class EthTxMonitor:
    def __init__(self, dut, tdp, tdn, half, tail_bits=0):
        self.dut, self.tdp, self.tdn, self.half = dut, tdp, tdn, half
        self.tail = tail_bits * 2 * half
        self.frames = []  # (bytes incl. FCS, fcs_ok)
        self.link_pulses = []  # (start cycle, cycles high) of bursts shorter than a byte: NLPs
        self.errors = []
        self._levels = []

    def start(self):
        self._task = cocotb.start_soon(self._run())

    def stop(self):
        self._task.cancel()

    async def _run(self):
        t = 0
        while True:
            await FallingEdge(self.dut.clk)
            t += 1
            out, oe = int(self.dut.uio_out.value), int(self.dut.uio_oe.value)
            p_on, n_on = (oe >> self.tdp) & 1, (oe >> self.tdn) & 1
            if p_on != n_on:
                self.errors.append(f"t={t}: only one of TD+/TD- driven")
            if p_on:
                p, n = (out >> self.tdp) & 1, (out >> self.tdn) & 1
                if p == n:
                    self.errors.append(f"t={t}: TD+ == TD- while driven")
                self._levels.append(p)
            elif self._levels:
                if len(self._levels) < 8 * 2 * self.half:
                    self.link_pulses.append((t - len(self._levels), sum(self._levels)))
                else:
                    self._decode(self._levels)
                self._levels = []

    def _decode(self, lv):
        h = self.half
        if self.tail:
            if lv[-self.tail:] != [1] * self.tail:
                self.errors.append("no TP_IDL (TD+ high) after the frame")
            lv = lv[: -self.tail]
        if len(lv) % (2 * h):
            self.errors.append(f"burst of {len(lv)} cycles is not a whole number of bits")
        bits = []
        for i in range(0, len(lv) - 2 * h + 1, 2 * h):
            first, second = lv[i : i + h], lv[i + h : i + 2 * h]
            if len(set(first)) != 1 or len(set(second)) != 1 or first[0] == second[0]:
                self.errors.append(f"bit {i // (2 * h)}: not a Manchester symbol {first + second}")
            bits.append(second[0])
        # preamble 1010... then SFD ...11
        s = "".join(map(str, bits))
        k = s.find("10101011")
        if k < 0 or k + 8 > 64 + 1:
            self.errors.append("no SFD")
            return
        if not set(s[:k]) <= {"0", "1"} or s[:k] != ("10" * 32)[:k]:
            self.errors.append("preamble is not 1010...")
        body = bits[k + 8 :]
        if len(body) % 8:
            self.errors.append(f"{len(body)} bits after the SFD")
        data = bytes(sum(body[8 * j + i] << i for i in range(8)) for j in range(len(body) // 8))
        ok = len(data) >= 4 and zlib.crc32(data[:-4]).to_bytes(4, "little") == data[-4:]
        self.frames.append((data, ok))


class EthRxDriver:
    """10BASE-T receive-side stimulus: what a line receiver (comparator) on RD+/RD-
    would give the chip. One logic input, idle low; per IEEE 802.3 clause 14 each
    100 ns bit is the complement of the bit for 50 ns, then the bit; after the
    frame the line stays high (TP_IDL) for idle_ns, then returns to idle.
    Frames are 56 bits of preamble (1010...), the SFD 0xD5, the bytes LSB first
    and the FCS (zlib CRC-32, little endian). Every edge can be moved by a
    seeded uniform jitter of +/- jitter_ps. Written from the standard, not the RTL."""

    def __init__(self, dut, ui_bit, bit_ps=100_000, jitter_ps=0, seed=1, idle_ns=300):
        from .pins import ui_pins

        self.pins, self.bit, self.bit_ps = ui_pins(dut), ui_bit, bit_ps
        self.jitter_ps, self.idle_ns = jitter_ps, idle_ns
        import random

        self.rng = random.Random(seed)
        self.pins.set(ui_bit, 0)

    @staticmethod
    def frame_bits(payload, fcs=True, bad_fcs=False):
        data = bytes(payload)
        if fcs:
            crc = zlib.crc32(data) ^ (1 if bad_fcs else 0)
            data += crc.to_bytes(4, "little")
        bits = [1, 0] * 28 + [1, 0, 1, 0, 1, 0, 1, 1]
        for byte in data:
            bits += [(byte >> i) & 1 for i in range(8)]
        return bits

    async def send(self, payload, **kw):
        from cocotb.triggers import Timer

        bits = self.frame_bits(payload, **kw)
        half = self.bit_ps // 2
        levels = []
        for b in bits:
            levels += [1 - b, b]
        levels.append(1)  # TP_IDL starts where the last bit ends
        # absolute edge times (ps) from the start of the frame, with jitter
        edges, t, cur = [], 0, 0
        for i, lv in enumerate(levels):
            if lv != cur:
                j = self.rng.randint(-self.jitter_ps, self.jitter_ps) if self.jitter_ps else 0
                edges.append((max(t + j, 0), lv))
                cur = lv
            t += half
        edges.append((t + self.idle_ns * 1000 - half, 0))  # back to idle after TP_IDL
        now = 0
        for te, lv in edges:
            if te > now:
                await Timer(te - now, unit="ps")
                now = te
            self.pins.set(self.bit, lv)
