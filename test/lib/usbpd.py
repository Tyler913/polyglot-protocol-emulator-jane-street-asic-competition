# Copyright (c) 2026 Tyler Hong
# SPDX-License-Identifier: Apache-2.0
"""Independent USB Power Delivery source model (physical and protocol layer),
written from the USB PD specification's description as implemented in public
software PHYs (Chromium EC), not from our RTL.

Physical layer: biphase mark coding at 300 kbit/s on the CC wire; a packet is
a 64-bit preamble (alternating, starting with 0), the SOP ordered set
(Sync-1 x3, Sync-2), the 16-bit header, 32-bit data objects and the CRC-32
(IEEE, over header and objects), all 4b5b coded (low nibble first, symbol
bits LSB first), then EOP; the transmitter ends with a final edge and
releases the line. Protocol: every message is answered by a GoodCRC that
echoes its MessageID.

PdSource: Source_Capabilities with fixed PDOs, then expects a Request,
answers Accept and PS_RDY, checking the sink's GoodCRCs and their timing.
"""

import binascii

import cocotb
from cocotb.triggers import FallingEdge

from .pins import bit

ENC = [0x1E, 0x09, 0x14, 0x15, 0x0A, 0x0B, 0x0E, 0x0F, 0x12, 0x13, 0x16, 0x17, 0x1A, 0x1B, 0x1C, 0x1D]
DEC = {v: i for i, v in enumerate(ENC)}
SYNC1, SYNC2, EOP = 0x18, 0x11, 0x0D
GOODCRC, ACCEPT, PS_RDY, SRC_CAP, REQUEST = 1, 3, 6, 1, 2


def header(mtype, cnt, mid, prole, drole):
    return mtype | (drole << 5) | (2 << 6) | (prole << 8) | ((mid & 7) << 9) | ((cnt & 7) << 12)


def packet_bits(hdr, objs):
    data = hdr.to_bytes(2, "little") + b"".join(o.to_bytes(4, "little") for o in objs)
    crc = binascii.crc32(data) & 0xFFFFFFFF
    syms = [SYNC1, SYNC1, SYNC1, SYNC2]
    for byte in data + crc.to_bytes(4, "little"):
        syms += [ENC[byte & 15], ENC[byte >> 4]]
    syms.append(EOP)
    bits = [i % 2 for i in range(64)]
    for s in syms:
        bits += [(s >> i) & 1 for i in range(5)]
    return bits


def bmc_halves(bits):
    out, lvl = [], 0
    for b in bits:
        lvl ^= 1
        out.append(lvl)
        if b:
            lvl ^= 1
        out.append(lvl)
    if lvl == 0:                      # final edge: up for one bit, then down
        out += [1, 1]
    return out + [0, 0]


def decode_bits(intervals, half):
    """Bits from the intervals between edges (first edge = start of bit 0)."""
    bits, i, errs = [], 0, []
    q = [max(1, min(2, round(n / half))) for n, _ in intervals]
    while i < len(q):
        if q[i] == 2:
            bits.append(0)
            i += 1
        elif i + 1 < len(q) and q[i + 1] == 1:
            bits.append(1)
            i += 2
        else:
            errs.append(f"half-bit interval alone at {i}")
            i += 1
    return bits, errs


def parse(bits):
    """(header, objects) from packet bits, or raise ValueError."""
    k = None
    for i in range(len(bits) - 20):
        syms = [sum(bits[i + 5 * j + b] << b for b in range(5)) for j in range(4)]
        if syms == [SYNC1, SYNC1, SYNC1, SYNC2]:
            k = i + 20
            break
    if k is None:
        raise ValueError("no SOP")
    pre = bits[:k - 20]
    if pre[-8:] != [0, 1] * 4:
        raise ValueError("preamble")
    nib = []
    while k + 5 <= len(bits):
        s = sum(bits[k + b] << b for b in range(5))
        k += 5
        if s == EOP:
            break
        if s not in DEC:
            raise ValueError(f"bad symbol {s:#x}")
        nib.append(DEC[s])
    else:
        raise ValueError("no EOP")
    data = bytes(nib[2 * i] | nib[2 * i + 1] << 4 for i in range(len(nib) // 2))
    if len(data) < 6 or (len(data) - 6) % 4:
        raise ValueError(f"length {len(data)}")
    if binascii.crc32(data[:-4]) & 0xFFFFFFFF != int.from_bytes(data[-4:], "little"):
        raise ValueError("CRC")
    hdr = int.from_bytes(data[:2], "little")
    objs = [int.from_bytes(data[2 + 4 * i:6 + 4 * i], "little") for i in range((len(data) - 6) // 4)]
    if (hdr >> 12) & 7 != len(objs):
        raise ValueError("object count")
    return hdr, objs


class PdSource:
    def __init__(self, dut, tx_uio, rx_ui, f_cpu, pdos):
        self.dut, self.tx, self.rx = dut, tx_uio, rx_ui
        self.half = f_cpu / 600_000
        self.pdos = pdos
        self.drive = None             # model's level on CC, None = released
        self.t = 0
        self.errors = []
        self.got = []                 # (time of the chip packet's first edge, header, objects)
        self.sent_end = []            # time our packets ended
        self._edges, self._lvl, self._last = [], None, None

    def start(self, ui):
        self.ui = ui
        self._task = cocotb.start_soon(self._run())

    def stop(self):
        self._task.cancel()

    async def _run(self):
        while True:
            await FallingEdge(self.dut.clk)
            self.t += 1
            oe = bit(self.dut.uio_oe, self.tx)
            c = bit(self.dut.uio_out, self.tx) if oe else None
            if c is not None and self.drive is not None:
                self.errors.append(f"CC contention at {self.t}")
            line = c if c is not None else (self.drive if self.drive is not None else 0)
            self.ui.set(self.rx, line)
            # the chip's packets: edges while it drives
            if c is not None:
                if self._lvl is not None and c != self._lvl:
                    if self._last is not None:
                        self._edges.append((self.t - self._last, self._lvl))
                    self._last = self.t
                elif self._lvl is None and c == 1:
                    self._last = self._first = self.t   # first edge out of idle
                self._lvl = c
            elif self._lvl is not None:          # released: packet over
                bits, errs = decode_bits(self._edges, self.half)
                try:
                    h, o = parse(bits)
                    self.got.append((self._first, h, o))
                except ValueError as e:
                    self.errors.append(f"chip packet: {e} ({errs[:2]})")
                self._edges, self._lvl, self._last = [], None, None

    async def send(self, hdr, objs=()):
        for h in bmc_halves(packet_bits(hdr, list(objs))):
            self.drive = h
            for _ in range(round(self.half)):
                await FallingEdge(self.dut.clk)
        self.drive = None
        self.sent_end.append(self.t)

    async def expect(self, n_before, timeout):
        for _ in range(timeout):
            if len(self.got) > n_before:
                return self.got[n_before]
            await FallingEdge(self.dut.clk)
        raise AssertionError(f"no packet from the chip within {timeout} cycles")
