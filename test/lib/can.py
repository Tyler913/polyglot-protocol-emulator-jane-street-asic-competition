# Copyright (c) 2026 Tyler Hong
# SPDX-License-Identifier: Apache-2.0
"""Independent CAN 2.0A model, written from ISO 11898-1 / Bosch CAN 2.0 (not
from the RTL): wired-AND bus (dominant 0 wins), bit stuffing after five equal
bits from SOF to the end of the CRC, CRC-15 (x^15+x^14+x^10+x^8+x^7+x^4+x^3+1),
CRC delimiter, ACK slot driven dominant by every receiver that got a good
frame, ACK delimiter, 7-bit EOF.

CanBus: each clock, bus = chip TXD AND every model node; the chip's RXD
gets the bus level after a transceiver loop delay. The chip's TXD is a uio
pin; while its driver is off the transceiver's pull-up makes it recessive.
CanNode: a controller that hard-syncs on SOF, resyncs on recessive-to-
dominant edges, samples at 75 % of the bit, receives and ACKs frames, and
can transmit one frame, joining an SOF it sees on the bus (arbitration).
"""

import cocotb
from cocotb.triggers import FallingEdge

from .pins import bit, ui_pins

POLY = 0x4599


def crc15(bits):
    c = 0
    for b in bits:
        fb = b ^ ((c >> 14) & 1)
        c = (c << 1) & 0x7FFF
        if fb:
            c ^= POLY
    return c


def frame_bits(fid, data):
    """Unstuffed SOF..CRC bits of a standard data frame."""
    bits = [0] + [(fid >> (10 - i)) & 1 for i in range(11)] + [0, 0, 0]
    bits += [(len(data) >> (3 - i)) & 1 for i in range(4)]
    for x in data:
        bits += [(x >> (7 - i)) & 1 for i in range(8)]
    c = crc15(bits)
    return bits + [(c >> (14 - i)) & 1 for i in range(15)]


def stuff(bits):
    out, run, last = [], 0, None
    for b in bits:
        out.append(b)
        run = run + 1 if b == last else 1
        last = b
        if run == 5:
            out.append(1 - b)
            last, run = 1 - b, 1
    return out


class CanBus:
    def __init__(self, dut, txd_uio, rxd_ui, bit_cycles, loop=4):
        self.dut, self.txd, self.rxd, self.bit = dut, txd_uio, rxd_ui, bit_cycles
        self.loop = loop
        self.ui = ui_pins(dut)
        self.nodes = []
        self.hist = [1] * (loop + 1)
        self.level = 1
        self.t = 0
        self.ui.set(rxd_ui, 1)

    def add(self, node):
        self.nodes.append(node)
        return node

    def start(self):
        self._task = cocotb.start_soon(self._run())

    def stop(self):
        self._task.cancel()

    async def _run(self):
        while True:
            await FallingEdge(self.dut.clk)
            self.t += 1
            lvl = bit(self.dut.uio_out, self.txd) if bit(self.dut.uio_oe, self.txd) else 1
            for n in self.nodes:
                lvl &= n.tx
            self.level = lvl
            self.hist = self.hist[1:] + [lvl]
            self.ui.set(self.rxd, self.hist[0])
            for n in self.nodes:
                n.step(self.t, lvl)


class CanNode:
    def __init__(self, bus, name):
        self.bus, self.name, self.bit = bus, name, bus.bit
        self.tx = 1
        self.frames = []  # received or sent: dicts id, data, crc_ok, acked, by
        self.errors = []
        self.pending = None  # (id, data) to send
        self.join_sof = False
        self._reset_rx()
        self.idle_bits = 11
        self.prev = 1
        self.next_sample = None

    def send(self, fid, data, join_sof=True):
        self.pending = (fid, list(data))
        self.join_sof = join_sof

    def _reset_rx(self):
        self.state = "idle"
        self.raw = []  # destuffed bits of the frame
        self.run, self.last = 0, None
        self.tx_bits = None
        self.txi = 0
        self.ack_drive = False

    def step(self, t, lvl):
        # Bit timing: hard sync at SOF, resync on recessive->dominant edges
        if self.prev == 1 and lvl == 0:
            if self.state == "idle" and self.idle_bits >= 2:
                self.state = "frame"
                self.raw, self.run, self.last = [], 0, None
                self.bitstart = t
                if self.pending and self.join_sof:          # start our frame on this SOF
                    self.tx_bits = stuff(frame_bits(*self.pending))
                    self.txi = 1
                    self.tx = 0                              # drive our own SOF too
                    self.sending = self.pending
            elif self.state == "frame":                    # resync on the edge
                if t - self.bitstart > self.bit / 2:         # early edge: the next bit starts now
                    self.bitstart = t
                    self.prev = lvl
                    self._drive_next()
                    return
                self.bitstart = t                            # late edge: shift the timing
        self.prev = lvl
        if self.state == "idle":
            if (t - getattr(self, "idle_mark", 0)) >= self.bit:
                self.idle_mark = t
                self.idle_bits = self.idle_bits + 1 if lvl else 0
            return
        pos = t - self.bitstart
        if pos >= round(self.bit):                       # next bit begins
            self.bitstart += round(self.bit)
            pos = t - self.bitstart
            self._drive_next()
        if pos == round(0.75 * self.bit):
            self._sample(lvl)

    def _drive_next(self):
        if self.ack_drive:
            self.tx = 0
            self.ack_drive = False
            self.ack_bit = True
            return
        if getattr(self, "ack_bit", False):
            self.ack_bit = False
        if self.tx_bits is not None and self.txi < len(self.tx_bits):
            self.tx = self.tx_bits[self.txi]
            self.txi += 1
        else:
            self.tx = 1

    def _sample(self, lvl):
        if self.tx_bits is not None and self.txi <= len(self.tx_bits) and self.tx == 1 and lvl == 0 \
                and self.phase() == "arb":
            self.tx_bits = None                           # lost arbitration: become a receiver
            self.tx = 1
        end0 = self.crc_end()
        if self.phase() in ("arb", "ctrl", "data", "crc") or (self.run == 5 and end0 and len(self.raw) == end0):
            # destuff
            if self.run == 5:
                if lvl == self.last:
                    self.errors.append("stuff error")
                self.run, self.last = 1, lvl
                return
            self.run = self.run + 1 if lvl == self.last else 1
            self.last = lvl
        self.raw.append(lvl)
        n = len(self.raw)
        end = self.crc_end()
        if end and n == end + 1:                         # CRC delimiter sampled
            if not lvl:
                self.errors.append("CRC delimiter dominant")
            ok = crc15(self.raw[:end]) == 0
            if ok and self.tx_bits is None:
                self.ack_drive = True                    # ACK in the next bit
            self.crc_ok = ok
        elif end and n == end + 2:                       # ACK slot
            self.acked = lvl == 0
        elif end and n == end + 2 + 1 + 7:               # ACK delimiter + EOF
            fid = int("".join(map(str, self.raw[1:12])), 2)
            dlc = min(8, int("".join(map(str, self.raw[15:19])), 2))
            data = [int("".join(map(str, self.raw[19 + 8 * i: 27 + 8 * i])), 2) for i in range(dlc)]
            by = self.name if self.tx_bits is not None else "other"
            self.frames.append(dict(id=fid, data=data, crc_ok=self.crc_ok, acked=self.acked, by=by))
            if by == self.name:
                self.pending = None
            self._reset_rx()
            self.idle_bits = 0
            self.idle_mark = self.bitstart

    def phase(self):
        n = len(self.raw)
        if n < 12:
            return "arb"
        if n < 19:
            return "ctrl"
        end = self.crc_end()
        if end is None or n < end - 15:
            return "data"
        if n < end:
            return "crc"
        return "tail"

    def crc_end(self):
        if len(self.raw) < 19:
            return None
        dlc = min(8, int("".join(map(str, self.raw[15:19])), 2))
        return 19 + 8 * dlc + 15
