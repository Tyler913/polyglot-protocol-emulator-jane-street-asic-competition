# Copyright (c) 2026 Tyler Hong
# SPDX-License-Identifier: Apache-2.0
"""LIN bus with an independent slave node, written from ISO 17987 / LIN 2.2A
(break of at least 13 dominant bits, detected by a slave at 11 bits; break
delimiter; sync 0x55; protected ID with parity P0 = ID0^ID1^ID2^ID4,
P1 = ~(ID1^ID3^ID4^ID5); classic checksum over data, enhanced over PID +
data, both the inverted sum with carry), not from the RTL.

The bus is the wired AND of the chip's TXD and the slave's output; the
chip's RXD (a ui pin) follows the bus, as through a LIN transceiver. The
slave records every frame and answers the IDs in `responses`."""

import cocotb
from cocotb.triggers import ClockCycles, ReadOnly, RisingEdge

from .pins import bit, ui_pins


def pid(i):
    p0 = (i ^ (i >> 1) ^ (i >> 2) ^ (i >> 4)) & 1
    p1 = ~((i >> 1) ^ (i >> 3) ^ (i >> 4) ^ (i >> 5)) & 1
    return (i & 0x3F) | (p0 << 6) | (p1 << 7)


def checksum(data, start=0):
    s = start
    for b in data:
        s += b
        if s > 0xFF:
            s -= 0xFF
    return ~s & 0xFF


class LinBus:
    def __init__(self, dut, txd_uo, rxd_ui, bit_cycles, responses=None, lengths=None, classic=()):
        self.dut, self.txd, self.rxd, self.bit = dut, txd_uo, rxd_ui, bit_cycles
        self.ui = ui_pins(dut)
        self.responses = responses or {}  # id -> bytes the slave publishes
        self.lengths = lengths or {}      # id -> data length of frames the master publishes
        self.classic = set(classic)       # ids using the classic checksum
        self.slave = 1
        self.bus = 1
        self.t = 0
        self.fall = 0  # time of the last falling bus edge
        self.frames = []  # (id, data, checksum_ok)
        self.errors = []
        self.ui.set(rxd_ui, 1)

    def start(self):
        self._tasks = [cocotb.start_soon(self._wire()), cocotb.start_soon(self._slave())]

    def stop(self):
        for t in self._tasks:
            t.cancel()

    async def _wire(self):
        while True:
            await RisingEdge(self.dut.clk)
            await ReadOnly()
            self.t += 1
            v = bit(self.dut.uo_out, self.txd) & self.slave
            if self.bus and not v:
                self.fall = self.t
            self.bus = v
            await RisingEdge(self.dut.clk)  # write outside the read-only phase
            self.t += 1
            self.ui.set(self.rxd, self.bus)
            # one bus update per two cycles is plenty at 19200 baud (2604 cycles per bit)

    async def _wait_level(self, v):
        while self.bus != v:
            await ClockCycles(self.dut.clk, 1)

    async def _get_byte(self, timeout_bits=20):
        t0 = self.t
        while self.bus != 0:
            await ClockCycles(self.dut.clk, 1)
            if self.t - t0 > timeout_bits * self.bit:
                return None
        await ClockCycles(self.dut.clk, int(self.bit * 1.5))
        v = 0
        for i in range(8):
            v |= self.bus << i
            await ClockCycles(self.dut.clk, int(self.bit))
        if self.bus != 1:
            self.errors.append("framing error")
        return v

    async def _send_byte(self, b):
        for v in [0] + [(b >> i) & 1 for i in range(8)] + [1]:
            self.slave = v
            await ClockCycles(self.dut.clk, round(self.bit))
        self.slave = 1

    async def _slave(self):
        # Before its first header the chip's TXD is not yet configured (low):
        # start listening once the bus has been recessive for a bit time.
        while True:
            await self._wait_level(1)
            t0 = self.t
            while self.bus and self.t - t0 < self.bit:
                await ClockCycles(self.dut.clk, 1)
            if self.bus:
                break
        while True:
            await self._wait_level(0)
            await self._wait_level(1)
            low = (self.t - self.fall) / self.bit
            if low < 11:
                continue  # not a break
            if low < 12.99:
                self.errors.append(f"break of {low:.1f} bits (master must send >= 13)")
            sync = await self._get_byte()
            if sync != 0x55:
                self.errors.append(f"sync {sync}")
                continue
            p = await self._get_byte()
            i = p & 0x3F
            if pid(i) != p:
                self.errors.append(f"PID parity {p:#04x}")
                continue
            start = 0 if i in self.classic else p
            if i in self.responses:
                await ClockCycles(self.dut.clk, round(self.bit))  # response space
                d = bytes(self.responses[i])
                for b in d + bytes([checksum(d, start)]):
                    await self._send_byte(b)
                self.frames.append((i, d, True))
            elif i in self.lengths:
                d = bytes([await self._get_byte() for _ in range(self.lengths[i])])
                c = await self._get_byte()
                self.frames.append((i, d, c == checksum(d, start)))
