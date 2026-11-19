# Copyright (c) 2026 Tyler Hong
# SPDX-License-Identifier: Apache-2.0
"""Independent ISO/IEC 7816-3 smart card (T=0, direct convention), written
from the standard, not from the RTL. Everything is timed in card clocks,
as in a real card: 1 etu = 372 clocks (Fi = 372, Di = 1). Characters:
start bit, 8 data bits LSB first, even parity, then at least 2 etu of guard
time. The I/O line is open drain with a pull-up (line = reader AND card).
The card answers RST high with its ATR 400..40000 clocks later, and to
GET CHALLENGE (00 84 00 00 Le) with the procedure byte INS, Le bytes and
90 00. The model checks the clock is free-running (no period over 1.5x the
first one), the reader's parity, and that the reader's characters leave
the guard time."""

import cocotb
from cocotb.triggers import ReadOnly, RisingEdge

from .pins import bit

ETU = 372


class SmartCard:
    def __init__(self, dut, clk_uo, rst_uo, io_uio, atr, challenge):
        self.dut, self.clk, self.rst, self.io = dut, clk_uo, rst_uo, io_uio
        self.atr, self.challenge = list(atr), list(challenge)
        self.errors, self.received, self.periods = [], [], []
        self.atr_delay = None

    def start(self):
        self._task = cocotb.start_soon(self._run())

    def stop(self):
        self._task.cancel()

    async def _run(self):
        t, k, prev_clk, prev_rst, last_rise = 0, 0, 0, 0, None
        out = 1                                 # card's open-drain output
        tx = []                                 # (clock count, level) schedule
        rx_start, rx_bits = None, []
        rst_k, line_prev = None, 1
        hdr = []
        while True:
            await RisingEdge(self.dut.clk)
            await ReadOnly()
            t += 1
            c = bit(self.dut.uo_out, self.clk)
            r = bit(self.dut.uo_out, self.rst)
            oe = (int(self.dut.uio_oe.value) >> self.io) & 1
            o = (int(self.dut.uio_out.value) >> self.io) & 1
            reader = 0 if (oe and not o) else 1
            line = reader & out
            clk_edge = c and not prev_clk
            prev_clk = c
            if clk_edge:
                if last_rise is not None:
                    p = t - last_rise
                    if self.periods and p > 1.5 * self.periods[0]:
                        self.errors.append(f"card clock stopped for {p} cycles")
                    if len(self.periods) < 4:
                        self.periods.append(p)
                last_rise = t
                k += 1
                # RST rising: schedule the ATR 1000 clocks later
                if r and not prev_rst:
                    rst_k = k
                    self.atr_delay = 1000
                    tx = self._schedule(self.atr, k + 1000)
                prev_rst = r
                # card transmit
                while tx and tx[0][0] <= k:
                    out = tx.pop(0)[1]
                line = reader & out
                # card receive (only while the card is not sending)
                if not tx and out == 1:
                    if rx_start is None and line == 0 and line_prev == 1:
                        rx_start, rx_bits = k, []
                    elif rx_start is not None:
                        pos = k - rx_start
                        if pos == ETU // 2 + ETU * len(rx_bits) and len(rx_bits) < 10:
                            rx_bits.append(line)
                        if len(rx_bits) == 10:
                            v = sum(rx_bits[1 + i] << i for i in range(8))
                            if rx_bits[0] != 0 or rx_bits[9] != (bin(v).count("1") & 1):
                                self.errors.append(f"reader character {v:#x}: start/parity")
                            self.received.append(v)
                            hdr.append(v)
                            rx_start = None
                            if len(hdr) == 5:
                                tx = self._command(hdr, k)
                                hdr = []
                line_prev = line
            await RisingEdge(self.dut.clk)
            t += 1
            v = int(self.dut.uio_in.value) & ~(1 << self.io) & 0xFF
            self.dut.uio_in.value = v | ((reader & out) << self.io)

    def _schedule(self, data, k0):
        s, kk = [], k0
        for b in data:
            bits = [0] + [(b >> i) & 1 for i in range(8)] + [bin(b).count("1") & 1]
            for v in bits:
                s.append((kk, v))
                kk += ETU
            s.append((kk, 1))
            kk += 2 * ETU                         # guard time
        return s

    def _command(self, h, k):
        cla, ins, p1, p2, p3 = h
        if ins == 0x84 and p3 == len(self.challenge):
            return self._schedule([0x84] + self.challenge + [0x90, 0x00], k + 2 * ETU)
        self.errors.append(f"unexpected command {bytes(h).hex()}")
        return self._schedule([0x6D, 0x00], k + 2 * ETU)
