# Copyright (c) 2026 Tyler Hong
# SPDX-License-Identifier: Apache-2.0
"""Independent SPI peer models, written from the SPI mode definitions
(CPOL = idle level of SCK, CPHA = 0: sample on the leading edge,
CPHA = 1: sample on the trailing edge), not from the RTL.

SpiRam: a 256-byte SPI RAM target (0x02 WRITE addr data..., 0x03 READ addr
-> data...) that watches SCK/MOSI/CS on chip outputs and drives MISO on a
ui_in bit. It also checks the bus protocol.

SpiController: drives SCK and MOSI on ui_in bits and samples MISO from a
chip output, to test the chip as an SPI target.
"""

import cocotb
from cocotb.triggers import ClockCycles, ReadOnly, RisingEdge

from .pins import bit, ui_pins


class SpiRam:
    def __init__(self, dut, mode, sck, mosi, cs, miso_ui):
        self.dut = dut
        self.cpol = (mode >> 1) & 1
        self.cpha = mode & 1
        self.sck, self.mosi, self.cs = sck, mosi, cs  # uo_out bit numbers
        self.miso_ui = miso_ui
        self.ui = ui_pins(dut)
        self.mem = bytearray(256)
        self.errors = []
        self.frames = []  # bytes received per CS frame
        self.in_frame = False
        self.idle_edges = 0  # CS is low after reset; a frame begins only on a falling CS edge
        self.ui.set(miso_ui, 1)

    def start(self):
        self._task = cocotb.start_soon(self._run())

    def stop(self):
        self._task.cancel()

    def _pins(self):
        # single bits: uo_out[7] (host MISO) can be X while the host writes
        o = self.dut.uo_out
        return bit(o, self.sck), bit(o, self.mosi), bit(o, self.cs)

    async def _run(self):
        sck_l, _, cs_l = self._pins()
        pending = None
        while True:
            await RisingEdge(self.dut.clk)
            if pending is not None:  # drive MISO one cycle after the edge (device delay)
                self.ui.set(self.miso_ui, pending)
                pending = None
            await ReadOnly()
            sck, mosi, cs = self._pins()
            out = None  # MISO bit to drive after this cycle
            if cs_l == 1 and cs == 0:  # frame start
                self.in_frame = True
                if sck != self.cpol:
                    self.errors.append("SCK not idle when CS fell")
                self.frame = []
                self.nbits = 0
                self.sh = 0
                self.cmd = None
                self.addr = None
                self.tx = 0xFF
                if self.cpha == 0:
                    out = (self.tx >> 7) & 1
            elif cs_l == 0 and cs == 1 and self.in_frame:  # frame end
                self.in_frame = False
                if sck != self.cpol:
                    self.errors.append("SCK not idle when CS rose")
                if self.nbits % 8:
                    self.errors.append(f"frame ended after {self.nbits} bits (not a whole byte)")
                self.frames.append(bytes(self.frame))
                out = 1
            elif self.in_frame and sck != sck_l:
                leading = sck != self.cpol
                sample = leading if self.cpha == 0 else not leading
                if sample:
                    self.sh = ((self.sh << 1) | mosi) & 0xFF
                    self.nbits += 1
                    if self.nbits % 8 == 0:
                        self._byte(self.sh)
                else:  # shift edge: present the next MISO bit
                    k = self.nbits % 8  # bits already sampled in this byte
                    if self.cpha == 0:
                        if k:  # CPHA 0: bit 0 of the next byte goes out on the trailing edge after the last sample
                            out = (self.tx >> (7 - k)) & 1
                        else:
                            out = (self.tx >> 7) & 1
                    else:
                        out = (self.tx >> (7 - k)) & 1
            elif cs == 1 and sck != sck_l:
                self.idle_edges += 1  # allowed (targets ignore SCK while deselected), e.g. SCK settling to CPOL
            sck_l, cs_l = sck, cs
            pending = out

    def _byte(self, b):
        self.frame.append(b)
        n = len(self.frame)
        if n == 1:
            self.cmd = b
        elif n == 2:
            self.addr = b
        elif self.cmd == 0x02:
            self.mem[self.addr] = b
            self.addr = (self.addr + 1) & 0xFF
        nxt = 0xFF
        if self.cmd == 0x03 and n >= 2:
            nxt = self.mem[self.addr]
            self.addr = (self.addr + 1) & 0xFF
        self.tx = nxt


class SpiController:
    def __init__(self, dut, mode, sck_ui, mosi_ui, miso_uo, cs_ui=None, half=16):
        self.dut = dut
        self.cpol = (mode >> 1) & 1
        self.cpha = mode & 1
        self.sck_ui, self.mosi_ui, self.miso_uo, self.cs_ui = sck_ui, mosi_ui, miso_uo, cs_ui
        self.half = half
        self.ui = ui_pins(dut)
        self.ui.set(sck_ui, self.cpol)
        self.ui.set(mosi_ui, 0)
        if cs_ui is not None:
            self.ui.set(cs_ui, 1)

    def _miso(self):
        return bit(self.dut.uo_out, self.miso_uo)

    async def select(self, v):
        self.ui.set(self.cs_ui, 0 if v else 1)
        await ClockCycles(self.dut.clk, self.half)

    async def xfer(self, data, gap=0, bits=8):
        rx = []
        clk = self.dut.clk
        for b in data:
            r = 0
            for i in range(bits - 1, -1, -1):
                bit = (b >> i) & 1
                if self.cpha == 0:
                    self.ui.set(self.mosi_ui, bit)
                    await ClockCycles(clk, self.half)
                    self.ui.set(self.sck_ui, 1 - self.cpol)  # leading edge: sample
                    r = (r << 1) | self._miso()
                    await ClockCycles(clk, self.half)
                    self.ui.set(self.sck_ui, self.cpol)  # trailing edge
                else:
                    self.ui.set(self.sck_ui, 1 - self.cpol)  # leading edge: shift
                    self.ui.set(self.mosi_ui, bit)
                    await ClockCycles(clk, self.half)
                    self.ui.set(self.sck_ui, self.cpol)  # trailing edge: sample
                    r = (r << 1) | self._miso()
                    await ClockCycles(clk, self.half)
            rx.append(r)
            if gap:
                await ClockCycles(clk, gap)
        return rx
