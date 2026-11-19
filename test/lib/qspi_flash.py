# Copyright (c) 2026 Tyler Hong
# SPDX-License-Identifier: Apache-2.0
"""Independent quad-SPI NOR flash, written from the W25Q-series datasheet
description of "Fast Read Quad Output" (0x6B), not from the RTL: SPI mode 0
(SCK idles low, inputs sampled on rising edges, outputs change on falling
edges); after CS# falls: command (8 bits) and 24-bit address on IO0, MSB
first, then 8 dummy clocks; from the falling edge after the last dummy
clock the flash drives IO3..IO0 with one nibble per clock, high nibble of
each byte first, address auto-incrementing, until CS# rises. The model
reports bus contention if the host still drives IO0 while the flash does."""

import cocotb
from cocotb.triggers import ReadOnly, RisingEdge

from .pins import bit, ui_pins


class QspiFlash:
    def __init__(self, dut, sck_uo, cs_uo, io0_uio, io_ui, mem):
        self.dut, self.sck, self.cs, self.io0, self.io_ui = dut, sck_uo, cs_uo, io0_uio, io_ui
        self.mem = bytes(mem)
        self.ui = ui_pins(dut)
        self.errors = []
        self.commands = []

    def start(self):
        self._task = cocotb.start_soon(self._run())

    def stop(self):
        self._task.cancel()

    def _drive(self, nib):
        if nib is None:
            for k in range(3):
                self.ui.set(self.io_ui[k], 0)
            self.io0_val = None
            return
        for k in range(3):
            self.ui.set(self.io_ui[k], (nib >> (k + 1)) & 1)
        self.io0_val = nib & 1

    async def _run(self):
        prev_sck, prev_cs = 0, 1
        self.io0_val = None
        rises, shift, addr, nibs = 0, 0, 0, 0
        while True:
            await RisingEdge(self.dut.clk)
            await ReadOnly()
            sck, cs = bit(self.dut.uo_out, self.sck), bit(self.dut.uo_out, self.cs)
            oe0 = (int(self.dut.uio_oe.value) >> self.io0) & 1
            out0 = (int(self.dut.uio_out.value) >> self.io0) & 1
            drive = None
            if cs and not prev_cs:                       # deselect
                if rises:
                    self.commands.append((shift >> 32 & 0xFF if rises >= 40 else None, addr, nibs // 2))
                rises, shift, nibs = 0, 0, 0
                drive = "off"
            elif not cs and prev_cs:
                rises, shift, nibs = 0, 0, 0
            elif not cs:
                if sck and not prev_sck:                 # rising edge: sample
                    if rises < 40:
                        if rises < 32 and not oe0:           # command + address; dummy clocks are don't-care
                            self.errors.append(f"IO0 not driven by the host at clock {rises}")
                        shift = (shift << 1) | out0
                    rises += 1
                    if rises == 40:
                        cmd, addr = (shift >> 32) & 0xFF, (shift >> 8) & 0xFFFFFF
                        if cmd != 0x6B:
                            self.errors.append(f"command {cmd:#x}")
                elif prev_sck and not sck and rises >= 40:   # falling edge: next nibble out
                    b = self.mem[(addr + nibs // 2) % len(self.mem)]
                    drive = (b >> 4) if nibs % 2 == 0 else (b & 0xF)
                    nibs += 1
            if self.io0_val is not None and oe0:
                self.errors.append("bus contention on IO0 (host and flash both drive)")
            prev_sck, prev_cs = sck, cs
            await RisingEdge(self.dut.clk)
            if drive == "off":
                self._drive(None)
            elif drive is not None:
                self._drive(drive)
            v = int(self.dut.uio_in.value) & ~(1 << self.io0) & 0xFF
            io0 = self.io0_val if self.io0_val is not None else out0
            self.dut.uio_in.value = v | (io0 << self.io0)
