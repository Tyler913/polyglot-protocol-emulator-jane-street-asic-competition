# Copyright (c) 2026 Tyler Hong
# SPDX-License-Identifier: Apache-2.0
"""Independent SD card model in SPI mode, written from the SD Physical Layer
Simplified Specification (part 7, SPI mode): 48-bit commands (start 01,
index, argument, CRC7 with G(x) = x^7 + x^3 + 1, end bit 1), R1/R3/R7
responses after 1..8 bytes (NCR), ACMD41 initialization, OCR with CCS, and
CMD17 data: token 0xFE, 512 bytes, CRC16 (x^16 + x^12 + x^5 + 1). Mode 0:
samples MOSI on the rising SCK edge, changes MISO after the falling edge.
Every command's CRC7 is checked (a card checks CMD0/CMD8 only by default;
this model is stricter). Not written from the RTL."""

import cocotb
from cocotb.triggers import ReadOnly, RisingEdge
from cocotb.utils import get_sim_time

from .pins import bit, ui_pins


def crc7(data):
    c = 0
    for b in data:
        for i in range(7, -1, -1):
            fb = ((c >> 6) & 1) ^ ((b >> i) & 1)
            c = (c << 1) & 0x7F
            if fb:
                c ^= 0x09
    return c


def crc16(data):
    c = 0
    for b in data:
        for i in range(7, -1, -1):
            fb = ((c >> 15) & 1) ^ ((b >> i) & 1)
            c = (c << 1) & 0xFFFF
            if fb:
                c ^= 0x1021
    return c


def block_data(n):
    return bytes((7 * i + 3 * n) & 0xFF for i in range(512))


class SdCard:
    def __init__(self, dut, sck, mosi, cs, miso_ui, busy_inits=2):
        self.dut, self.sck, self.mosi, self.cs, self.miso = dut, sck, mosi, cs, miso_ui
        self.ui = ui_pins(dut)
        self.ui.set(miso_ui, 1)
        self.errors, self.cmds = [], []
        self.spi_mode = False
        self.idle = True
        self.busy_inits = busy_inits
        self.app = False
        self.pre_clocks = 0
        self.out = []  # bytes queued on MISO

    def start(self):
        self._task = cocotb.start_soon(self._run())

    def stop(self):
        self._task.cancel()

    def _pins(self):
        o = self.dut.uo_out   # single bits: uo_out[7] (host MISO) can be X
        return bit(o, self.sck), bit(o, self.mosi), bit(o, self.cs)

    async def _run(self):
        sck_l, _, cs_l = self._pins()
        sh, n, tx, cmd = 0, 0, 0xFF, []
        pending = None
        while True:
            await RisingEdge(self.dut.clk)
            if pending is not None:
                self.ui.set(self.miso, pending)
                pending = None
            await ReadOnly()
            sck, mosi, cs = self._pins()
            if cs == 1:
                if sck and not sck_l and mosi:
                    self.pre_clocks += 1
                if cs_l == 0:
                    pending = 1
            elif cs_l == 1:  # selected
                sh, n = 0, 0
                tx = self.out.pop(0) if self.out else 0xFF
                pending = (tx >> 7) & 1
            elif sck and not sck_l:  # rising: sample
                sh = ((sh << 1) | mosi) & 0xFF
                n += 1
            elif not sck and sck_l:  # falling: next MISO bit
                if n % 8 == 0:
                    if n:
                        cmd = self._byte(sh, cmd)
                    tx = self.out.pop(0) if self.out else 0xFF
                pending = (tx >> (7 - n % 8)) & 1
            sck_l, cs_l = sck, cs

    def _byte(self, b, cmd):
        if not cmd and b & 0xC0 != 0x40:
            return cmd  # idle 0xFF between commands
        cmd = cmd + [b]
        if len(cmd) < 6:
            return cmd
        idx, arg = cmd[0] & 0x3F, int.from_bytes(bytes(cmd[1:5]), "big")
        if cmd[5] & 1 != 1:
            self.errors.append(f"CMD{idx}: end bit 0")
        if crc7(cmd[:5]) != cmd[5] >> 1:
            self.errors.append(f"CMD{idx}: CRC7 {cmd[5] >> 1:#04x}, want {crc7(cmd[:5]):#04x}")
        self.cmds.append((idx, arg, self.app))
        self.cmd_times = getattr(self, "cmd_times", []) + [round(get_sim_time("ns"))]
        self.out = [0xFF] + self._exec(idx, arg)  # NCR = 1 byte
        return []

    def _exec(self, idx, arg):
        app, self.app = self.app, False
        r1 = 0x01 if self.idle else 0x00
        if idx == 0:
            if self.pre_clocks < 74:
                self.errors.append(f"only {self.pre_clocks} clocks before CMD0")
            self.spi_mode, self.idle = True, True
            return [0x01]
        if not self.spi_mode:
            self.errors.append(f"CMD{idx} before CMD0")
            return []
        if idx == 8:
            return [r1, 0, 0, (arg >> 8) & 0xF, arg & 0xFF]
        if idx == 55:
            self.app = True
            return [r1]
        if idx == 41 and app:
            if self.busy_inits:
                self.busy_inits -= 1
            else:
                self.idle = False
            return [0x01 if self.idle else 0x00]
        if idx == 58:
            ocr = 0x80FF8000 | (0x40000000 if not self.idle else 0)
            return [r1] + list(ocr.to_bytes(4, "big"))
        if idx == 17 and not self.idle:
            d = block_data(arg)
            c = crc16(d)
            return [0x00, 0xFF, 0xFF, 0xFE] + list(d) + [c >> 8, c & 0xFF]
        return [r1 | 0x04]  # illegal command
