# Copyright (c) 2026 Tyler Hong
# SPDX-License-Identifier: Apache-2.0
"""Independent RMII PHY model (RMII 1.2 specification, IEEE 802.3 clause 3/4
frame format), not from the RTL: two bits per 50 MHz REF_CLK cycle (the chip
clock), each byte LSB first as dibits {bit1, bit0}, {bit3, bit2}, ...
Transmit side (chip -> PHY): TX_EN high for the whole frame; 7 bytes 0x55,
SFD 0xD5, data, FCS = CRC-32 (zlib) of the data, little endian; at least 96
bit times (48 cycles) between frames. Receive side (PHY -> chip): CRS_DV high
with the same preamble/SFD/data/FCS on RXD; the model leaves 48 cycles
between frames and can corrupt an FCS on purpose."""

import zlib

import cocotb
from cocotb.triggers import ReadOnly, RisingEdge

from .pins import bit, ui_pins


def frame_dibits(data, bad_fcs=False):
    fcs = zlib.crc32(bytes(data)) ^ (1 if bad_fcs else 0)
    raw = bytes([0x55] * 7 + [0xD5]) + bytes(data) + fcs.to_bytes(4, "little")
    return [(b >> (2 * k)) & 3 for b in raw for k in range(4)]


class RmiiPhy:
    def __init__(self, dut, txd=(0, 1), tx_en=2, rxd=(0, 1), crs_dv=2):
        self.dut, self.txd, self.tx_en, self.rxd, self.crs = dut, txd, tx_en, rxd, crs_dv
        self.ui = ui_pins(dut)
        for p in (*rxd, crs_dv):
            self.ui.set(p, 0)
        self.frames, self.errors, self.gaps = [], [], []
        self.queue = []          # (data, bad_fcs, gap_cycles) to send to the chip

    def start(self):
        self._tasks = [cocotb.start_soon(self._tx_mon()), cocotb.start_soon(self._rx_drv())]

    def stop(self):
        for t in self._tasks:
            t.cancel()

    async def _tx_mon(self):
        t, cur, last_end = 0, None, None
        while True:
            await RisingEdge(self.dut.clk)
            await ReadOnly()
            t += 1
            en = bit(self.dut.uo_out, self.tx_en)
            d = bit(self.dut.uo_out, self.txd[0]) | (bit(self.dut.uo_out, self.txd[1]) << 1)
            if en:
                if cur is None:
                    cur = []
                    if last_end is not None:
                        self.gaps.append(t - last_end)
                        if t - last_end < 48:
                            self.errors.append(f"inter-frame gap {t - last_end} cycles")
                cur.append(d)
            elif cur is not None:
                self._check(cur)
                cur, last_end = None, t

    def _check(self, dib):
        if len(dib) % 4:
            self.errors.append(f"frame of {len(dib)} dibits (not whole bytes)")
            return
        raw = bytes(sum(dib[4 * i + k] << (2 * k) for k in range(4)) for i in range(len(dib) // 4))
        if raw[:8] != bytes([0x55] * 7 + [0xD5]):
            self.errors.append(f"preamble/SFD {raw[:8].hex()}")
            return
        data, fcs = raw[8:-4], int.from_bytes(raw[-4:], "little")
        if zlib.crc32(data) != fcs:
            self.errors.append(f"FCS of a {len(data)}-byte frame")
        self.frames.append(data)

    async def _rx_drv(self):
        while True:
            await RisingEdge(self.dut.clk)
            if not self.queue:
                continue
            data, bad, gap = self.queue.pop(0)
            for _ in range(gap):
                await RisingEdge(self.dut.clk)
            for d in frame_dibits(data, bad):
                self.ui.set(self.crs, 1)
                self.ui.set(self.rxd[0], d & 1)
                self.ui.set(self.rxd[1], d >> 1)
                await RisingEdge(self.dut.clk)
            self.ui.set(self.crs, 0)
            self.ui.set(self.rxd[0], 0)
            self.ui.set(self.rxd[1], 0)
