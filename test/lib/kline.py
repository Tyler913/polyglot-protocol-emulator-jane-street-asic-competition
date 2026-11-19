# Copyright (c) 2026 Tyler Hong
# SPDX-License-Identifier: Apache-2.0
"""Independent K-line ECU, written from ISO 14230-2/-4 (KWP2000 on K-line,
OBD) and SAE J1979 (service 0x01 PIDs 0x0C, 0x0D), not from the RTL:

- K is one wire: line = tester TX AND ECU output; the tester's RX follows it.
- Fast init: after the bus was idle >= 300 ms (W5), K low 25 ms +/- 1 ms
  (TiniL); the first request byte starts 50 ms +/- 1 ms after the falling
  edge (TWuP).
- 10400 baud 8N1. Tester bytes P4 = 5..20 ms apart; the ECU answers after
  P2 = 30 ms with bytes P1 = 1 ms apart; the next request must come at
  least P3 = 55 ms after the reply.
- Messages: format byte (address mode in bits 7..6, length in 5..0),
  target, source, data, checksum (sum mod 256). StartCommunication (0x81)
  -> 0xC1 + key bytes 0x8F 0xEF; before it, nothing else is answered."""

import cocotb
from cocotb.triggers import ClockCycles, ReadOnly, RisingEdge

from .pins import bit, ui_pins
from .serial import Fmt, SerialRx

FMT = Fmt("8N1")
BIT = 50e6 / 10400
MS = 50_000


class KLineEcu:
    def __init__(self, dut, tx_uo, rx_ui, rpm_raw=0x1AF8, speed=88, addr=0x10):
        self.dut, self.tx, self.rxp = dut, tx_uo, rx_ui
        self.ui = ui_pins(dut)
        self.rpm_raw, self.speed, self.addr = rpm_raw, speed, addr
        self.out = 1
        self.line = 1
        self.t = 0
        self.idle_since = None
        self.fall = None
        self.wake = None           # time of the wake-up falling edge
        self.started = False
        self.requests = []
        self.errors = []
        self.ui.set(rx_ui, 1)
        self.rx = SerialRx(dut, lambda: self.line, BIT, FMT, strict=False)

    def start(self):
        self.rx.start()
        self._tasks = [cocotb.start_soon(self._wire()), cocotb.start_soon(self._serve())]

    def stop(self):
        self.rx.stop()
        for t in self._tasks:
            t.cancel()

    async def _wire(self):
        while True:
            await RisingEdge(self.dut.clk)
            await ReadOnly()
            self.t += 1
            line = bit(self.dut.uo_out, self.tx) & self.out
            if line != self.line:
                if line == 0:
                    self.fall = self.t
                    idle = self.t - self.idle_since if self.idle_since is not None else 0
                    self._idle_before = idle
                elif self.fall is not None and self.wake is None:
                    low = self.t - self.fall
                    if 24 * MS <= low <= 26 * MS:
                        if self._idle_before < 300 * MS:
                            self.errors.append(f"bus idle only {self._idle_before / MS:.1f} ms before init")
                        self.wake = self.fall
                    elif low > 2 * BIT and self.idle_since is not None:
                        self.errors.append(f"K low for {low / MS:.2f} ms: not a fast init")
                    self.idle_since = self.t
                if line == 1 and self.idle_since is None:
                    self.idle_since = self.t
            self.line = line
            await RisingEdge(self.dut.clk)
            self.t += 1
            self.ui.set(self.rxp, line)

    async def _send(self, msg):
        for i, b in enumerate(msg):
            if i:
                await ClockCycles(self.dut.clk, MS)          # P1
            for v in FMT.logic_bits(b):
                self.out = v
                await ClockCycles(self.dut.clk, round(BIT))
        self.out = 1

    def _reply(self, data):
        m = [0x80 | len(data), 0xF1, self.addr] + list(data)
        return m + [sum(m) & 0xFF]

    async def _serve(self):
        n0, msg, starts, last_reply_end = 0, [], [], None
        while True:
            await ClockCycles(self.dut.clk, 500)
            if self.wake is None:
                n0 = len(self.rx.frames)
                continue
            while n0 < len(self.rx.frames):
                v, ts = self.rx.frames[n0]
                ok = self.rx.ok[n0]
                n0 += 1
                if v is None or not ok:
                    continue
                if not msg:
                    if not self.started and not self.requests:
                        d = ts - self.wake
                        if abs(d - 50 * MS) > MS:
                            self.errors.append(f"first byte {d / MS:.2f} ms after the wake-up edge (TWuP 50)")
                    if last_reply_end is not None and ts - last_reply_end < 55 * MS:
                        self.errors.append(f"P3 {(ts - last_reply_end) / MS:.1f} ms")
                elif not (5 * MS - 2 <= ts - starts[-1] <= 20 * MS):
                    self.errors.append(f"P4 {(ts - starts[-1]) / MS:.2f} ms")
                msg.append(v)
                starts.append(ts)
                if len(msg) >= 1 and len(msg) == (msg[0] & 0x3F) + 4:
                    req, msg, starts = msg, [], []
                    self.requests.append(bytes(req).hex())
                    if (sum(req[:-1]) & 0xFF) != req[-1]:
                        self.errors.append("request checksum")
                        continue
                    data = req[3:-1]
                    if data == [0x81]:
                        self.started = True
                        rep = [0xC1, 0x8F, 0xEF]
                    elif self.started and data == [0x01, 0x0C]:
                        rep = [0x41, 0x0C, self.rpm_raw >> 8, self.rpm_raw & 0xFF]
                    elif self.started and data == [0x01, 0x0D]:
                        rep = [0x41, 0x0D, self.speed]
                    else:
                        continue
                    end_req = self.t
                    await ClockCycles(self.dut.clk, 30 * MS - (self.t - end_req))   # P2
                    await self._send(self._reply(rep))
                    last_reply_end = self.t
                    n0 = len(self.rx.frames)
