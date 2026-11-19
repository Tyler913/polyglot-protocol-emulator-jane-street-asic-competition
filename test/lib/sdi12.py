# Copyright (c) 2026 Tyler Hong
# SPDX-License-Identifier: Apache-2.0
"""Independent SDI-12 sensor, written from the SDI-12 Specification v1.4,
not from the RTL: one line, 1200 baud 7E1, negative logic (marking = low,
the idle level, held by the bus pull-down; spacing = high). A command starts
after a BREAK (the sensor checks the recorder's spacing is >= 12 ms) and
marking (>= 8.33 ms); characters follow with gaps <= 1.66 ms and end with
'!'. The sensor answers 8.33 ms after the command (within the 15 ms limit),
after checking that the recorder has released the line. Commands: "?!",
"aI!", "aM!" (reply atttn, then a service request "a<CR><LF>" when the
measurement is done), "aD0!"."""

import cocotb
from cocotb.triggers import ClockCycles, ReadOnly, RisingEdge

from .serial import Fmt, SerialRx

FMT = Fmt("7E1", invert=True)
BIT = 50e6 / 1200
MS = 50_000


class Sdi12Sensor:
    def __init__(self, dut, pin, address="0", ident="14EXAMPLESOIL01100SN42",
                 values="+21.5-3.25", ttt=1, sr_ms=30):
        self.dut, self.pin = dut, pin
        self.addr, self.ident, self.values = address, ident, values
        self.ttt, self.sr_ms = ttt, sr_ms
        self.drive = 0            # 1 = sensor drives spacing (high)
        self.line = 0
        self.t = 0
        self.high_since = None
        self.breaks = []          # (spacing length, start)
        self.commands = []
        self.sent = []
        self.errors = []
        self.rx = SerialRx(dut, lambda: self.line, BIT, FMT, strict=False)

    def start(self):
        self.rx.start()
        self._tasks = [cocotb.start_soon(self._wire()), cocotb.start_soon(self._serve())]

    def stop(self):
        self.rx.stop()
        for t in self._tasks:
            t.cancel()

    def chip_drives(self):
        return (int(self.dut.uio_oe.value) >> self.pin) & 1

    async def _wire(self):
        while True:
            await RisingEdge(self.dut.clk)
            await ReadOnly()
            self.t += 1
            oe = (int(self.dut.uio_oe.value) >> self.pin) & 1
            out = (int(self.dut.uio_out.value) >> self.pin) & 1
            line = (oe & out) | self.drive
            if line and not self.line:
                self.high_since = self.t
            if not line and self.line and self.high_since is not None:
                self.breaks.append((self.t - self.high_since, self.high_since))
            self.line = line
            await RisingEdge(self.dut.clk)
            self.t += 1
            v = int(self.dut.uio_in.value) & ~(1 << self.pin) & 0xFF
            self.dut.uio_in.value = v | (line << self.pin)

    async def _send(self, text):
        self.sent.append(text)
        if self.chip_drives():
            self.errors.append("recorder still drives the line when the sensor answers")
        for ch in text.encode():
            for b in FMT.logic_bits(ch):
                self.drive = FMT.line(b)
                await ClockCycles(self.dut.clk, round(BIT))
        self.drive = 0

    async def _serve(self):
        n0 = 0
        while True:
            await ClockCycles(self.dut.clk, 500)
            # a command: a BREAK (>= 12 ms spacing), then characters up to '!'
            brk = [b for b in self.breaks if b[0] >= 12 * MS]
            if not brk:
                continue
            length, t_brk = brk[-1]
            self.breaks.clear()
            if length < 12 * MS:
                self.errors.append(f"break of {length / MS:.2f} ms")
            cmd, first, last_end = "", None, None
            while not cmd.endswith("!"):
                await ClockCycles(self.dut.clk, 500)
                frames = [(v, ts, ok) for (v, ts), ok in zip(self.rx.frames, self.rx.ok) if ts > t_brk]
                frames = [f for f in frames if f[0] is not None]
                if frames and len(cmd) < len(frames):
                    v, ts, ok = frames[len(cmd)]
                    if not ok:
                        self.errors.append("parity/stop error in a command")
                    if first is None:
                        first = ts
                        mark = ts - (t_brk + length)
                        if mark < 8.33 * MS:
                            self.errors.append(f"marking after break {mark / MS:.2f} ms")
                    elif ts - last_end > 1.66 * MS:
                        self.errors.append("gap > 1.66 ms inside a command")
                    last_end = ts + 10 * BIT
                    cmd += chr(v)
                if self.t - t_brk > 200 * MS:
                    break
            self.commands.append(cmd)
            self.rx.frames.clear()
            self.rx.ok.clear()
            await ClockCycles(self.dut.clk, round(8.5 * MS))   # 8.33 ms marking, < 15 ms
            a = self.addr
            if cmd == "?!":
                await self._send(a + "\r\n")
            elif cmd == a + "I!":
                await self._send(a + self.ident + "\r\n")
            elif cmd == a + "M!":
                n = len([c for c in self.values if c in "+-"])
                await self._send(f"{a}{self.ttt:03d}{n}\r\n")
                await ClockCycles(self.dut.clk, self.sr_ms * MS)
                await self._send(a + "\r\n")                   # service request
            elif cmd == a + "D0!":
                await self._send(a + self.values + "\r\n")
            self.rx.frames.clear()
            self.rx.ok.clear()
