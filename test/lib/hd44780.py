# Copyright (c) 2026 Tyler Hong
# SPDX-License-Identifier: Apache-2.0
"""Independent HD44780 model on the 4-bit bus, write only, written from the
Hitachi HD44780U data sheet (bus timing for VCC 4.5-5.5 V: tAS >= 40 ns,
PWEH >= 230 ns, tcycE >= 500 ns, tDSW >= 80 ns, tH >= 10 ns; execution
times 37 us, 41 us for data writes (37 + tADD 4), 1.52 ms for clear;
initialization by instruction: 0x3, wait > 4.1 ms, 0x3, wait > 100 us, 0x3,
0x2; instruction set), not from the RTL.

Event driven: it wakes only when uo_out changes. Instructions that arrive
before the previous one has finished are errors (the busy flag is not read
on a write-only bus)."""

import cocotb

from cocotb.utils import get_sim_time

from .pins import bit


class Hd44780:
    def __init__(self, dut, rs=0, e=1, d4=2):
        self.dut, self.rs, self.e, self.d4 = dut, rs, e, d4
        self.ddram = [0x20] * 0x68
        self.ac, self.inc, self.mode4 = 0, 1, False
        self.lines2 = False
        self.display_on = False
        self.errors = []
        self.init_step = 0
        self.busy_until = 0.0
        self.half = None
        self.last = {}  # signal -> time of its last change (ns)
        self.e_rise = self.e_fall = -1e9

    def start(self):
        self._task = cocotb.start_soon(self._run())

    def stop(self):
        self._task.cancel()

    def text(self, line):
        base = 0x40 * line
        return bytes(self.ddram[base : base + 16]).decode().rstrip()

    def _sig(self):
        try:
            v = [bit(self.dut.uo_out, i) for i in range(6)]
        except ValueError:
            return None
        return {"rs": v[self.rs], "e": v[self.e], "d": sum(v[self.d4 + k] << k for k in range(4))}

    async def _run(self):
        cur = self._sig() or {"rs": 0, "e": 0, "d": 0}
        while True:
            await self.dut.uo_out.value_change
            t = get_sim_time("ns")
            new = self._sig()
            if new is None:
                continue
            for k in ("rs", "d"):
                if new[k] != cur[k]:
                    if cur["e"] == 1:
                        self.errors.append(f"{k} changed while E high at {t:.0f} ns")
                    if t - self.e_fall < 10:
                        self.errors.append(f"hold time {t - self.e_fall:.0f} ns")
                    self.last[k] = t
            if new["e"] != cur["e"]:
                if new["e"] == 1:
                    if t - self.last.get("rs", -1e9) < 40:
                        self.errors.append(f"tAS {t - self.last['rs']:.0f} ns")
                    if t - self.e_rise < 500:
                        self.errors.append(f"tcycE {t - self.e_rise:.0f} ns")
                    self.e_rise = t
                else:
                    if t - self.e_rise < 230:
                        self.errors.append(f"PWEH {t - self.e_rise:.0f} ns")
                    if t - self.last.get("d", -1e9) < 80:
                        self.errors.append(f"tDSW {t - self.last['d']:.0f} ns")
                    self.e_fall = t
                    self._latch(t, new["rs"], new["d"])
            cur = new

    def _latch(self, t, rs, d):
        if self.init_step < 4:  # 8-bit mode instructions, upper nibble only
            want, gap = [(3, 0), (3, 4.1e6), (3, 100e3), (2, 37e3)][self.init_step]
            if rs or d != want:
                self.errors.append(f"init step {self.init_step}: got {d:#x}")
            if self.init_step and t - self.prev_t < gap:
                self.errors.append(f"init step {self.init_step} after {t - self.prev_t:.0f} ns < {gap:.0f}")
            self.prev_t = t
            self.init_step += 1
            if self.init_step == 4:
                self.mode4 = True
                self.busy_until = t + 37e3
            return
        if self.half is None:
            if t < self.busy_until:
                self.errors.append(f"write at {t:.0f} ns while busy until {self.busy_until:.0f}")
            self.half = d
            return
        b = (self.half << 4) | d
        self.half = None
        if rs:
            self.ddram[self.ac % 0x68] = b
            self.ac = (self.ac + self.inc) % 0x68
            self.busy_until = t + 41e3
            return
        self.busy_until = t + (1.52e6 if b in (1, 2, 3) else 37e3)
        if b == 0x01:
            self.ddram = [0x20] * 0x68
            self.ac, self.inc = 0, 1
        elif b & 0x80:
            self.ac = b & 0x7F
        elif b & 0x20:
            if b & 0x10:
                self.errors.append("function set to 8-bit after 4-bit init")
            self.lines2 = bool(b & 0x08)
        elif b & 0x08:
            self.display_on = bool(b & 0x04)
        elif b & 0x04:
            self.inc = 1 if b & 0x02 else -1
