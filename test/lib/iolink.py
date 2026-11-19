# Copyright (c) 2026 Tyler Hong
# SPDX-License-Identifier: Apache-2.0
"""Independent IO-Link device (IEC 61131-9 / IO-Link Interface Spec V1.1.x),
written from the specification's rules, not from the RTL or the C code:

- C/Q is one wire: line = master TxD AND device output (both idle high);
  the chip's RxD follows the line.
- Wake-up request: the line low for 75..85 us after idle; anything else
  (e.g. the long low before the program runs) does not wake the device.
- The device talks only at COM2 (38.4 kbaud, 8E1); frames with parity or
  stop errors (e.g. a master trying COM3 first) make the message invalid.
- Checksum (A.1.6): XOR of all octets with the checksum bits zero and the
  seed 0x52, 8 bits compressed to 6 with the A.1 equations.
- STARTUP / PREOPERATE: TYPE_0 messages (read: MC, CKT -> OD, CKS; write:
  MC, CKT, OD -> CKS) on the page channel; MasterCommand 0x9A
  (DevicePreoperate), 0x99 / 0x98 (DeviceOperate / ProcessDataOutputOperate).
- OPERATE: TYPE_2 messages, reply OD + process data in + CKS; CKS bit 6 =
  PD invalid (0 here), bit 7 = event (0).
The device answers 3 bit times after the master's last octet."""

import cocotb
from cocotb.triggers import ClockCycles, ReadOnly, RisingEdge

from .pins import bit, ui_pins
from .serial import Fmt, SerialRx

FMT = Fmt("8E1")
BIT = 50e6 / 38400


def ck6(octets):
    c = 0x52
    for o in octets:
        c ^= o
    d = [(c >> i) & 1 for i in range(8)]
    return ((d[7] ^ d[5] ^ d[3] ^ d[1]) << 5 | (d[6] ^ d[4] ^ d[2] ^ d[0]) << 4 |
            (d[7] ^ d[6]) << 3 | (d[5] ^ d[4]) << 2 | (d[3] ^ d[2]) << 1 | (d[1] ^ d[0]))


class IoLinkDevice:
    def __init__(self, dut, txd_uo, rxd_ui, page, pd_values):
        self.dut, self.txd, self.rxd = dut, txd_uo, rxd_ui
        self.ui = ui_pins(dut)
        self.page = dict(page)           # address -> octet (Direct Parameter page 1)
        self.pd_values = list(pd_values)  # process data (bytes) per OPERATE cycle
        self.out = 1
        self.line = 1
        self.t = 0
        self.awake_at = None
        self.state = "SIO"
        self.log = []                    # (state, message hex, reply hex or None)
        self.errors = []
        self.master_cycle = None
        self.operate_times = []
        self.ui.set(rxd_ui, 1)
        self.rx = SerialRx(dut, lambda: self.line, BIT, FMT, strict=False)

    def start(self):
        self.rx.start()
        self._tasks = [cocotb.start_soon(self._wire()), cocotb.start_soon(self._dev())]

    def stop(self):
        self.rx.stop()
        for t in self._tasks:
            t.cancel()

    async def _wire(self):
        low_since, seen_idle = None, False
        while True:
            await RisingEdge(self.dut.clk)
            await ReadOnly()
            self.t += 1
            m = bit(self.dut.uo_out, self.txd)
            line = m & self.out
            if self.state == "SIO":                  # wake-up detection on the master's TxD
                if m == 1:
                    if low_since is not None and seen_idle:
                        w = (self.t - low_since) / 50   # us
                        if 75 <= w <= 85:
                            self.state = "STARTUP"
                            self.awake_at = self.t
                        elif w > 2:
                            self.errors.append(f"line low for {w:.1f} us: not a WURQ")
                    if low_since is None and self.t > 50:
                        seen_idle = True
                    low_since = None
                elif low_since is None:
                    low_since = self.t
            self.line = line
            await RisingEdge(self.dut.clk)
            self.t += 1
            self.ui.set(self.rxd, self.line)

    async def _send(self, octets):
        for o in octets:
            for b in FMT.logic_bits(o):
                self.out = b
                await ClockCycles(self.dut.clk, round(BIT))
        self.out = 1

    def _expect(self, mc):
        """Master message length for this state and MC."""
        if self.state == "OPERATE":
            return 2 if mc & 0x80 else 3
        return 2 if mc & 0x80 else 3

    async def _dev(self):
        n0 = 0
        msg, ok = [], True
        last = 0
        while True:
            await ClockCycles(self.dut.clk, 64)
            if self.state == "SIO":
                n0 = len(self.rx.frames)
                continue
            frames = self.rx.frames
            while n0 < len(frames):
                v, ts = frames[n0]
                good = self.rx.ok[n0]
                n0 += 1
                if self.out == 0 or ts < self.awake_at:
                    continue
                if msg and ts - last > 14 * BIT:     # too long a gap: start over
                    msg, ok = [], True
                last = ts
                msg.append(v if v is not None else 0)
                ok &= good and v is not None
                if len(msg) >= 2 and len(msg) == self._expect(msg[0]):
                    await self._handle(msg, ok)
                    msg, ok = [], True
                    n0 = len(self.rx.frames)          # skip our own reply's echo

    async def _handle(self, msg, ok):
        mc, ckt = msg[0], msg[1]
        mtype = ckt >> 6
        check = list(msg)
        check[1] = ckt & 0xC0
        if not ok or ck6(check) != (ckt & 0x3F):
            self.log.append((self.state, bytes(msg).hex(), None))
            return
        want_type = 2 if self.state == "OPERATE" else 0
        if mtype != want_type:
            self.errors.append(f"M-sequence type {mtype} in {self.state}")
            self.log.append((self.state, bytes(msg).hex(), None))
            return
        read, chan, addr = mc >> 7, (mc >> 5) & 3, mc & 0x1F
        if chan != 1:
            self.errors.append(f"channel {chan} not supported")
            return
        await ClockCycles(self.dut.clk, round(3 * BIT))
        state0 = self.state
        if read:
            reply = [self.page.get(addr, 0)]
            if self.state == "OPERATE":
                k = len(self.operate_times)
                self.operate_times.append(self.t)
                reply += list(self.pd_values[k % len(self.pd_values)])
        else:
            od = msg[2]
            reply = []
            if addr == 0x00:
                if od == 0x9A and self.state in ("STARTUP", "PREOPERATE"):
                    self.state = "PREOPERATE"
                elif od in (0x99, 0x98) and self.state == "PREOPERATE":
                    self.state = "OPERATE"
                else:
                    self.errors.append(f"MasterCommand {od:#x} in {self.state}")
            elif addr == 0x01:
                self.master_cycle = od
        cks = 0x00                                   # no event, PD valid
        reply.append(cks | ck6(reply + [cks]))
        self.log.append((state0, bytes(msg).hex(), bytes(reply).hex()))
        await self._send(reply)
