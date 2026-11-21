# Copyright (c) 2026 Tyler Hong
# SPDX-License-Identifier: Apache-2.0
"""Independent MIPI I3C (SDR) target model, written from the MIPI I3C Basic
specification's public description, not from our RTL.

Bus: SDA and SCL with pull-ups (open drain) plus push-pull drivers. START =
SDA falls while SCL is high, STOP = SDA rises while SCL is high. After a
START the address header (7-bit address + RnW) and its ACK are open drain:
the controller must never drive SDA high there. Data words are push-pull: 8
data bits MSB first and a T bit. A controller write's T bit is odd parity
(the 9 bits hold an odd number of ones); in a read the target sends T = 1 if
more data follows and T = 0 on the last byte. Data changes while SCL is low.

Common Command Codes handled: broadcast (address 7E) RSTDAA 0x06; direct
SETDASA 0x87 (static address -> dynamic address, data = dynamic address << 1),
GETPID 0x8D (6 bytes), GETBCR 0x8E, GETDCR 0x8F. Private write to the
dynamic address: register pointer then data; private read: 4 bytes from the
pointer.

I3cBus resolves uio[sda], uio[scl] every clock and records protocol errors:
contention (both ends driving different levels), the controller driving SDA
high in an open-drain phase, and data words that were not push-pull.
"""

import cocotb
from cocotb.triggers import FallingEdge

from .pins import bit


def odd_t(x):
    return 1 - (bin(x).count("1") & 1)


class I3cTarget:
    def __init__(self, static_addr, pid=0x0123456789AB, bcr=0x06, dcr=0x44):
        self.static, self.dyn = static_addr, None
        self.pid, self.bcr, self.dcr = pid, bcr, dcr
        self.regs = bytearray(range(0x40, 0x80))
        self.ptr = 0
        self.log = []          # (kind, detail)


class I3cBus:
    def __init__(self, dut, sda, scl, target):
        self.dut, self.sda, self.scl, self.t = dut, sda, scl, target
        self.errors = []
        self.drive = None      # target's SDA drive: None, 0 or 1
        self.prev = (1, 1)
        self.state = "idle"
        self.words = []        # completed words of the current message
        self.cnt, self.acc = 0, 0
        self.pp_ok = True
        self.chip_hi = False
        self.ccc = None
        self.mode = None

    def start(self):
        self._task = cocotb.start_soon(self._run())

    def stop(self):
        self._task.cancel()

    def _err(self, msg):
        if not self.errors or self.errors[-1] != msg:
            self.errors.append(msg)

    async def _run(self):
        dut = self.dut
        while True:
            await FallingEdge(dut.clk)
            out, oe = int(dut.uio_out.value), int(dut.uio_oe.value)
            c_sda = ((out >> self.sda) & 1) if (oe >> self.sda) & 1 else None
            c_scl = ((out >> self.scl) & 1) if (oe >> self.scl) & 1 else None
            if c_sda is not None and self.drive is not None and c_sda != self.drive:
                self._err(f"SDA contention in {self.state}")
            sda = c_sda if c_sda is not None else (self.drive if self.drive is not None else 1)
            scl = c_scl if c_scl is not None else 1
            v = int(dut.uio_in.value)
            v = (v & ~((1 << self.sda) | (1 << self.scl))) | (sda << self.sda) | (scl << self.scl)
            dut.uio_in.value = v
            self._step(sda, scl, c_sda)
            self.prev = (sda, scl)

    # ---- protocol ----------------------------------------------------------
    def _step(self, sda, scl, c_sda):
        psda, pscl = self.prev
        if scl and pscl and psda and not sda:              # START / repeated START
            self._end_message()
            self.state, self.cnt, self.acc, self.drive = "hdr", 0, 0, None
            self.pp_ok, self.chip_hi = True, False
            return
        if scl and pscl and not psda and sda:              # STOP
            self._end_message()
            self.state, self.drive = "idle", None
            self.ccc = None
            return
        if self.state in ("hdr", "ack", "ack_bit") and c_sda == 1:
            self.chip_hi = True                            # controller drove SDA high
        if scl and not pscl:                               # rising SCL: sample
            if self.state == "data_w" and sda == 1 and c_sda != 1:
                self.pp_ok = False                         # a 1 left to the pull-up
            self._rise(sda)
        elif pscl and not scl:                             # falling SCL: targets change SDA
            self._fall()

    def _rise(self, sda):
        if self.state == "hdr":
            self.acc = (self.acc << 1) | sda
            self.cnt += 1
            if self.cnt == 8:
                if self.chip_hi:
                    self._err("controller drove SDA high in the open-drain header")
                addr, rnw = self.acc >> 1, self.acc & 1
                self.addr, self.rnw = addr, rnw
                self.ack = addr == 0x7E and not rnw or addr == self.t.dyn and self.t.dyn is not None or \
                    (addr == self.t.static and self.ccc == 0x87)
                self.state = "ack"
        elif self.state == "ack_bit":
            if self.ack:
                self.mode = "r" if self.rnw else "w"
                self.cnt, self.acc, self.chip_hi = 0, 0, False
                self.state = "data_r" if self.rnw else "data_w"
                if self.rnw:
                    self.rbytes = self._read_data()
                    self.ri, self.rbit = 0, 0
                self.words = []
            else:
                self.state = "nack"
            if self.chip_hi:
                self._err("controller drove SDA high in the open-drain ACK")
        elif self.state == "data_w":
            self.acc = (self.acc << 1) | sda
            self.cnt += 1
            if self.cnt == 9:
                x, t = self.acc >> 1, self.acc & 1
                if t != odd_t(x):
                    self._err(f"bad T (parity) bit after {x:#04x}")
                if not self.pp_ok:
                    self._err(f"data word {x:#04x} not push-pull")
                self.words.append(x)
                self.cnt, self.acc, self.pp_ok = 0, 0, True
        elif self.state == "data_r":
            pass

    def _fall(self):
        if self.state == "ack":
            self.drive = 0 if self.ack else None
            self.state = "ack_bit"
        elif self.state == "ack_bit":
            pass
        elif self.state in ("data_w", "hdr") and self.drive is not None:
            self.drive = None
        elif self.state == "data_r":
            if self.ri < len(self.rbytes):
                x = self.rbytes[self.ri]
                if self.rbit < 8:
                    self.drive = (x >> (7 - self.rbit)) & 1
                    self.rbit += 1
                else:                                       # T bit: 1 = more, 0 = last byte
                    last = self.ri == len(self.rbytes) - 1
                    self.drive = 0 if last else 1
                    self.rbit = 0
                    self.ri += 1
            else:
                self.drive = None                           # after T = 0: controller ends

    def _read_data(self):
        c = self.ccc
        if c == 0x8D:
            return list(self.t.pid.to_bytes(6, "big"))
        if c == 0x8E:
            return [self.t.bcr]
        if c == 0x8F:
            return [self.t.dcr]
        r = [self.t.regs[(self.t.ptr + i) % len(self.t.regs)] for i in range(4)]
        self.t.ptr = (self.t.ptr + 4) % len(self.t.regs)
        return r

    def _end_message(self):
        if self.state in ("data_w", "data_r") and self.mode:
            if self.mode == "w":
                w = self.words
                if self.addr == 0x7E:
                    if w:
                        self.ccc = w[0]
                        if w[0] == 0x06:
                            self.t.dyn = None
                        self.t.log.append(("ccc", w[0], w[1:]))
                elif self.ccc == 0x87 and self.addr == self.t.static:
                    if w:
                        self.t.dyn = w[0] >> 1
                        self.t.log.append(("setdasa", self.t.dyn))
                elif self.addr == self.t.dyn:
                    if w:
                        self.t.ptr = w[0] % len(self.t.regs)
                        for i, x in enumerate(w[1:]):
                            self.t.regs[(self.t.ptr + i) % len(self.t.regs)] = x
                    self.t.log.append(("write", list(w)))
            else:
                self.t.log.append(("read", self.addr, self.ccc))
        self.mode = None
        self.drive = None
