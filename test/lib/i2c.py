# Copyright (c) 2026 Tyler Hong
# SPDX-License-Identifier: Apache-2.0
"""Independent I2C peer models, written from the I2C bus specification
(open-drain lines with pull-ups; data changes only while SCL is low; START =
SDA falls while SCL is high; STOP = SDA rises while SCL is high), not from
the RTL.

OpenDrainBus resolves the uio pins every clock: a line is low if the chip
(uio_oe=1, uio_out=0) or any model pulls it low, otherwise the pull-up makes
it high. The chip must never drive a bus line high. Models update their
pull-downs on the falling clock edge from the levels they see.

I2cEeprom: a 24C02-style EEPROM target (page write with a write cycle during
which it ignores its address, sequential/random read, optional clock
stretching) that also checks data setup/hold against SCL.

I2cController: drives SCL/SDA through the bus to test the chip as a target;
it waits while the target stretches SCL.
"""

import cocotb
from cocotb.triggers import ClockCycles, FallingEdge


class OpenDrainBus:
    def __init__(self, dut, pins):
        self.dut = dut
        self.pins = pins  # uio pin numbers that are open-drain bus lines
        self.devices = []
        self.pull = {p: set() for p in pins}  # pin -> names of devices pulling low
        self.level = {p: 1 for p in pins}
        self.t = 0
        self.errors = []
        self.chip_changes = []  # (t, pin, level) when the chip's pull on a line changed
        self._chip_low = {p: False for p in pins}
        self.chip_changed_now = set()
        dut.uio_in.value = 0

    def add(self, dev):
        self.devices.append(dev)
        return dev

    def set_low(self, name, pin, low):
        if low:
            self.pull[pin].add(name)
        else:
            self.pull[pin].discard(name)

    def start(self):
        self._task = cocotb.start_soon(self._run())

    def stop(self):
        self._task.cancel()

    def _resolve(self, out, oe, record=False):
        val = 0
        for p in range(8):
            if p in self.pins:
                chip_low = bool((oe >> p) & 1) and not ((out >> p) & 1)
                if record:
                    if (oe >> p) & 1 and (out >> p) & 1:
                        self.errors.append(f"t={self.t}: chip drives bus line uio[{p}] high")
                    if chip_low != self._chip_low[p]:
                        self.chip_changes.append((self.t, p, 0 if chip_low else 1))
                        self.chip_changed_now.add(p)
                        self._chip_low[p] = chip_low
                lvl = 0 if (chip_low or self.pull[p]) else 1
                self.level[p] = lvl
                val |= lvl << p
            elif (oe >> p) & 1:
                val |= ((out >> p) & 1) << p  # plain output: loop back
        return val

    async def _run(self):
        while True:
            await FallingEdge(self.dut.clk)
            self.t += 1
            out = int(self.dut.uio_out.value)
            oe = int(self.dut.uio_oe.value)
            self.chip_changed_now = set()
            self._resolve(out, oe, record=True)
            for d in self.devices:
                d.step(self.t)
            self.dut.uio_in.value = self._resolve(out, oe)


class I2cEeprom:
    def __init__(self, bus, sda, scl, addr=0x50, page=8, write_cycles=3000,
                 stretch=0, setup_min=4, hold_min=4):
        self.bus, self.sda, self.scl, self.addr = bus, sda, scl, addr
        self.page, self.write_cycles, self.stretch = page, write_cycles, stretch
        self.setup_min, self.hold_min = setup_min, hold_min
        self.name = f"eeprom{addr:#x}"
        self.mem = bytearray(256)
        self.errors = []
        self.log = []  # transactions: list of (kind, bytes)
        self.busy_until = 0
        self.phase = "idle"
        self.pscl, self.psda = 1, 1
        self.t_fall = -1000
        self.t_sda = -1000
        self.stretch_left = 0
        self.my_sda_low = False

    def _drive_sda(self, low):
        self.my_sda_low = low
        self.bus.set_low(self.name, self.sda, low)

    def step(self, t):
        scl, sda = self.bus.level[self.scl], self.bus.level[self.sda]
        if self.stretch_left:
            self.stretch_left -= 1
            self.bus.set_low(self.name + "_scl", self.scl, self.stretch_left > 0)
        rise, fall = scl and not self.pscl, (not scl) and self.pscl
        chip_moved_sda = self.sda in self.bus.chip_changed_now
        if chip_moved_sda and not (scl and self.pscl):
            # Data change by the chip while SCL is low: check hold after SCL fell
            if self.phase != "idle" and t - self.t_fall < self.hold_min:
                self.errors.append(f"t={t}: chip changed SDA {t - self.t_fall} cycles after SCL fell (hold)")
            self.t_sda = t
        if sda != self.psda and scl and self.pscl:
            (self._start if sda == 0 else self._stop)(t)
        if rise:
            if self.phase in ("rx", "addr") and t - self.t_sda < self.setup_min:
                self.errors.append(f"t={t}: chip changed SDA {t - self.t_sda} cycles before SCL rose (setup)")
            self._rise(sda)
        elif fall:
            self.t_fall = t
            self._fall(t)
        self.pscl, self.psda = scl, sda

    def _start(self, t):
        if self.phase in ("rx",) and self.bits > 1:  # a repeated START starts with one SCL rise
            self.errors.append(f"t={t}: START in the middle of a byte")
        self._commit(t)
        self.phase, self.bits, self.sh, self.first = "addr", 0, 0, True
        self._drive_sda(False)
        self.cur = []

    def _stop(self, t):
        self._commit(t)
        if self.phase not in ("idle",):
            self.log.append(("stop", bytes(self.cur)))
        self.phase = "idle"
        self._drive_sda(False)

    def _commit(self, t):
        if getattr(self, "pending", None):
            for a, b in self.pending:
                self.mem[a] = b
            self.pending = None
            self.busy_until = t + self.write_cycles

    def _rise(self, sda):
        if self.phase in ("addr", "rx"):
            self.sh = ((self.sh << 1) | sda) & 0xFF
            self.bits += 1
        elif self.phase == "tx":
            self.bits += 1
        elif self.phase == "ack_in":
            self.master_ack = sda == 0

    def _fall(self, t):
        if self.phase in ("addr", "rx") and self.bits == 8:
            b = self.sh
            self.cur.append(b)
            ack = True
            if self.phase == "addr":
                if (b >> 1) != self.addr or t < self.busy_until:
                    ack = False
                else:
                    self.rw = b & 1
                    self.nbyte = 0
            else:
                if self.nbyte == 0:
                    self.ptr = b
                    self.pending = []
                else:
                    self.pending.append((self.ptr, b))
                    self.ptr = (self.ptr & ~(self.page - 1)) | ((self.ptr + 1) & (self.page - 1))
                self.nbyte += 1
            self._drive_sda(ack)
            self.phase = "ack_out" if ack else "nack_out"
        elif self.phase == "ack_out":
            if self.rw:
                self._load_tx()
            else:
                self._drive_sda(False)
                self.phase, self.bits, self.sh = "rx", 0, 0
            if self.stretch:
                self.stretch_left = self.stretch
                self.bus.set_low(self.name + "_scl", self.scl, True)
        elif self.phase == "nack_out":
            self._drive_sda(False)
            self.phase = "ignore"
        elif self.phase == "tx":
            if self.bits == 8:
                self._drive_sda(False)
                self.phase = "ack_in"
            else:
                self._drive_sda(not ((self.tx >> (7 - self.bits)) & 1))
        elif self.phase == "ack_in":
            if self.master_ack:
                self._load_tx()
            else:
                self._drive_sda(False)
                self.phase = "ignore"

    def _load_tx(self):
        self.tx = self.mem[self.ptr]
        self.ptr = (self.ptr + 1) & 0xFF
        self.cur.append(self.tx)
        self.phase, self.bits = "tx", 0
        self._drive_sda(not ((self.tx >> 7) & 1))


class I2cController:
    """Bit-level I2C controller on the bus. half = SCL half period in cycles."""

    def __init__(self, bus, sda, scl, half=60):
        self.bus, self.sda, self.scl, self.half = bus, sda, scl, half
        self.name = "ctrl"
        self.clk = bus.dut.clk
        self.errors = []

    def _set(self, pin, low):
        self.bus.set_low(self.name, pin, low)

    async def _wait(self, n):
        await ClockCycles(self.clk, n)

    async def _scl_high(self):
        self._set(self.scl, False)
        n = 0
        while self.bus.level[self.scl] == 0:  # target stretching
            await self._wait(1)
            n += 1
            if n > 200_000:
                raise AssertionError("SCL held low for too long")
        await self._wait(self.half)

    async def start(self):
        self._set(self.sda, True)  # SDA falls while SCL is high
        await self._wait(self.half)
        self._set(self.scl, True)
        await self._wait(self.half // 2)

    async def repeated_start(self):
        self._set(self.sda, False)
        await self._wait(self.half // 2)
        await self._scl_high()
        await self.start()

    async def stop(self):
        self._set(self.sda, True)
        await self._wait(self.half // 2)
        await self._scl_high()
        self._set(self.sda, False)
        await self._wait(self.half)

    async def _bit(self, v):
        """Send one bit (1 = release) while SCL is low; return the sampled level."""
        self._set(self.sda, not v)
        await self._wait(self.half // 2)
        await self._scl_high()
        level = self.bus.level[self.sda]
        self._set(self.scl, True)
        await self._wait(self.half // 2)
        return level

    async def write(self, b):
        for i in range(7, -1, -1):
            await self._bit((b >> i) & 1)
        return (await self._bit(1)) == 0  # True = ACK

    async def read(self, ack):
        v = 0
        for _ in range(8):
            v = (v << 1) | await self._bit(1)
        await self._bit(0 if ack else 1)
        return v
