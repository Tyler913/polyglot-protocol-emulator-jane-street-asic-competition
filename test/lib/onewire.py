# Copyright (c) 2026 Tyler Hong
# SPDX-License-Identifier: Apache-2.0
"""Independent 1-Wire device and master models (DS18B20-style ROM), from the Maxim 1-Wire
timing specification (standard speed): reset low >= 480 us; presence pulse
15-60 us after release, 60-240 us long; write-1 low 1-15 us, write-0 low
60-120 us, slot >= 60 us, recovery >= 1 us; the device samples a write slot
15-60 us after it starts and holds the line low (read 0) for ~30 us.
Runs on OpenDrainBus (test/lib/i2c.py)."""


from cocotb.triggers import ClockCycles


def crc8_maxim(data):
    c = 0
    for x in data:
        for _ in range(8):
            mix = (c ^ x) & 1
            c >>= 1
            if mix:
                c ^= 0x8C
            x >>= 1
    return c


class OneWireDevice:
    def __init__(self, bus, pin, clk_ns, rom7):
        self.bus, self.pin, self.ns = bus, pin, clk_ns
        self.rom = bytes(rom7) + bytes([crc8_maxim(rom7)])
        self.name = "ow"
        self.errors = []
        self.plev = 1
        self.fall = None
        self.rise = None
        self.state = "idle"
        self.rx_bits = []
        self.tx_bits = []
        self.hold_until = None
        self.presence = None  # (start, end) cycles
        self.slot_starts = []

    def us(self, cycles):
        return cycles * self.ns / 1000

    def cyc(self, us):
        return round(us * 1000 / self.ns)

    def step(self, t):
        if self.hold_until is not None and t >= self.hold_until:
            self.bus.set_low(self.name, self.pin, False)
            self.hold_until = None
            self.fall = None          # our own release is not a master edge
        if self.presence and t == self.presence[0]:
            self.bus.set_low(self.name, self.pin, True)
            self.hold_until = self.presence[1]
            self.presence = None
            self.fall = None
        lev = self.bus.level[self.pin]
        mine = self.name in self.bus.pull[self.pin]
        if self.plev and not lev and not mine:          # master pulls low: slot or reset start
            if self.rise is not None and self.us(t - self.rise) < 1:
                self.errors.append(f"t={t}: recovery {self.us(t - self.rise):.1f} us < 1 us")
            if self.slot_starts and self.state == "rom" and self.us(t - self.slot_starts[-1]) < 60:
                self.errors.append(f"t={t}: slot {self.us(t - self.slot_starts[-1]):.1f} us < 60 us")
            self.fall = t
            self.slot_starts.append(t)
            if self.state == "rom" and self.tx_bits:     # read slot: answer a 0 by holding low
                b = self.tx_bits.pop(0)
                if b == 0:
                    self.bus.set_low(self.name, self.pin, True)
                    self.hold_until = t + self.cyc(30)
        elif not self.plev and lev and self.fall is not None and not mine:   # master releases
            low = self.us(t - self.fall)
            self.rise = t
            if low >= 480:
                self.state = "cmd"
                self.rx_bits = []
                self.presence = (t + self.cyc(30), t + self.cyc(30 + 120))
            elif low > 120 and low < 480:
                self.errors.append(f"t={t}: low pulse of {low:.1f} us (neither slot nor reset)")
            elif self.state == "cmd":
                if 1 <= low <= 15:
                    self.rx_bits.append(1)
                elif 60 <= low <= 120:
                    self.rx_bits.append(0)
                else:
                    self.errors.append(f"t={t}: write slot low for {low:.1f} us")
                if len(self.rx_bits) == 8:
                    cmd = sum(b << i for i, b in enumerate(self.rx_bits))
                    if cmd == 0x33:
                        self.state = "rom"
                        self.tx_bits = [(x >> i) & 1 for x in self.rom for i in range(8)]
                    else:
                        self.errors.append(f"unexpected command {cmd:#04x}")
                        self.state = "idle"
            elif self.state == "rom" and not 1 <= low <= 15 and low < 30:
                self.errors.append(f"t={t}: read slot low for {low:.1f} us")
        self.plev = lev


class OneWireMaster:
    """Independent 1-Wire master (standard speed) on OpenDrainBus, from the same
    Maxim timing: reset 480 us low, presence sampled 70 us after release
    (and its length measured), write-1 6 us low / 64 us high, write-0 60 us low
    / 10 us high, read: 6 us low, sample at 15 us, slot 70 us. ROM search as
    in Maxim application note 187."""

    def __init__(self, bus, pin, clk_ns):
        self.bus, self.pin, self.ns = bus, pin, clk_ns
        self.name = "master"
        self.clk = bus.dut.clk
        self.presence_us = []

    def step(self, t):
        pass

    def cyc(self, us):
        return round(us * 1000 / self.ns)

    async def _low(self, us):
        self.bus.set_low(self.name, self.pin, True)
        await ClockCycles(self.clk, self.cyc(us))
        self.bus.set_low(self.name, self.pin, False)

    async def reset(self):
        await self._low(480)
        present, low_from, n = False, None, 0
        for k in range(self.cyc(480)):
            await ClockCycles(self.clk, 1)
            lv = self.bus.level[self.pin]
            if lv == 0 and low_from is None:
                low_from = k
            if lv == 1 and low_from is not None and n == 0:
                n = k - low_from
            if k == self.cyc(70):
                present = lv == 0
        if low_from is not None:
            self.presence_us.append((low_from * self.ns / 1000, n * self.ns / 1000))
        return present

    async def write_bit(self, b):
        if b:
            await self._low(6)
            await ClockCycles(self.clk, self.cyc(64))
        else:
            await self._low(60)
            await ClockCycles(self.clk, self.cyc(10))

    async def read_bit(self):
        await self._low(6)
        await ClockCycles(self.clk, self.cyc(9))
        b = self.bus.level[self.pin]
        await ClockCycles(self.clk, self.cyc(55))
        while self.bus.level[self.pin] == 0:  # recovery: wait until released
            await ClockCycles(self.clk, 1)
        await ClockCycles(self.clk, self.cyc(2))
        return b

    async def write_byte(self, v):
        for i in range(8):
            await self.write_bit((v >> i) & 1)

    async def read_bytes(self, n):
        out = []
        for _ in range(n):
            v = 0
            for i in range(8):
                v |= (await self.read_bit()) << i
            out.append(v)
        return bytes(out)

    async def search(self):
        """One pass of SEARCH ROM taking the 0 branch at conflicts; returns the ROM or None."""
        await self.write_byte(0xF0)
        bits = []
        for _ in range(64):
            b, nb = await self.read_bit(), await self.read_bit()
            if b and nb:
                return None  # no device
            d = 0 if (not b and not nb) else b
            await self.write_bit(d)
            bits.append(d)
        return bytes(sum(bits[8 * k + j] << j for j in range(8)) for k in range(8))
