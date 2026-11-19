# Copyright (c) 2026 Tyler Hong
# SPDX-License-Identifier: Apache-2.0
"""I2C built from Wire Engines, judged by independent I2C models on an
open-drain bus model.

- Controller (sw/examples/i2c_eeprom) against a 24C02-style EEPROM: absent
  address NACK, page write, ACK polling during the write cycle, random read
  with repeated START; with and without clock stretching by the EEPROM.
- Target (sw/examples/i2c_target), driven by an I2C controller model:
  register writes, pointer write + repeated START + read, a NACK for another
  address; the chip may change SDA only while SCL is low.
- Fast-mode Plus (1 MHz) both ways; as a controller the SCL low and high
  times are checked against the Fm+ minimums (0.5 us, 0.26 us).
"""

import cocotb
from cocotb.triggers import ClockCycles

from lib.i2c import I2cController, I2cEeprom, OpenDrainBus
from lib.soc import load, read_shared, start, wait_irq

DATA = bytes((0xA0 + 7 * i) & 0xFF for i in range(8))


class TargetSdaCheck:
    """The target may change SDA only while SCL is low."""

    def __init__(self, bus, sda, scl):
        self.bus, self.sda, self.scl = bus, sda, scl
        self.pscl = 1
        self.errors = []

    def step(self, t):
        scl = self.bus.level[self.scl]
        if self.sda in self.bus.chip_changed_now and scl and self.pscl:
            self.errors.append(f"t={t}: target changed SDA while SCL was high")
        self.pscl = scl


class SclTiming:
    """Shortest SCL low and high times while the chip drives the clock."""

    def __init__(self, bus, scl):
        self.bus, self.scl = bus, scl
        self.prev, self.since, self.low, self.high = 1, 0, [], []

    def step(self, t):
        v = self.bus.level[self.scl]
        if v != self.prev:
            (self.low if self.prev == 0 else self.high).append(t - self.since)
            self.since, self.prev = t, v


async def eeprom_case(dut, hz, stretch):
    host = await start(dut)
    bus = OpenDrainBus(dut, pins={0, 1})
    scl = bus.add(SclTiming(bus, scl=1))
    # Write cycle long enough for at least one NACKed poll (a real one is ~5 ms)
    ee = bus.add(I2cEeprom(bus, sda=0, scl=1, stretch=stretch, write_cycles=30 * 50_000_000 // hz))
    bus.start()
    await load(host, "i2c_eeprom", [hz])
    await host.ctrl(0x01)
    await wait_irq(dut, 1, timeout=600_000)
    bus.stop()

    assert not bus.errors, bus.errors[:5]
    assert not ee.errors, ee.errors[:5]
    absent_ack, polls, result = await read_shared(host, 2, 3)
    back = b"".join(w.to_bytes(4, "little") for w in await read_shared(host, 8, 2))
    dut._log.info(f"{hz} Hz, stretch {stretch}: polls {polls}, log {ee.log[:4]}")
    assert absent_ack == 0, "an absent address was acknowledged"
    assert polls >= 2, f"expected ACK polling during the write cycle, got {polls} poll(s)"
    assert bytes(ee.mem[0x10:0x18]) == DATA, f"EEPROM content {ee.mem[0x10:0x18].hex()}"
    assert back == DATA, f"read back {back.hex()}"
    assert result == 0x600D0000, f"firmware result {result:#010x}"
    return scl


@cocotb.test()
async def test_i2c_eeprom_400k(dut):
    await eeprom_case(dut, 400_000, stretch=0)


@cocotb.test()
async def test_i2c_eeprom_100k_stretch(dut):
    """The EEPROM holds SCL low for 300 cycles after every ACK."""
    await eeprom_case(dut, 100_000, stretch=300)


@cocotb.test()
async def test_i2c_eeprom_1m(dut):
    """Fast-mode Plus: 1 MHz SCL, Fm+ low/high minimums."""
    scl = await eeprom_case(dut, 1_000_000, stretch=0)
    lo, hi = min(scl.low), min(scl.high[1:])     # the first 'high' is the idle bus
    dut._log.info("Fm+ SCL: shortest low %d, high %d cycles", lo, hi)
    assert lo >= 25 and hi >= 13, f"SCL low {lo}, high {hi} cycles (Fm+: >= 25, >= 13)"


@cocotb.test()
async def test_i2c_target(dut):
    await target_case(dut, half=62)              # 400 kHz


@cocotb.test()
async def test_i2c_target_1m(dut):
    await target_case(dut, half=25)              # 1 MHz (Fm+)


async def target_case(dut, half):
    host = await start(dut)
    bus = OpenDrainBus(dut, pins={2, 3})
    chk = bus.add(TargetSdaCheck(bus, sda=2, scl=3))
    bus.start()
    await load(host, "i2c_target")
    await host.ctrl(0x01)
    await wait_irq(dut, 1, timeout=5_000)
    c = I2cController(bus, sda=2, scl=3, half=half)
    await ClockCycles(dut.clk, 50)

    # Write registers 3..5
    await c.start()
    assert await c.write(0x42 << 1), "no ACK for our address (write)"
    assert await c.write(3), "no ACK for the register number"
    for b in (0x11, 0x22, 0x33):
        assert await c.write(b), f"no ACK for data {b:#x}"
    await c.stop()

    # Another address: NACK
    await c.start()
    assert not await c.write(0x43 << 1), "another address was acknowledged"
    await c.stop()

    # Pointer to 2, repeated START, read 4 bytes
    await c.start()
    assert await c.write(0x42 << 1)
    assert await c.write(2)
    await c.repeated_start()
    assert await c.write((0x42 << 1) | 1), "no ACK for our address (read)"
    got = [await c.read(ack=i < 3) for i in range(4)]
    await c.stop()
    await ClockCycles(dut.clk, 200)
    bus.stop()

    assert not bus.errors, bus.errors[:5]
    assert not chk.errors, chk.errors[:5]
    assert got == [0x32, 0x11, 0x22, 0x33], f"read {[hex(x) for x in got]}"
    done, other = await read_shared(host, 1, 2)
    regs = b"".join(w.to_bytes(4, "little") for w in await read_shared(host, 8, 4))
    assert regs[2:6] == bytes([0x32, 0x11, 0x22, 0x33]), f"registers {regs.hex()}"
    assert (done, other) == (3, 1), f"transactions {done}, other-address {other}"
