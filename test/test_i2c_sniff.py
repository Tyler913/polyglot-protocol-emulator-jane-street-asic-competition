# Copyright (c) 2026 Tyler Hong
# SPDX-License-Identifier: Apache-2.0
"""Passive I2C bus analyzer (sw/examples/i2c_sniff): a model controller and a
model EEPROM (test/lib/i2c.py) talk at 400 kHz; the chip only listens and
must log every START, repeated START, byte, ACK/NACK and STOP."""

import cocotb
from cocotb.triggers import ClockCycles

from lib.i2c import I2cController, I2cEeprom, OpenDrainBus
from lib.soc import load, read_shared, start, wait_irq

S, P = 0x1000, 0x2000


def b(x, nack=0):
    return (x << 1) | nack


@cocotb.test()
async def test_i2c_sniffer(dut):
    host = await start(dut)
    bus = OpenDrainBus(dut, pins={2, 3})
    ee = bus.add(I2cEeprom(bus, sda=2, scl=3, write_cycles=2000))
    bus.start()
    await load(host, "i2c_sniff")
    await host.ctrl(0x01)
    await wait_irq(dut, 1, timeout=10_000)
    c = I2cController(bus, sda=2, scl=3, half=62)  # 400 kHz

    await c.start()
    for x in (0xA0, 0x10, 0xAA, 0x55):
        assert await c.write(x)
    await c.stop()
    await ClockCycles(dut.clk, 3000)  # EEPROM write cycle
    await c.start()
    assert await c.write(0xA0)
    assert await c.write(0x10)
    await c.repeated_start()
    assert await c.write(0xA1)
    r0 = await c.read(True)
    r1 = await c.read(False)
    await c.stop()
    await c.start()
    assert not await c.write(0x66)  # nobody at 0x33
    await c.stop()
    await ClockCycles(dut.clk, 500)
    assert (r0, r1) == (0xAA, 0x55)
    assert not ee.errors, ee.errors

    n, addr = await read_shared(host, 0, 2)
    raw = await host.read(addr, 2 * n)
    got = [int.from_bytes(raw[2 * i : 2 * i + 2], "little") for i in range(n)]
    want = [S, b(0xA0), b(0x10), b(0xAA), b(0x55), P,
            S, b(0xA0), b(0x10), S, b(0xA1), b(0xAA), b(0x55, 1), P,
            S, b(0x66, 1), P]
    assert got == want, [hex(x) for x in got]
