# Copyright (c) 2026 Tyler Hong
# SPDX-License-Identifier: Apache-2.0
"""SMBus with Packet Error Checking (sw/examples/smbus): the I2C target model
(test/lib/i2c.py, an EEPROM: pointer then data) stands in for the device.
The PEC bytes the chip writes are checked against a CRC-8 computed here from
the SMBus 3.x definition; for the reads the device memory is preloaded with
a correct PEC at one register and a wrong one at another."""

import cocotb

from lib.i2c import I2cEeprom, OpenDrainBus
from lib.soc import load, read_shared, start, wait_irq


def crc8(data):
    c = 0
    for b in data:
        c ^= b
        for _ in range(8):
            c = ((c << 1) ^ 0x07) & 0xFF if c & 0x80 else (c << 1) & 0xFF
    return c


@cocotb.test()
async def test_smbus_pec(dut):
    host = await start(dut)
    bus = OpenDrainBus(dut, pins={0, 1})
    ee = bus.add(I2cEeprom(bus, sda=0, scl=1, write_cycles=5000))
    a_w, a_r = 0x50 << 1, (0x50 << 1) | 1
    ee.mem[0x30] = 0x5C
    ee.mem[0x31] = crc8([a_w, 0x30, a_r, 0x5C])          # correct PEC
    ee.mem[0x40] = 0x77
    ee.mem[0x41] = crc8([a_w, 0x40, a_r, 0x77]) ^ 0x01   # corrupted PEC
    bus.start()
    await load(host, "smbus")
    await host.ctrl(0x01)
    await wait_irq(dut, 1, timeout=1_000_000)
    bus.stop()
    assert not bus.errors, bus.errors[:5]
    assert not ee.errors, ee.errors[:5]
    assert ee.mem[0x10] == 0xA5 and ee.mem[0x11] == crc8([a_w, 0x10, 0xA5]), ee.mem[0x10:0x12].hex()
    assert ee.mem[0x20:0x22] == b"\x34\x12" and ee.mem[0x22] == crc8([a_w, 0x20, 0x34, 0x12]), ee.mem[0x20:0x23].hex()
    r = await read_shared(host, 0, 5)
    assert r == [1, 0x5C, 1, 0, 0x600D5B05], [hex(x) for x in r]
