# Copyright (c) 2026 Tyler Hong
# SPDX-License-Identifier: Apache-2.0
"""The chip as a 1-Wire thermometer (sw/examples/onewire_slave) against an
independent 1-Wire master (test/lib/onewire.py): reset and presence timing,
READ ROM, SEARCH ROM (64 bit / complement / direction triplets), SKIP ROM +
READ SCRATCHPAD and MATCH ROM, each checked with the Maxim CRC8."""

import cocotb
from cocotb.triggers import ClockCycles

from lib.i2c import OpenDrainBus
from lib.onewire import OneWireMaster, crc8_maxim
from lib.soc import CLK_NS, load, read_shared, start

ROM = bytes([0x28, 0x5A, 0x33, 0x01, 0x80, 0x00, 0x00])


@cocotb.test()
async def test_onewire_slave(dut):
    host = await start(dut)
    bus = OpenDrainBus(dut, pins={6})
    m = bus.add(OneWireMaster(bus, pin=6, clk_ns=CLK_NS))
    bus.start()
    await load(host, "onewire_slave")
    await host.ctrl(0x01)
    await ClockCycles(dut.clk, 5000)

    assert await m.reset(), "no presence pulse"
    await m.write_byte(0x33)                       # READ ROM
    rom = await m.read_bytes(8)
    assert rom[:7] == ROM and crc8_maxim(rom[:7]) == rom[7], rom.hex()

    assert await m.reset()
    found = await m.search()                       # SEARCH ROM
    assert found == rom, f"search found {found and found.hex()}"

    assert await m.reset()
    await m.write_byte(0xCC)                       # SKIP ROM
    await m.write_byte(0xBE)                       # READ SCRATCHPAD
    sp = await m.read_bytes(9)
    assert crc8_maxim(sp[:8]) == sp[8], sp.hex()
    t = int.from_bytes(sp[:2], "little", signed=True) / 16
    assert t == 25.0625, t

    assert await m.reset()
    await m.write_byte(0x55)                       # MATCH ROM with our ROM
    for b in rom:
        await m.write_byte(b)
    await m.write_byte(0xBE)
    assert await m.read_bytes(9) == sp

    bus.stop()
    assert not bus.errors, bus.errors[:5]
    for start_us, length_us in m.presence_us:
        assert 15 <= start_us <= 60 and 60 <= length_us <= 240, m.presence_us
    r = await read_shared(host, 0, 3)
    assert r == [4, 4, 0xBE], r
