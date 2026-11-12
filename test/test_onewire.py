# Copyright (c) 2026 Tyler Hong
# SPDX-License-Identifier: Apache-2.0
"""1-Wire from timed GPIO only (sw/examples/onewire), judged by an independent
1-Wire device model with the standard-speed timing rules: reset/presence,
READ ROM, 64 read slots, CRC-8 of the ROM checked by the C code."""

import cocotb

from lib.i2c import OpenDrainBus
from lib.onewire import OneWireDevice
from lib.soc import CLK_NS, load, read_shared, start, wait_irq

ROM7 = bytes([0x28, 0xA1, 0xB2, 0xC3, 0xD4, 0xE5, 0xF6])


@cocotb.test()
async def test_onewire_read_rom(dut):
    host = await start(dut)
    bus = OpenDrainBus(dut, pins={6})
    dev = bus.add(OneWireDevice(bus, pin=6, clk_ns=CLK_NS, rom7=ROM7))
    bus.start()
    await load(host, "onewire")
    await host.ctrl(0x01)
    await wait_irq(dut, 1, timeout=400_000)
    bus.stop()

    assert not bus.errors, bus.errors[:5]
    assert not dev.errors, dev.errors[:5]
    present, lo, hi, crc_ok = await read_shared(host, 1, 4)
    rom = (lo | (hi << 32)).to_bytes(8, "little")
    assert present == 1, "no presence pulse seen"
    assert rom == dev.rom, f"ROM {rom.hex()} != {dev.rom.hex()}"
    assert crc_ok == 1, "CRC-8 check in C failed"
