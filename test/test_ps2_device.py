# Copyright (c) 2026 Tyler Hong
# SPDX-License-Identifier: Apache-2.0
"""The chip as a PS/2 keyboard (sw/examples/ps2_device) against an independent
PS/2 host model (test/lib/ps2_host.py): scan codes from the device, then the
host sends Set LEDs (0xED) and its argument with inhibit and request to send;
the device clocks them in, acknowledges each and answers 0xFA."""

import cocotb
from cocotb.triggers import ClockCycles

from lib.i2c import OpenDrainBus
from lib.ps2_host import Ps2Host
from lib.soc import CLK_NS, load, read_shared, start, wait_irq


@cocotb.test()
async def test_ps2_device(dut):
    host = await start(dut)
    bus = OpenDrainBus(dut, pins={2, 3})
    pc = bus.add(Ps2Host(bus, clk=2, data=3, clk_ns=CLK_NS))
    bus.start()
    await load(host, "ps2_device")
    await host.ctrl(0x01)
    for _ in range(400):
        await ClockCycles(dut.clk, 5000)
        if len(pc.received) >= 6:
            break
    pc.send(0xED)
    pc.send(0x02)
    try:
        await wait_irq(dut, 1, timeout=1_000_000)
    finally:
        dut._log.info(f"received {[hex(x) for x in pc.received]} acked {pc.acked} state {pc.state} "
                      f"errors {pc.errors[:4]}")
    await ClockCycles(dut.clk, 2000)
    bus.stop()
    assert not bus.errors, bus.errors[:5]
    assert not pc.errors, pc.errors[:5]
    assert pc.received == [0x33, 0xF0, 0x33, 0x43, 0xF0, 0x43, 0xFA, 0xFA], [hex(x) for x in pc.received]
    assert pc.acked == [0xED, 0x02]
    r = await read_shared(host, 0, 3)
    assert r == [0x02, 2, 0x600D0B0D], r
