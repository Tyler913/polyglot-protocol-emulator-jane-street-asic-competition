# Copyright (c) 2026 Tyler Hong
# SPDX-License-Identifier: Apache-2.0
"""PS/2 host (sw/examples/ps2_host) on a lite Wire Engine clocked by the
device, judged by an independent PS/2 keyboard model (lib/ps2.py): frames
from the keyboard, then host-to-device commands (inhibit >= 100 us, request
to send, bits changed only while CLK is low, parity, stop, device ACK) and
the keyboard's 0xFA answers."""

import cocotb

from lib.i2c import OpenDrainBus
from lib.ps2 import Ps2Keyboard
from lib.soc import CLK_NS, load, read_shared, start, wait_irq


@cocotb.test()
async def test_ps2_keyboard(dut):
    host = await start(dut)
    bus = OpenDrainBus(dut, pins={2, 3})
    kbd = bus.add(Ps2Keyboard(bus, clk=2, data=3, clk_ns=CLK_NS))
    bus.start()
    await load(host, "ps2_host", [4])
    await host.ctrl(0x01)
    await wait_irq(dut, 1, timeout=10_000)
    kbd.send(0xAA, 0x1C, 0xF0, 0x1C)          # self-test passed, 'A' pressed and released
    try:
        await wait_irq(dut, 0, timeout=1_500_000)
    finally:
        bus.stop()
        assert not bus.errors, bus.errors[:5]
        assert not kbd.errors, (kbd.errors[:5], [hex(c) for c in kbd.commands], [hex(c) for c in kbd.sent])
    assert kbd.commands == [0xED, 0x02], [hex(c) for c in kbd.commands]
    done, errors = await read_shared(host, 1, 2)
    got = await read_shared(host, 8, 6)
    assert done == 0x600D0006 and errors == 0, (hex(done), errors)
    assert got == [0xAA, 0x1C, 0xF0, 0x1C, 0xFA, 0xFA], [hex(x) for x in got]
