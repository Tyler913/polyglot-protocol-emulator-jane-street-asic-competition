# Copyright (c) 2026 Tyler Hong
# SPDX-License-Identifier: Apache-2.0
"""DALI controller (sw/examples/dali): Manchester encode and decode on a full
Wire Engine at the real 1200 bit/s, against an independent control gear
(test/lib/dali.py): forward frames on the Te grid, direct arc power, a query
answered by a backward frame, and a query nobody answers."""

import cocotb

from lib.dali import DaliGear
from lib.soc import load, read_shared, start, wait_irq


@cocotb.test()
async def test_dali(dut):
    host = await start(dut)
    gear = DaliGear(dut, tx_uo=4, rx_ui=2, address=3)
    gear.start()
    await load(host, "dali")
    await host.ctrl(0x01)
    try:
        await wait_irq(dut, 1, timeout=7_000_000)
    finally:
        gear.stop()
        dut._log.info(f"frames {gear.frames} errors {gear.errors[:3]}")
    assert not gear.errors, gear.errors[:5]
    assert gear.frames == [(6, 0xC8), (7, 0xA0), (19, 0xA0)], gear.frames
    r = await read_shared(host, 0, 3)
    assert r == [0xC8, 0x100, 0x600DDA11], [hex(x) for x in r]
