# Copyright (c) 2026 Tyler Hong
# SPDX-License-Identifier: Apache-2.0
"""MDIO management station (sw/examples/mdio) against an independent clause 22
PHY model (test/lib/mdio.py): reads with bus turnaround, a write, and a read
from an address with no PHY (the line stays at its pull-up)."""

import cocotb

from lib.mdio import MdioPhy
from lib.soc import load, read_shared, start, wait_irq


@cocotb.test()
async def test_mdio(dut):
    host = await start(dut)
    dut.uio_in.value = 1  # MDIO pull-up
    phy = MdioPhy(dut, mdio=0, mdc=1, phy_addr=1, regs={0: 0x1140, 2: 0x0022, 3: 0x1622})
    phy.start()
    await load(host, "mdio")
    await host.ctrl(0x01)
    await wait_irq(dut, 1, timeout=200_000)
    phy.stop()
    assert not phy.errors, phy.errors
    assert phy.frames == [("read", 1, 2, 0x0022), ("read", 1, 3, 0x1622), ("write", 1, 0, 0x1200),
                          ("read", 1, 0, 0x1200)], phy.frames
    r = await read_shared(host, 0, 5)
    assert r == [0x0022, 0x1622, 0x1200, 0xFFFF, 0x600D3D10], [hex(x) for x in r]
