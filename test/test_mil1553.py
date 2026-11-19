# Copyright (c) 2026 Tyler Hong
# SPDX-License-Identifier: Apache-2.0
"""MIL-STD-1553B bus controller (sw/examples/mil1553), judged by an
independent remote terminal model (lib/mil1553.py): Manchester II words with
command and data syncs and odd parity, TXA/TXB never driven together, the
BC-to-RT and RT-to-BC transfers, and no response from an absent RT."""

import cocotb

from lib.mil1553 import Rt1553
from lib.soc import load, read_shared, start, wait_irq


@cocotb.test()
async def test_mil1553_bc(dut):
    host = await start(dut)
    rt = Rt1553(dut, txa_uo=4, txb_uo=5, rxa_ui=0, address=5, tx_data=[0xCAFE, 0x1553, 0x0042])
    rt.start()
    await load(host, "mil1553")
    await host.ctrl(0x01)
    await wait_irq(dut, 1, timeout=10_000)
    await wait_irq(dut, 0, timeout=200_000)
    rt.stop()
    dut._log.info("short pulses outside words: %s", rt.glitches)
    dut._log.info("RT saw %s", [[hex(w) for _, w in m] for m in rt.messages])
    assert not rt.errors, rt.errors[:5]
    assert not rt.glitches, f"pulses on the bus outside words: {rt.glitches}"
    assert rt.received == {1: [0x10A5, 0x20A5, 0x30A5, 0x40A5]}, rt.received
    assert len(rt.messages) == 3
    res = await read_shared(host, 0, 26)
    assert res[2] == 0x600D0012, hex(res[2])
    assert (res[8], res[9]) == (5 << 11, 0), [hex(x) for x in res[8:10]]
    assert (res[16], res[17], list(res[18:21])) == (5 << 11, 3, [0xCAFE, 0x1553, 0x0042]), [hex(x) for x in res[16:21]]
    assert res[24] == 0xFFFFFFFF, hex(res[24])
