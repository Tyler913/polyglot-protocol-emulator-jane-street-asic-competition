# Copyright (c) 2026 Tyler Hong
# SPDX-License-Identifier: Apache-2.0
"""USB Power Delivery sink (sw/examples/usb_pd_sink) on Wire Engine 1's
biphase mark line code, judged by an independent USB PD source model
(test/lib/usbpd.py): BMC at 300 kbit/s, 4b5b, CRC-32, GoodCRC within
tTransmit (195 us), Source_Capabilities -> Request -> Accept -> PS_RDY."""

import cocotb
from cocotb.triggers import ClockCycles

from lib.pins import ui_pins
from lib.soc import F_CPU, load, read_shared, start, wait_irq
from lib.usbpd import ACCEPT, GOODCRC, PS_RDY, REQUEST, SRC_CAP, PdSource, header

T_TRANSMIT = round(195e-6 * F_CPU)


def fixed(mv, ma):
    return ((mv // 50) << 10) | (ma // 10)


async def goodcrc_for(src, n, mid):
    t, h, o = await src.expect(n, 200_000)
    assert h & 0x1F == GOODCRC and (h >> 12) & 7 == 0, f"expected GoodCRC, got header {h:#06x}"
    assert (h >> 9) & 7 == mid, f"GoodCRC echoes ID {(h >> 9) & 7}, expected {mid}"
    assert (h >> 8) & 1 == 0, "GoodCRC from a sink must say power role sink"
    gap = t - src.sent_end[-1]
    assert 0 < gap <= T_TRANSMIT, f"GoodCRC started {gap} cycles after our EOP (limit {T_TRANSMIT})"


@cocotb.test()
async def test_usb_pd_sink(dut):
    await run(dut)


async def run(dut):
    host = await start(dut)
    ui = ui_pins(dut)
    src = PdSource(dut, tx_uio=2, rx_ui=4, f_cpu=F_CPU, pdos=[fixed(5000, 3000), fixed(9000, 3000), fixed(15000, 3000)])
    src.start(ui)
    await load(host, "usb_pd_sink")
    await host.ctrl(0x01)
    await wait_irq(dut, 1, timeout=10_000)
    await ClockCycles(dut.clk, 2000)

    await src.send(header(SRC_CAP, 3, 0, 1, 1), src.pdos)
    await goodcrc_for(src, 0, 0)
    t, h, o = await src.expect(1, 300_000)
    assert h & 0x1F == REQUEST and (h >> 12) & 7 == 1, f"expected Request, got {h:#06x}"
    rdo = o[0]
    assert rdo >> 28 == 2, f"Request selects object {rdo >> 28}, expected 2 (9 V)"
    assert (rdo >> 10) & 0x3FF == 200 and rdo & 0x3FF == 200, f"RDO currents {rdo:#010x}"
    await ClockCycles(dut.clk, 500)
    await src.send(header(GOODCRC, 0, (h >> 9) & 7, 1, 1))
    await ClockCycles(dut.clk, 3000)
    await src.send(header(ACCEPT, 0, 1, 1, 1))
    await goodcrc_for(src, 2, 1)
    await ClockCycles(dut.clk, 20_000)
    await src.send(header(PS_RDY, 0, 2, 1, 1))
    await goodcrc_for(src, 3, 2)
    await wait_irq(dut, 0, timeout=200_000)
    src.stop()
    res = await read_shared(host, 0, 6)
    dut._log.info(f"chip results {[hex(x) for x in res]}")
    assert not src.errors, src.errors[:5]
    assert res[0] == 0x600D0BD0, f"done {res[0]:#x}"
    assert res[1:6] == [3, 2, 9000, 2000, 0 | 1 << 4 | 2 << 8], res
