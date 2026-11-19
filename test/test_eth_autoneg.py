# Copyright (c) 2026 Tyler Hong
# SPDX-License-Identifier: Apache-2.0
"""10BASE-T auto-negotiation with fast link pulses (sw/examples/eth_autoneg,
60 MHz), against an independent clause-28 link partner (lib/autoneg.py):
FLP burst timing both ways, Link Code Words, acknowledge, and the chosen
mode: full duplex with a partner that has it, half duplex otherwise."""

import cocotb
from cocotb.triggers import Timer

from lib.autoneg import AutonegPartner
from lib.soc import load, read_shared, start, wait_irq


async def run(dut, partner_ability, want_mode, seed):
    host = await start(dut, clk_ns=16.666)
    p = AutonegPartner(dut, tdp_uio=6, rd_ui=0, ability=partner_ability, seed=seed)
    p.start()
    await load(host, "eth_autoneg")
    await host.ctrl(0x01)
    await wait_irq(dut, 1, timeout=10_000)
    await wait_irq(dut, 0, timeout=40_000_000)
    await Timer(1, unit="ms")
    p.stop()
    res = await read_shared(host, 0, 6)
    dut._log.info("chip: %s; partner saw %s, result %s", [hex(x) for x in res[2:6]],
                  [hex(w) for w in p.words], p.result)
    assert not p.errors, p.errors[:5]
    assert res[2] == 0x600D0016, hex(res[2])
    assert res[3] & 0x1FFF == p.ability & 0x1FFF and res[3] & (1 << 14), hex(res[3])
    assert res[4] == want_mode and p.result == want_mode, (res[4], p.result)
    assert all(w & 0x1FFF == 0x0061 for w in p.words), [hex(w) for w in p.words]


@cocotb.test()
async def test_autoneg_full_duplex(dut):
    await run(dut, 0x0020 | 0x0040 | 0x0080, 2, seed=1)    # partner: 10BASE-T HD/FD, 100BASE-TX


@cocotb.test()
async def test_autoneg_half_duplex(dut):
    await run(dut, 0x0020, 1, seed=2)                       # partner: 10BASE-T half duplex only
