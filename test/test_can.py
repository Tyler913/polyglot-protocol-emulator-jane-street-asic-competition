# Copyright (c) 2026 Tyler Hong
# SPDX-License-Identifier: Apache-2.0
"""CAN 2.0A at 500 kbit/s from a Wire Engine (sw/tests/can), judged by an
independent CAN bus model (test/lib/can.py): stuffing, CRC-15, ACK, and
arbitration against a second node with a lower (winning) identifier."""

import cocotb
from cocotb.triggers import ClockCycles

from lib.can import CanBus, CanNode
from lib.soc import F_CPU, load, read_shared, start, wait_irq

BIT = F_CPU / 500_000


async def run_case(dut, rival):
    host = await start(dut)
    bus = CanBus(dut, txd_uio=0, rxd_ui=2, bit_cycles=BIT)
    node = bus.add(CanNode(bus, "node"))
    if rival:
        node.send(0x100, [0x11, 0x22, 0x33], join_sof=True)   # joins the chip's SOF and wins
    bus.start()
    await load(host, "can")
    await host.ctrl(0x01)
    await wait_irq(dut, 1, timeout=10_000)
    await wait_irq(dut, 0, timeout=200_000)
    await ClockCycles(dut.clk, 200)
    bus.stop()
    res = await read_shared(host, 0, 7)
    dut._log.info(f"node frames {node.frames}, chip results {[hex(x) for x in res]}")
    assert not node.errors, node.errors[:5]
    return res, node


@cocotb.test()
async def test_can_send_ack(dut):
    res, node = await run_case(dut, rival=False)
    assert res[0] == 0x600D0003 and res[1] == 1, f"done {res[0]:#x}, attempts {res[1]}"
    assert node.frames == [dict(id=0x123, data=[0xDE, 0xAD, 0xBE, 0xEF], crc_ok=True, acked=True, by="other")]
    assert res[2] == 1, "chip did not see its frame acknowledged"


@cocotb.test()
async def test_can_arbitration(dut):
    res, node = await run_case(dut, rival=True)
    ids = [(f["id"], f["by"]) for f in node.frames]
    assert ids == [(0x100, "node"), (0x123, "other")], f"bus order {ids}"
    assert all(f["crc_ok"] for f in node.frames), node.frames
    assert res[1] == 2, f"chip made {res[1]} attempts (expected: lose once, then send)"
    assert res[2] == 1, "retried frame not acknowledged"
    assert (res[3], res[4], res[5] & 0xFFFFFF, res[6]) == (1, 0x100, 0x332211, 1), f"chip received {res[3:]}"
