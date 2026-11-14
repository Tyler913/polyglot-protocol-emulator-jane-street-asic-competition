# Copyright (c) 2026 Tyler Hong
# SPDX-License-Identifier: Apache-2.0
"""CAN 2.0A at 500 kbit/s from a Wire Engine (sw/tests/can), judged by an
independent CAN bus model (test/lib/can.py): stuffing, CRC-15, ACK, and
arbitration against a second node with a lower (winning) identifier. The chip
also acknowledges other nodes' frames (receiver ACK, sw/tests/can_rx), and
not a frame with a bad CRC."""

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
    assert node.frames[0]["acked"], "the chip did not acknowledge the winner's frame"
    assert all(f["crc_ok"] for f in node.frames), node.frames
    assert res[1] == 2, f"chip made {res[1]} attempts (expected: lose once, then send)"
    assert res[2] == 1, "retried frame not acknowledged"
    assert (res[3], res[4], res[5] & 0xFFFFFF, res[6]) == (1, 0x100, 0x332211, 1), f"chip received {res[3:]}"


@cocotb.test()
async def test_can_receiver_ack(dut):
    """The node sends five frames (DLC 0, 3, 8, one with a bad CRC, then a good
    one); the chip, a receiver only, must ACK exactly the good ones."""
    host = await start(dut)
    bus = CanBus(dut, txd_uio=0, rxd_ui=2, bit_cycles=BIT)
    node = bus.add(CanNode(bus, "node"))
    bus.start()
    await load(host, "can_rx", [5])
    await host.ctrl(0x01)
    await wait_irq(dut, 1, timeout=10_000)
    sent = [(0x0F0, [], False), (0x2A5, [1, 2, 3], False),
            (0x7FF, [0xFF, 0x00, 0x55, 0xAA, 0x0F, 0xF0, 0x81, 0x7E], False),
            (0x321, [0xC4, 0x3B], True), (0x055, [0x99], False)]
    for fid, data, bad in sent:
        node.send(fid, data, join_sof=False, bad_crc=bad)
        for _ in range(200):
            await ClockCycles(dut.clk, 1000)
            if node.pending is None:
                break
        else:
            raise AssertionError(f"node never finished frame {fid:#x}")
    await wait_irq(dut, 0, timeout=50_000)
    bus.stop()
    assert not node.errors, node.errors[:5]
    got = [(f["id"], f["acked"], f["crc_ok"]) for f in node.frames]
    assert got == [(fid, not bad, not bad) for fid, _, bad in sent], f"node saw {got}"
    res = await read_shared(host, 0, 2 + 3 * 5)
    assert res[0] == 0x600D0004 and res[1] == 5, [hex(x) for x in res[:2]]
    chip = [(res[2 + 3 * k], res[3 + 3 * k] & 0xFF, res[3 + 3 * k] >> 8) for k in range(5)]
    want = [(fid, len(d), int(not bad)) for fid, d, bad in sent]
    assert chip == want, f"chip received {chip}"
    dut._log.info("receiver ACK: %s", got)
    # The chip's ACK pulses: one bit long, inside the ACK slot of the node's
    # timing (the node samples at 75 % of the bit)
    e = bus.chip_edges
    pulses = [(e[i][0], e[i + 1][0]) for i in range(len(e) - 1) if e[i][1] == 0 and e[i + 1][1] == 1]
    assert len(pulses) == 4, pulses
    widths = sorted({b - a for a, b in pulses})
    dut._log.info("ACK pulse widths %s cycles (bit %d)", widths, BIT)
    assert all(abs(w - BIT) <= 1 for w in widths), widths
    slots = [t for t, (fid, _, bad) in zip(node.ack_slots, sent) if not bad]
    offs = [p0 - t for (p0, _), t in zip(pulses, slots)]
    dut._log.info("ACK pulse start - ACK slot start: %s cycles", offs)
    assert all(0 <= o <= BIT // 4 for o in offs), offs
