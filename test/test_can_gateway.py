# Copyright (c) 2026 Tyler Hong
# SPDX-License-Identifier: Apache-2.0
"""CAN <-> serial gateway (sw/examples/can_gateway, Lawicel/SLCAN text
protocol) under load, judged by an independent CAN node model and UART
models. CAN 1 Mbit/s, UART 3 Mbaud.

1. Full bus load: the node sends 200 random classic frames (standard and
   extended IDs, 0-8 bytes) back to back with only the 3-bit intermission
   between them; every one must come out of the UART as text, in order.
2. Both ways: while the node sends 60 more frames, the host sends 30 frames
   as t/T commands; the chip must queue each ("z"/"Z"), put every one on the
   bus with a good CRC (retrying after a lost arbitration) and still forward
   all 60 of the node's frames.
3. Acceptance filter: with code 0x120 / mask 0x7F0 only IDs 0x120-0x12F pass.
Statistics come from the host mailbox; GATEWAY_FRAMES=n changes phase 1."""

import json
import os
import random

import cocotb
from cocotb.triggers import ClockCycles

from lib.can import EFF, CanBus
from lib.canfd import CanFdNode
from lib.soc import F_CPU, load, read_shared, start, wait_irq
from lib.uart import UartDriver, UartMonitor, bit_cycles

BAUD = 3_000_000
RATE = 1_000_000


def text(fid, data):
    if fid & EFF:
        s = "T%08X" % (fid & 0x1FFFFFFF)
    else:
        s = "t%03X" % fid
    return s + str(len(data)) + "".join("%02X" % b for b in data)


def rand_frame(rng, ext_share=0.25):
    if rng.random() < ext_share:
        fid = EFF | rng.randrange(1 << 29)
    else:
        fid = rng.randrange(1 << 11)
    return dict(fid=fid, data=[rng.randrange(256) for _ in range(rng.randrange(9))], fd=False, brs=False)


class Lines:
    def __init__(self, mon):
        self.mon, self.k = mon, 0

    def all(self):
        return bytes(self.mon.bytes).decode("latin-1").split("\r")[:-1]


async def wait_for(dut, cond, timeout, what):
    for _ in range(timeout // 1000):
        if cond():
            return
        await ClockCycles(dut.clk, 1000)
    raise AssertionError(f"timeout waiting for {what}")


@cocotb.test()
async def test_can_gateway(dut):
    rng = random.Random(1234)
    n1 = int(os.environ.get("GATEWAY_FRAMES", "200"))
    host = await start(dut)
    tn = F_CPU // RATE
    bus = CanBus(dut, txd_uio=0, rxd_ui=2, bit_cycles=tn)
    node = bus.add(CanFdNode(bus, "node", tn, tn))
    ubit = bit_cycles(BAUD)
    mon = UartMonitor(dut, lambda: (int(dut.uo_out.value) >> 1) & 1, ubit)
    drv = UartDriver(dut, 0, ubit)
    lines = Lines(mon)
    await load(host, "can_gateway", [BAUD, RATE, 1])
    bus.start()
    mon.start()
    await host.ctrl(0x01)
    await wait_irq(dut, 1, timeout=200_000)
    await ClockCycles(dut.clk, 2000)

    await drv.send(b"V\r")
    await wait_for(dut, lambda: lines.all()[-1:] == ["V0101"], 50_000, "version reply")
    base = len(lines.all())

    # 1. full bus load, node -> chip -> UART
    frames1 = [rand_frame(rng) for _ in range(n1)]
    t1 = dut.clk_count if hasattr(dut, "clk_count") else None
    node.send_all(frames1, gap_bits=3)
    exp1 = [text(f["fid"], f["data"]) for f in frames1]
    await wait_for(dut, lambda: len(lines.all()) >= base + n1, n1 * 200 * tn + 500_000, "phase 1 lines")
    got1 = lines.all()[base:base + n1]
    bad = [(i, g, e) for i, (g, e) in enumerate(zip(got1, exp1)) if g != e]
    assert not bad, f"phase 1: {len(bad)} wrong lines, first {bad[:3]}"
    assert not node.errors, node.errors[:5]
    acked = sum(1 for f in node.frames if f["by"] == "node" and f["acked"])
    assert acked == n1, f"chip acknowledged {acked} of {n1} frames"
    s1 = await read_shared(host, 8, 11)

    # 2. both ways
    base = len(lines.all())
    frames2 = [rand_frame(rng) for _ in range(60)]
    cmds = [rand_frame(rng, ext_share=0.3) for _ in range(30)]
    node.send_all(frames2, gap_bits=20)
    exp_replies = []
    for c in cmds:
        await drv.send((text(c["fid"], c["data"]) + "\r").encode())
        exp_replies.append("Z" if c["fid"] & EFF else "z")
        await ClockCycles(dut.clk, rng.randrange(2000, 30000))
    want = 60 + 30
    try:
        await wait_for(dut, lambda: len(lines.all()) >= base + want, 60 * 200 * tn + 2_000_000, "phase 2 lines")
    except AssertionError:
        st = await read_shared(host, 8, 11)
        got = lines.all()[base:]
        dut._log.error(f"phase 2: {len(got)} lines, replies {[g for g in got if len(g) < 3]}, stats {st}, "
                       f"node frames by chip {[(hex(f['id']), f['crc_ok'], f['acked']) for f in node.frames if f['by'] == 'other'][:10]}, "
                       f"node queue {len(node.queue)} pending {node.pending is not None}, errors {node.errors[:5]}")
        raise
    await wait_for(dut, lambda: sum(1 for f in node.frames if f["by"] == "other") >= 30, 2_000_000,
                   "chip frames on the bus")
    got2 = lines.all()[base:]
    replies = [g for g in got2 if g in ("z", "Z")]
    fwd = [g for g in got2 if g not in ("z", "Z")]
    assert replies == exp_replies, f"replies {replies}"
    assert fwd == [text(f["fid"], f["data"]) for f in frames2], f"phase 2 forwarded {len(fwd)}: {fwd[:3]}"
    chip = [f for f in node.frames if f["by"] == "other"]
    assert [(f["id"], f["data"]) for f in chip] == [(c["fid"], c["data"]) for c in cmds], \
        f"chip frames {[(hex(f['id']), f['data']) for f in chip[:3]]}"
    assert all(f["crc_ok"] and f["acked"] for f in chip), "chip frame without good CRC / ACK"
    assert not node.errors, node.errors[:5]

    # 3. acceptance filter
    base = len(lines.all())
    await drv.send(b"M120\r")
    await drv.send(b"m7F0\r")
    await wait_for(dut, lambda: len(lines.all()) >= base + 2, 100_000, "filter replies")
    base = len(lines.all())
    frames3 = [dict(fid=0x118 + i, data=[i], fd=False, brs=False) for i in range(32)]
    node.send_all(frames3, gap_bits=3)
    await wait_for(dut, lambda: len([f for f in node.frames if f["by"] == "node"]) >= n1 + 60 + 32,
                   32 * 100 * tn + 200_000, "phase 3 frames")
    await ClockCycles(dut.clk, 20_000)
    got3 = lines.all()[base:]
    assert got3 == [text(f["fid"], f["data"]) for f in frames3 if 0x120 <= f["fid"] <= 0x12F], got3

    s = await read_shared(host, 8, 11)
    stats = dict(received=s[0], forwarded=s[1], filtered=s[2], sent=s[3], arb_retries=s[4],
                 can_rx_overflow=s[5], text_dropped=s[6], cmd_errors=s[7], worst_pass=s[8],
                 commands=s[9], phase1=dict(received=s1[0], forwarded=s1[1], worst_pass=s1[8]),
                 uart_bytes=len(mon.bytes))
    dut._log.info("GATEWAY " + json.dumps(stats))
    assert s[5] == 0 and s[6] == 0 and s[7] == 0, stats
    bus.stop()
