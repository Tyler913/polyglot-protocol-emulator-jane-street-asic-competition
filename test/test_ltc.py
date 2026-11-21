# Copyright (c) 2026 Tyler Hong
# SPDX-License-Identifier: Apache-2.0
"""SMPTE linear timecode at 30 fps (sw/examples/ltc) with the engines'
biphase mark code, judged by an independent LTC model (test/lib/ltc.py): the
chip's output is decoded from its edge intervals (BCD fields, polarity bit,
sync word, frame count), and an LTC stream from the model, started in the
middle of a frame, is read back by engine 1."""

import cocotb
from cocotb.triggers import ClockCycles, Edge, First
from cocotb.utils import get_sim_time

from lib.ltc import SYNC, bmc_halves, fields, frame_bits
from lib.pins import bit, ui_pins
from lib.soc import F_CPU, load, read_shared, start, wait_irq
from lib.usbpd import decode_bits

HALF = F_CPU / 2400 / 2


def next_time(t):
    h, m, s, f = t
    f += 1
    if f == 30:
        f, s = 0, s + 1
    if s == 60:
        s, m = 0, m + 1
    if m == 60:
        m, h = 0, h + 1
    return (h, m, s, f)


@cocotb.test()
async def test_ltc(dut):
    host = await start(dut)
    ui = ui_pins(dut)
    ui.set(3, 0)
    start_t = (23, 59, 59, 28)
    nsend, nread = 3, 2
    await load(host, "ltc", [nsend, nread, 23 << 24 | 59 << 16 | 59 << 8 | 28])
    edges = []

    async def watch():                     # the chip's LTC output: (cycle, level) at every change
        prev = None
        while True:
            await First(Edge(dut.uio_out), Edge(dut.uio_oe))
            v = bit(dut.uio_out, 0) if bit(dut.uio_oe, 0) else None
            if v != prev:
                edges.append((get_sim_time("ns") / 20.0, v))
                prev = v
    mon = cocotb.start_soon(watch())
    await host.ctrl(0x01)
    await wait_irq(dut, 1, timeout=10_000)
    # model stream to the chip: from the middle of frame 0, frames 01:02:03:04..
    rx_times = [(1, 2, 3, 4)]
    for _ in range(3):
        rx_times.append(next_time(rx_times[-1]))
    halves = []
    lvl = 0
    for t in rx_times:
        h = bmc_halves(frame_bits(*t), lvl)
        halves += h
        lvl = h[-1]
    halves = halves[97:]
    for i, hb in enumerate(halves):
        ui.set(3, hb)
        await ClockCycles(dut.clk, round((i + 1) * HALF) - round(i * HALF))
    await wait_irq(dut, 0, timeout=4 * round(HALF) * 160)
    res = await read_shared(host, 0, 4 + nread)
    got_rx = [((x >> 24) & 255, (x >> 16) & 255, (x >> 8) & 255, x & 255) for x in res[4:4 + nread]]
    dut._log.info(f"chip read {got_rx}")
    assert res[0] == 0x600D017C and res[1] == nread, f"done {res[0]:#x}, read {res[1]}"
    assert got_rx == rx_times[1:1 + nread], f"chip read {got_rx}, expected {rx_times[1:1 + nread]}"
    mon.cancel()
    lv = [(t, v) for t, v in edges if v is not None]
    ivals = [(round(lv[i + 1][0] - lv[i][0]), lv[i][1]) for i in range(len(lv) - 1)]
    if lv and lv[0][1] == 0:
        ivals = ivals[1:]                  # start at the first edge out of idle (low)
    bits, errs = decode_bits(ivals, HALF)
    assert len(bits) >= 80 * (nsend - 1) and not [e for e in errs if int(e.split()[-1]) < 1.4 * 80 * (nsend - 1)], errs[:3]
    # frames: find sync words; the chip starts from an idle line, so frame 0 begins at bit 0
    frames = [bits[i:i + 80] for i in range(0, len(bits) - 79, 80)]
    exp, tt = [], start_t
    for _ in range(nsend):
        exp.append(tt)
        tt = next_time(tt)
    assert len(frames) >= nsend - 1, f"{len(bits)} bits decoded"
    for k, fr in enumerate(frames[:nsend - 1]):
        assert fr[64:80] == SYNC, f"frame {k}: sync word {fr[64:80]}"
        assert fr.count(0) % 2 == 0, f"frame {k}: polarity bit wrong"
        assert fields(fr) == exp[k], f"frame {k}: {fields(fr)} vs {exp[k]}"
