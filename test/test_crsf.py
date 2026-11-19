# Copyright (c) 2026 Tyler Hong
# SPDX-License-Identifier: Apache-2.0
"""CRSF (ExpressLRS / Crossfire) flight-controller side on a lite engine
(sw/examples/crsf), judged by an independent CRSF model (lib/crsf.py) on the
serial line model: frames from the "receiver" at 500 Hz (one every 2 ms),
RC channels and link statistics decoded, a frame with a bad CRC rejected,
and a battery telemetry frame answered after every good RC frame."""

import random

import cocotb
from cocotb.triggers import ClockCycles

from lib import crsf
from lib.pins import bit, ui_pins
from lib.serial import SerialRx, SerialTx
from lib.soc import load, read_shared, start, wait_irq

PERIOD = 100_000  # 2 ms


@cocotb.test()
async def test_crsf_fc(dut):
    host = await start(dut)
    rng = random.Random(5)
    rc = [[rng.randrange(172, 1812) for _ in range(16)] for _ in range(3)]
    bad = crsf.rc_channels(rc[1])
    bad[-1] ^= 0x01                                   # CRC error
    frames = [crsf.link_stats(70, 100), crsf.rc_channels(rc[0]), bad,
              crsf.rc_channels(rc[1]), crsf.link_stats(85, 97), crsf.rc_channels(rc[2])]
    tx = SerialTx(dut, ui_pins(dut), 1, crsf.BIT_CYCLES, crsf.FMT)
    rx = SerialRx(dut, lambda: bit(dut.uo_out, 1), crsf.BIT_CYCLES, crsf.FMT)
    rx.start()
    await load(host, "crsf", [3, 168, 125, 64])       # 16.8 V, 12.5 A, 64 %
    await host.ctrl(0x01)
    await wait_irq(dut, 1, timeout=10_000)
    await ClockCycles(dut.clk, 5_000)
    for fr in frames:
        await tx.send(fr)
        await ClockCycles(dut.clk, PERIOD - round(len(fr) * 10 * crsf.BIT_CYCLES))
    await wait_irq(dut, 0, timeout=100_000)
    await ClockCycles(dut.clk, 20_000)
    rx.stop()
    assert not rx.errors, rx.errors[:5]
    res = await read_shared(host, 0, 60)
    assert res[4] == 0x600D0008, hex(res[4])
    assert res[5] == 1, f"rejected {res[5]} frames (want 1: the CRC error)"
    assert res[6] == 85 | (97 << 8), f"link stats {res[6]:#x}"
    for k in range(3):
        w = res[32 + 8 * k: 40 + 8 * k]
        got = [x for v in w for x in (v & 0xFFFF, v >> 16)]
        assert got == rc[k], f"RC frame {k}: {got}"
    tele = crsf.parse(rx.values)
    want = [(0x08, [0, 168, 0, 125, 0, 0, k + 1, 64]) for k in range(3)]
    assert tele == want, f"telemetry {tele}"
