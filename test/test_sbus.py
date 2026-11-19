# Copyright (c) 2026 Tyler Hong
# SPDX-License-Identifier: Apache-2.0
"""Futaba S.BUS both ways on a lite engine (sw/examples/sbus), judged by an
independent frame model (lib/sbus.py) and serial line model (lib/serial.py):
8E2 on an inverted line at 100000 baud, 16 x 11-bit channels, flags.
Decode: three frames, the middle one with a parity error that must be
rejected. Encode: two frames, every byte's parity and stop bits checked."""

import random

import cocotb
from cocotb.triggers import ClockCycles

from lib import sbus
from lib.pins import bit, ui_pins
from lib.serial import SerialRx, SerialTx
from lib.soc import load, read_shared, start, wait_irq


def unpack(words):
    ch = []
    for w in words[:8]:
        ch += [w & 0xFFFF, w >> 16]
    return ch, words[8]


@cocotb.test()
async def test_sbus_decode(dut):
    host = await start(dut)
    rng = random.Random(7)
    frames = [([rng.randrange(2048) for _ in range(16)], 0x00),
              ([rng.randrange(2048) for _ in range(16)], 0x00),
              ([0, 2047] + [rng.randrange(2048) for _ in range(14)], 0x0C | 0x03)]  # lost + failsafe, ch17/18
    frames[1][0][0] = 0x0F  # a 0x0F data byte inside a frame must not resync the decoder
    tx = SerialTx(dut, ui_pins(dut), 0, sbus.BIT_CYCLES, sbus.FMT)
    await load(host, "sbus", [0, 2])
    await host.ctrl(0x01)
    await wait_irq(dut, 1, timeout=10_000)
    await ClockCycles(dut.clk, 40_000)
    for i, (ch, fl) in enumerate(frames):
        await tx.send(sbus.encode(ch, fl), bad_parity=(5,) if i == 1 else ())
        await ClockCycles(dut.clk, 50_000)    # 1 ms gap between frames
    await wait_irq(dut, 0, timeout=20_000)
    res = await read_shared(host, 0, 50)
    assert res[2] == 0x600D0007, hex(res[2])
    assert res[3] == 1, f"rejected {res[3]} frames (want 1: the parity error)"
    got = [unpack(res[32 + 9 * k: 41 + 9 * k]) for k in range(2)]
    want = [frames[0], frames[2]]
    assert got == want, f"chip decoded {got}"


@cocotb.test()
async def test_sbus_encode(dut):
    host = await start(dut)
    rng = random.Random(11)
    ch = [rng.randrange(2048) for _ in range(16)]
    ch[3], ch[4] = 0, 2047
    flags = 0x08
    params = [1, 2] + [0] * 6 + [ch[2 * k] | (ch[2 * k + 1] << 16) for k in range(8)] + [flags]
    rx = SerialRx(dut, lambda: bit(dut.uo_out, 0), sbus.BIT_CYCLES, sbus.FMT)
    rx.start()
    await load(host, "sbus", params)
    await host.ctrl(0x01)
    await wait_irq(dut, 1, timeout=10_000)
    await wait_irq(dut, 0, timeout=1_000_000)
    await ClockCycles(dut.clk, 2_000)
    rx.stop()
    assert not rx.errors, rx.errors[:5]
    vals = rx.values
    assert len(vals) == 50, f"{len(vals)} bytes"
    for k in range(2):
        assert sbus.decode(vals[25 * k: 25 * k + 25]) == (ch, flags), f"frame {k}"
    starts = [t for v, t in rx.frames]
    period = starts[25] - starts[0]
    assert abs(period - 350_000) <= 1_000, f"frame period {period} cycles (want 7 ms +/- 20 us)"
    gaps = [starts[i + 1] - starts[i] for i in range(24)]
    assert all(g == 6000 for g in gaps), f"byte spacing {sorted(set(gaps))} (want 12 bits back to back)"
