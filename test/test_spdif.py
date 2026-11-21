# Copyright (c) 2026 Tyler Hong
# SPDX-License-Identifier: Apache-2.0
"""S/PDIF (IEC 60958) with the engines' biphase-mark line code
(sw/tests/spdif), judged by an independent model (test/lib/spdif.py).

TX (Wire Engine 0): the model decodes the chip's line from its edge
intervals: preambles, a transition at every slot start, parity, samples,
channel-status bits. RX (Wire Engine 1): the model sends a stream, starting
mid-subframe, with +/-1 cycle of jitter on the edges; the chip must lock on
the first preamble and deliver every later subframe. 48 and 96 kHz at 50 MHz."""

import random

import cocotb
from cocotb.triggers import ClockCycles

from lib.pins import ui_pins
from lib.soc import F_CPU, load, read_shared, start, wait_irq
from lib.spdif import TYPE, SpdifSink, SpdifSource, payload, stream_halves


def expected_tx(n):
    subs = []
    for i in range(n):
        a = (0x123456 + i * 0x0A0B0C) & 0xFFFFFF
        c = 1 if i in (0, 2) else 0
        subs.append(("M" if i % 192 else "B", payload(a, c=c)))
        subs.append(("W", payload(~a & 0xFFFFFF, c=c)))
    return subs


async def chip_sends(dut, fs, n):
    host = await start(dut)
    half = F_CPU / fs / 128
    sink = SpdifSink(dut, 0, half)
    sink.start()
    await load(host, "spdif", [1, fs, n])
    await host.ctrl(0x01)
    await wait_irq(dut, 1, timeout=10_000)
    await wait_irq(dut, 0, timeout=int(n * F_CPU / fs) + 50_000)
    await ClockCycles(dut.clk, 100)
    sink.stop()
    subs = sink.decode()
    assert not sink.errors, sink.errors[:5]
    exp = expected_tx(n)[:-1]     # the last slot of the last subframe has no edge after it
    got = [(p, x) for p, x, ok in subs]
    assert all(ok for _, _, ok in subs), "parity"
    assert got == exp, f"decoded {len(got)} subframes, first mismatch at " \
        f"{next((i for i, (g, e) in enumerate(zip(got, exp)) if g != e), len(got))}"


@cocotb.test()
async def test_spdif_send_48k(dut):
    await chip_sends(dut, 48_000, 24)


@cocotb.test()
async def test_spdif_send_96k(dut):
    await chip_sends(dut, 96_000, 24)


async def chip_receives(dut, fs, jitter):
    host = await start(dut)
    half = F_CPU / fs / 128
    ui = ui_pins(dut)
    ui.set(3, 0)
    rng = random.Random(fs)
    subs = []
    for i in range(32):
        a = rng.getrandbits(24)
        b = rng.getrandbits(24)
        subs.append(("B" if i == 3 else "M", payload(a, v=i & 1, c=(i >> 1) & 1)))
        subs.append(("W", payload(b, u=1)))
    halves = stream_halves(subs)[37:]           # start in the middle of the first subframe
    n = 52
    await load(host, "spdif", [2, fs, n])
    await host.ctrl(0x01)
    await wait_irq(dut, 1, timeout=10_000)
    src = SpdifSource(dut, ui, 3, half)
    jit = (lambda i: rng.choice((-1, 0, 1))) if jitter else None
    await src.play(halves, jit)
    await wait_irq(dut, 0, timeout=20_000)
    res = await read_shared(host, 0, 8 + n)
    assert res[0] == 0x600D05D1 and res[1] == n, f"done {res[0]:#x}, received {res[1]}"
    got = [(w & 3, (w >> 2) & 0xFFFFFFF) for w in res[8:8 + n]]
    exp = [(TYPE[p], x) for p, x in subs[1:1 + n]]   # subframe 0 was cut: the chip locks on subframe 1
    assert got == exp, f"first mismatch at {next((i for i, (g, e) in enumerate(zip(got, exp)) if g != e), None)}: " \
        f"{[(t, hex(x)) for t, x in got[:4]]} vs {[(t, hex(x)) for t, x in exp[:4]]}"


@cocotb.test()
async def test_spdif_receive_48k(dut):
    await chip_receives(dut, 48_000, jitter=True)


@cocotb.test()
async def test_spdif_receive_96k(dut):
    await chip_receives(dut, 96_000, jitter=False)
