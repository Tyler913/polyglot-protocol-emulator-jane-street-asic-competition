# Copyright (c) 2026 Tyler Hong
# SPDX-License-Identifier: Apache-2.0
"""Logic analyzer (sw/examples/logic_analyzer): the SRAM as a capture buffer.

- Burst mode at 25, 16.7 and 50 MS/s: the testbench drives uio[7:0] with a
  count that steps every clock; every one of the 3,099 samples must be the
  count exactly PERIOD later than the one before (none late, none missed).
- Change-log mode (a DMA stream): a random bit stream on ui[0] (10 samples per bit),
  sampled every LA_LOG_PERIOD cycles (default 30: 1.67 MS/s, the most the CPU
  keeps up with at about 26 cycles per sample) for 30,000 samples, kept as changes only; the edges rebuilt from the
  log must match the stream bit for bit (within one sample), with no sample
  lost; at 5 MS/s the CPU falls behind and the DMA must flag the lost samples.
Results are logged as "LA {...}"."""

import json
import os
import random

import cocotb
from cocotb.triggers import ClockCycles, FallingEdge

from lib.pins import ui_pins
from lib.soc import load, read_shared, start, wait_irq


PERIOD_LOG = int(os.environ.get("LA_LOG_PERIOD", "30"))


async def drive_count(dut, cycles):
    for i in range(cycles):
        await FallingEdge(dut.clk)
        dut.uio_in.value = i & 0xFF


async def run_burst(dut, period):
    host = await start(dut)
    drv = cocotb.start_soon(drive_count(dut, 10_000_000))   # counting from the start
    await load(host, "logic_analyzer", [0, period, 0, 0, 0])
    await host.ctrl(0x01)
    await wait_irq(dut, 1, timeout=100_000)
    await wait_irq(dut, 0, timeout=2600 * period + 50_000)
    drv.cancel()
    r = await read_shared(host, 8, 7)
    assert r[0] == 0x600D10A5, hex(r[0])
    n, status, addr = r[1], r[2], r[3]
    raw = await host.read(addr, 4 * n)
    s = [int.from_bytes(raw[4 * i:4 * i + 4], "little") & 0xFF for i in range(n)]
    steps = [(s[i + 1] - s[i]) & 0xFF for i in range(n - 1)]
    wrong = sum(1 for d in steps if d != period & 0xFF)
    from collections import Counter
    hist = dict(Counter(steps).most_common(6))
    first_bad = next((i for i, d in enumerate(steps) if d != period & 0xFF), None)
    return dict(period=period, msps=round(50 / period, 1), samples=n, bytes=4 * n,
                late_flag=status & 1, wrong_steps=wrong, cycles=r[6], step_hist=hist, first_bad=first_bad,
                around=s[first_bad - 3:first_bad + 5] if first_bad else None)


@cocotb.test()
async def test_burst_25msps(dut):
    out = await run_burst(dut, 2)
    dut._log.info("LA " + json.dumps(out))
    assert out["late_flag"] == 0 and out["wrong_steps"] == 0, out


@cocotb.test()
async def test_burst_16msps(dut):
    out = await run_burst(dut, 3)
    dut._log.info("LA " + json.dumps(out))
    assert out["late_flag"] == 0 and out["wrong_steps"] == 0, out


@cocotb.test()
async def test_burst_50msps_limit(dut):
    """One sample per clock is beyond the DMA (it needs an IO read and an SRAM
    write per sample): the run must say so with the late flag."""
    out = await run_burst(dut, 1)
    dut._log.info("LA " + json.dumps(out))
    assert out["late_flag"] == 1 or out["wrong_steps"] == 0, out


async def change_log(dut, period):
    rng = random.Random(7)
    bit, nsamp = 10 * period, 30_000
    host = await start(dut)
    ui = ui_pins(dut)
    ui.set(0, 1)
    await load(host, "logic_analyzer", [1, period, 0, 0, 0, nsamp])
    await host.ctrl(0x01)
    await wait_irq(dut, 1, timeout=100_000)
    bits = [rng.randrange(2) for _ in range(nsamp * period // bit + 20)]

    async def stream():
        for b in bits:
            ui.set(0, b)
            await ClockCycles(dut.clk, bit)
    drv = cocotb.start_soon(stream())
    await wait_irq(dut, 0, timeout=nsamp * 60 + 100_000)
    drv.cancel()
    r = await read_shared(host, 8, 7)
    assert r[0] == 0x600D10A5, hex(r[0])
    k, addr = r[1], r[3]
    raw = await host.read(addr, 4 * k)
    e = [int.from_bytes(raw[4 * i:4 * i + 4], "little") for i in range(k)]
    # rebuild ui[0] as runs of samples
    runs, val = [], (e[0] >> 24) & 1
    for w in e[1:]:
        v, n = (w >> 24) & 1, w & 0xFFFF
        runs.append((val, n))
        val = v
    # expected runs from the bit stream (in samples), the first and last partial
    exp, cur, length = [], bits[0], 0
    for b in bits:
        if b == cur:
            length += bit
        else:
            exp.append((cur, length / period))
            cur, length = b, bit
    got_vals = [v for v, _ in runs]
    exp_vals = [v for v, _ in exp]
    m = len(runs) - 1
    # the log starts at the trigger, so its first run is partial; compare the inner runs
    inner_ok = all(abs(n - x) <= 1 for (_, n), (_, x) in zip(runs[1:m], exp[1:m]))
    out = dict(period=period, msps=50 / period, samples=nsamp, entries=k, bytes=4 * k,
               raw_bytes_needed=4 * nsamp, ring_overruns=r[5], late_flag=r[2] & 1, cycles=r[6],
               cpu_cycles_per_sample=round(r[6] / nsamp, 1),
               samples_lost_est=max(0, round(r[6] / period) - nsamp),
               values_match=got_vals[1:m] == exp_vals[1:m], lengths_match=inner_ok)
    dut._log.info("LA " + json.dumps(out))
    return out


@cocotb.test()
async def test_change_log(dut):
    out = await change_log(dut, PERIOD_LOG)
    assert out["values_match"] and out["lengths_match"] and out["late_flag"] == 0, out


@cocotb.test()
async def test_change_log_overrun(dut):
    """Too fast for the CPU (5 MS/s): the DMA stream (CTRL.STREAM) must report
    the samples it had to drop when the ring was full."""
    out = await change_log(dut, 10)
    assert out["late_flag"] == 1, out
