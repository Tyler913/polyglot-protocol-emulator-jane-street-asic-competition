# Copyright (c) 2026 Tyler Hong
# SPDX-License-Identifier: Apache-2.0
"""Timed GPIO: every scheduled pin change must land in exactly the cycle
the C code asked for, independent of how long the code took."""

import os

import cocotb
from cocotb.triggers import ReadOnly, RisingEdge

from lib.soc import load, read_shared, start, wait_irq


def now_counter(dut):
    # RTL only: the chip's cycle counter, to compare absolute times
    return int(dut.user_project.u_io.u_sys.now.value)


# Needs the chip's NOW counter by name, which only exists in the RTL.
@cocotb.test(skip=os.environ.get("GATES") == "yes")
async def test_timed_gpio(dut):
    host = await start(dut)
    await load(host, "timed_gpio")

    changes = []  # (NOW, uo[5:0], uio_oe[7])
    done = False

    async def monitor():
        last = None
        while not done:
            await RisingEdge(dut.clk)
            await ReadOnly()
            v = (int(dut.uo_out.value) & 0x3F, (int(dut.uio_oe.value) >> 7) & 1)
            if v != last:
                changes.append((now_counter(dut), *v))
                last = v

    mon = cocotb.start_soon(monitor())
    await host.ctrl(0x01)
    await wait_irq(dut, 1, timeout=20_000)
    done = True
    await mon
    await RisingEdge(dut.clk)  # leave the read-only phase before driving pins

    res = await read_shared(host, 0, 8)
    assert res[0] == 0x600D0001, f"firmware did not finish: {res[0]:#010x}"
    t0 = res[1]

    # Expected schedule relative to t0: (offset, uo[0], uo[5], uio_oe[7])
    expect = [(0, 1, 0, 0), (1, 0, 0, 0), (2, 1, 0, 0), (10, 0, 0, 0), (11, 1, 0, 0),
              (500, 0, 0, 0), (600, 1, 1, 0), (601, 0, 0, 0), (700, 0, 0, 1)]
    seen = [(t - t0, uo & 1, (uo >> 5) & 1, oe) for t, uo, oe in changes if 0 <= t - t0 <= 700]
    dut._log.info(f"t0 = {t0}, changes relative to t0: {seen}")
    assert seen == expect, f"timed changes {seen} != {expect}"

    assert res[2] == 0, "LATE set although every change was scheduled ahead"
    assert res[3] == 1, "LATE not set for a change scheduled in the past"
    assert res[4] == 0, f"capture time off by {res[4] - (1 << 32) if res[4] >> 31 else res[4]} cycles"
    assert res[5] == 1, "blocking timeout flag not set"
    assert res[6] == 0xFFFFFFFF, f"timed-out read returned {res[6]:#010x}"
    assert 50 <= res[7] <= 60, f"timed-out access took {res[7]} cycles, limit 50"
