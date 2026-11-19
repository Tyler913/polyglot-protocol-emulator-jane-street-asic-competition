# Copyright (c) 2026 Tyler Hong
# SPDX-License-Identifier: Apache-2.0
"""100 Mbit/s Ethernet through the experimental RMII port (src/pg_rmii.v,
RTL built with make RMII=yes; sw/tests/rmii_echo), against an independent
RMII PHY model (lib/rmii.py): frames of 60, 64, 1000 and 1514 bytes in, one
with a bad FCS that must be dropped; every good frame echoed with the MAC
addresses swapped, preamble/SFD/FCS and the inter-frame gap checked, and the
C code keeping up (no RX overflow, no TX underrun)."""

import random

import cocotb
from cocotb.triggers import ClockCycles

from lib.rmii import RmiiPhy
from lib.soc import load, read_shared, start, wait_irq


def frame(n, seed):
    rng = random.Random(seed)
    dst = bytes([0x02, 0, 0, 0, 0, 0x01])
    src = bytes([0x02, 0x11, 0x22, 0x33, 0x44, seed])
    return dst + src + bytes([0x88, 0xB5]) + bytes(rng.randrange(256) for _ in range(n - 14))


@cocotb.test()
async def test_rmii_echo(dut):
    host = await start(dut)
    phy = RmiiPhy(dut)
    phy.start()
    sizes = [60, 64, 1000, 1514]
    frames = [frame(n, k) for k, n in enumerate(sizes)]
    await load(host, "rmii_echo", [len(frames) + 1])
    await host.ctrl(0x01)
    await wait_irq(dut, 1, timeout=10_000)
    # One CPU moves both directions, one at a time: each frame comes after the
    # previous echo (a frame arriving while C sends is lost, RX overflow is flagged)
    phy.queue = [(frames[0], False, 100), (frames[1], False, 3000), (frame(80, 9), True, 3000)]
    phy.queue += [(f, False, 9000) for f in frames[2:]]
    try:
        await wait_irq(dut, 0, timeout=400_000)
        await ClockCycles(dut.clk, 2000)
    finally:
        phy.stop()
        await ClockCycles(dut.clk, 2)
        r = await read_shared(host, 2, 5)
        dut._log.info("chip %s; PHY got %s frames (%s bytes), errors %s, queue left %d", [hex(x) for x in r],
                      len(phy.frames), [len(f) for f in phy.frames], phy.errors[:3], len(phy.queue))
    assert not phy.errors, phy.errors[:5]
    want = [f[6:12] + f[0:6] + f[12:] for f in frames]
    assert len(phy.frames) == len(want), f"{len(phy.frames)} frames echoed"
    for k, (g, w) in enumerate(zip(phy.frames, want)):
        assert g == w, f"frame {k}: {len(g)} bytes, first difference at {next(i for i in range(min(len(g), len(w))) if g[i] != w[i]) if g != w[:len(g)] else len(g)}"
    res = await read_shared(host, 0, 7)
    dut._log.info("results %s, gaps %s", [hex(x) for x in res[2:7]], phy.gaps)
    assert res[2] == 0x600D0017 and (res[3], res[4], res[5]) == (4, 1, 0), [hex(x) for x in res[2:6]]
    assert res[6] & 0x6 == 0, f"STATUS {res[6]:#x}: underrun or overflow"
