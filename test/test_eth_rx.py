# Copyright (c) 2026 Tyler Hong
# SPDX-License-Identifier: Apache-2.0
"""10BASE-T receive on a full Wire Engine (sw/tests/eth_rx): the engine
recovers the clock from the Manchester edges, finds the SFD and checks the
FCS. Stimulus from an independent IEEE 802.3 transmitter model
(test/lib/eth.py, EthRxDriver) with seeded, independent jitter on every
edge. Runs at 50 MHz and at 60 MHz, at the jitter bounds the decoder
accepts by design: an edge is a mid-bit transition if it comes at least
DIV - DIV/4 cycles after the previous one, so boundary and mid-bit
transitions stay apart while each edge moves by less than
+/-5 ns at 50 MHz and +/-8.3 ns at 60 MHz (IEEE 802.3 allows +/-13.5 ns at
the receiver, mostly from cable ISI; see docs)."""

import os

import cocotb
from cocotb.triggers import ClockCycles, Timer

from lib.eth import EthRxDriver
from lib.soc import load, read_shared, start, wait_irq


def frames(n, seed):
    import random

    r = random.Random(seed)
    out = []
    for i in range(n):
        size = [60, 64, 61, 99, 128][i % 5]
        body = bytes(r.randrange(256) for _ in range(size - 14))
        out.append(bytes.fromhex("ffffffffffff02000000000a") + b"\x88\xb5" + body)
    return out


async def run(dut, clk_ps, jitter_ps, bad=(), bit_ps=100_000):
    host = await start(dut, clk_ns=clk_ps / 1000)
    drv = EthRxDriver(dut, ui_bit=0, bit_ps=bit_ps, jitter_ps=jitter_ps, seed=clk_ps)
    want = frames(5, clk_ps)
    await load(host, "eth_rx", [len(want), round(100_000 / clk_ps)])
    await host.ctrl(0x01)
    await wait_irq(dut, 1, timeout=20_000)
    for i, f in enumerate(want):
        await drv.send(f, bad_fcs=(i in bad))
        await Timer(9_600, unit="ns")  # inter-frame gap
    await wait_irq(dut, 0, timeout=20_000)
    res = await read_shared(host, 1, len(want))
    addr = (await read_shared(host, 16, 1))[0]
    assert (await read_shared(host, 17, 1))[0] == 0x600D0E7E
    off = 0
    for i, f in enumerate(want):
        n = res[i] if res[i] < 0x80000000 else res[i] - (1 << 32)
        if i in bad:
            assert n == -1, f"frame {i}: bad FCS not reported ({n})"
            continue
        assert n == len(f) + 4, f"frame {i}: length {n}, want {len(f) + 4}"
        got = await host.read(addr + off, len(f))
        assert got == f, f"frame {i}: {got.hex()} != {f.hex()}"
        off += (n + 3) & ~3


@cocotb.test()
async def test_eth_rx_50mhz(dut):
    """50 MHz, +/-5 ns edge jitter; frame 2 has a wrong FCS."""
    await run(dut, 20_000, 5_000, bad=(2,))


@cocotb.test()
async def test_eth_rx_60mhz(dut):
    """60 MHz (16.666 ns), +/-8 ns edge jitter, sender 0.1 % slow (10x the
    802.3 frequency tolerance): the decoder follows the sender's clock."""
    await run(dut, 16_666, 8_000, bit_ps=100_100)
