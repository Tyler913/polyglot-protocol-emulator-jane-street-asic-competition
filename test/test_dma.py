# Copyright (c) 2026 Tyler Hong
# SPDX-License-Identifier: Apache-2.0
"""DMA channels (sw/tests/dma): words move between SRAM rings and IO
registers without the CPU. UART at 1 Mbaud on a lite engine: 120 bytes
received back to back while the CPU does not read the engine (its FIFO holds
4), and 120 bytes sent from a ring with no gap between frames (UART model,
lib/uart.py). Logic analyzer and pattern generator: one channel samples
GPIO_IN every 10 cycles while the other writes a table to the uo pins every
25 cycles; the pins must change exactly every 25 cycles with the table's
values, and the samples must show the same sequence."""

import cocotb
from cocotb.triggers import ClockCycles, Edge
from cocotb.utils import get_sim_time

from lib.soc import load, read_shared, start, wait_irq
from lib.uart import UartDriver, UartMonitor, bit_cycles

BIT = bit_cycles(1_000_000)


@cocotb.test()
async def test_dma_uart_rx(dut):
    host = await start(dut)
    drv = UartDriver(dut, 1, BIT)
    data = bytes((i * 37 + 5) & 0xFF for i in range(120))
    await load(host, "dma", [1, len(data)])
    await host.ctrl(0x01)
    await wait_irq(dut, 1, timeout=10_000)
    await drv.send(data)
    await wait_irq(dut, 0, timeout=100_000)
    res = await read_shared(host, 0, 8 + 30)
    got = b"".join(x.to_bytes(4, "little") for x in res[8:38])[:len(data)]
    assert res[0] == 0x600D0D3A, f"done {res[0]:#x}"
    assert res[2] & 0x300 == 0, f"framing or engine RX overflow: {res[2]:#x}"
    assert got == data, f"first difference at {next(i for i in range(len(data)) if got[i] != data[i])}"


@cocotb.test()
async def test_dma_uart_tx(dut):
    host = await start(dut)
    mon = UartMonitor(dut, lambda: (int(dut.uo_out.value) >> 0) & 1, BIT)
    mon.start()
    n = 120
    await load(host, "dma", [2, n])
    await host.ctrl(0x01)
    await wait_irq(dut, 1, timeout=10_000)
    await wait_irq(dut, 0, timeout=200_000)
    await ClockCycles(dut.clk, 50)
    mon.stop()
    assert not mon.errors, mon.errors[:3]
    assert bytes(mon.bytes) == bytes(0x30 + i for i in range(n)), bytes(mon.bytes)
    gaps = [b - a for a, b in zip(mon.times, mon.times[1:])]
    assert all(abs(g - 10 * BIT) <= 1 for g in gaps), f"frames not back to back: {sorted(set(gaps))}"


@cocotb.test()
async def test_dma_capture_pattern(dut):
    host = await start(dut)
    n = 56
    changes = []

    async def watch():                 # last value at each time step (gate level: pins settle in delta steps)
        while True:
            await Edge(dut.uo_out)
            t = round(get_sim_time("ns") / 20)
            v = (int(dut.uo_out.value) >> 1) & 31
            if changes and changes[-1][0] == t:
                changes[-1] = (t, v)
            else:
                changes.append((t, v))
    task = cocotb.start_soon(watch())
    await load(host, "dma", [3, n])
    await host.ctrl(0x01)
    await wait_irq(dut, 1, timeout=10_000)
    await wait_irq(dut, 0, timeout=20_000)
    task.cancel()
    dedup = []
    for t, v in changes:
        if not dedup or dedup[-1][1] != v:
            dedup.append((t, v))
    changes[:] = dedup
    res = await read_shared(host, 0, 8 + n)
    assert res[0] == 0x600D0D3A, f"done {res[0]:#x}"
    assert res[2] & 0x11 == 0, f"a timer tick was missed: status {res[2]:#x}"
    exp = [(i * 7) & 31 for i in range(39)]
    vals = [v for _, v in changes]
    k = vals.index(exp[1]) if exp[1] in vals else -1
    assert k >= 0 and vals[k:k + 38] == exp[1:39], f"pattern on the pins {vals[:12]}"
    times = [t for t, _ in changes[k:k + 38]]
    steps = {b - a for a, b in zip(times, times[1:])}
    assert steps == {25}, f"pattern steps {sorted(steps)} cycles (expected exactly 25)"
    # the samples (uo read back through the input synchronizer) show the same values
    smp = [(x >> 9) & 31 for x in res[8:8 + n]]
    seq = [smp[0]] + [b for a, b in zip(smp, smp[1:]) if b != a]
    i0 = exp.index(seq[1]) if seq[1] in exp else -1
    assert i0 >= 0 and seq[1:] == exp[i0:i0 + len(seq) - 1], f"captured {seq}"
    runs, r = [], 1
    for a, b in zip(smp, smp[1:]):
        if a == b:
            r += 1
        else:
            runs.append(r)
            r = 1
    assert set(runs[1:]) <= {2, 3}, f"samples per pattern step {runs} (25 / 10 cycles)"
