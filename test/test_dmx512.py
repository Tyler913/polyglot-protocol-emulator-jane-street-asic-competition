# Copyright (c) 2026 Tyler Hong
# SPDX-License-Identifier: Apache-2.0
"""DMX512 (ANSI E1.11) from a lite Wire Engine (sw/examples/dmx512): a
protocol added in a few lines of C with no hardware support, judged by an
independent receiver model (lib/dmx512.py): BREAK and MARK AFTER BREAK
lengths, 4 us bits, two stop bits, start code and channel data."""

import cocotb

from lib.dmx512 import Dmx512Model
from lib.soc import CLK_NS, load, read_shared, start, wait_irq


async def run(dut, channels, packets):
    host = await start(dut)
    m = Dmx512Model(dut, uo_bit=1, clk_ns=CLK_NS)
    m.start()
    await load(host, "dmx512", [channels, packets])
    await host.ctrl(0x01)
    slot_cycles = 44_000 // CLK_NS
    await wait_irq(dut, 1, timeout=packets * (channels + 4) * slot_cycles + 50_000)
    m.stop()
    assert not m.errors, m.errors[:5]
    got = m.all_packets()
    want = [[0] + [(7 * i + p) & 0xFF for i in range(1, channels + 1)] for p in range(packets)]
    assert got == want, f"{len(got)} packets; first {got[0][:8] if got else None}"
    assert (await read_shared(host, 2, 1))[0] == 0x600D0005


@cocotb.test()
async def test_dmx512_16ch(dut):
    await run(dut, 16, 3)


@cocotb.test()
async def test_dmx512_512ch(dut):
    """A full universe: 512 channels."""
    await run(dut, 512, 1)
