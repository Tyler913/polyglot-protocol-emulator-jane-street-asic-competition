# Copyright (c) 2026 Tyler Hong
# SPDX-License-Identifier: Apache-2.0
"""NEC infrared remote (sw/examples/ir_nec): the 38 kHz carrier is made by a
lite engine's bit stream (1010... at 76 kbit/s), judged by an independent
receiver model (lib/ir_nec.py): carrier frequency and duty, mark and space
lengths, address/command with their inverses, repeat codes every 108 ms."""

import cocotb

from lib.ir_nec import NecReceiver
from lib.soc import CLK_NS, load, read_shared, start, wait_irq


@cocotb.test()
async def test_ir_nec(dut):
    host = await start(dut)
    rx = NecReceiver(dut, uo_bit=2, clk_ns=CLK_NS)
    rx.start()
    await load(host, "ir_nec", [0x5A, 0x3C, 1])
    await host.ctrl(0x01)
    await wait_irq(dut, 1, timeout=12_000_000)
    rx.stop()
    got = rx.decode()
    assert not rx.errors, rx.errors[:5]
    assert got == [("frame", 0x5A, 0x3C), ("repeat",)], got
    gap_ms = (rx.leaders[1] - rx.leaders[0]) / 1e6
    assert abs(gap_ms - 108) < 1, f"repeat {gap_ms:.2f} ms after the frame"
    assert (await read_shared(host, 3, 1))[0] == 0x600D0007
