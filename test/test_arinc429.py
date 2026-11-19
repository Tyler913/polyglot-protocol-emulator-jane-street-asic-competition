# Copyright (c) 2026 Tyler Hong
# SPDX-License-Identifier: Apache-2.0
"""ARINC 429 both ways (sw/examples/arinc429), judged by an independent
line model (lib/arinc429.py): bipolar return-to-zero at 100 kbit/s with
half-bit and bit timing, word gaps, odd parity, label order; received words
including one with a parity error."""

import cocotb
from cocotb.triggers import ClockCycles

from lib.arinc429 import Rx429, Tx429, word
from lib.soc import load, read_shared, start, wait_irq

TX = [word(203, 0, 0x2A5A5, 3), word(310, 1, 0x00001, 0), word(377, 3, 0x7FFFF, 1)]
RX = [word(101, 2, 0x12345, 0), word(102, 0, 0x54321, 3, bad_parity=True), word(270, 1, 0x00F0F, 2)]


@cocotb.test()
async def test_arinc429(dut):
    host = await start(dut)
    rx = Rx429(dut, a_uo=2, b_uo=3)
    tx = Tx429(dut, hi_ui=1, lo_ui=2)
    rx.start()
    await load(host, "arinc429", [len(TX), len(RX)] + [0] * 6 + [w & 0x7FFFFFFF for w in TX])
    await host.ctrl(0x01)
    await wait_irq(dut, 1, timeout=10_000)
    await ClockCycles(dut.clk, 3 * 40 * 500)          # the chip's three words go out first
    await tx.send(RX)
    await wait_irq(dut, 0, timeout=200_000)
    rx.stop()
    assert not rx.errors, rx.errors[:5]
    assert rx.words == TX, [hex(w) for w in rx.words]
    res = await read_shared(host, 0, 21)
    assert res[2] == 0x600D0014, hex(res[2])
    assert list(res[16:19]) == RX, [hex(x) for x in res[16:19]]
    assert res[20] == 1, f"parity errors {res[20]}"
