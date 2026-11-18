# Copyright (c) 2026 Tyler Hong
# SPDX-License-Identifier: Apache-2.0
"""IR remote receiver (sw/examples/ir_rx): NEC and RC-5 frames decoded in C
from capture-unit timestamps, with the remote's timing off by -8 % and +8 %
(test/lib/ir_rx.py builds the waveforms from the two formats)."""

import cocotb
from cocotb.triggers import Timer

from lib.ir_rx import IrDriver
from lib.soc import load, read_shared, start, wait_irq


@cocotb.test()
async def test_ir_nec_rc5_receive(dut):
    host = await start(dut)
    await load(host, "ir_rx")
    ir = IrDriver(dut, ui_bit=3, scale=0.92)
    await host.ctrl(0x01)
    await Timer(100, unit="us")
    await ir.nec(0x04, 0x5A)
    await Timer(40, unit="ms")
    await ir.nec_repeat()
    await Timer(25, unit="ms")
    ir.scale = 1.08
    await ir.rc5(1, 0x05, 0x35)
    await Timer(25, unit="ms")
    await ir.rc5(0, 0x1F, 0x00)
    await Timer(25, unit="ms")
    r = await read_shared(host, 0, 5)
    assert r == [4, 0x01045A00, 0x01045AFF, 0x05010535, 0x05001F00], [hex(x) for x in r]
