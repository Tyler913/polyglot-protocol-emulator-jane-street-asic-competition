# Copyright (c) 2026 Tyler Hong
# SPDX-License-Identifier: Apache-2.0
"""MIPI I3C SDR controller (sw/tests/i3c) on Wire Engine 0, judged by an
independent I3C target model (test/lib/i3c.py): open-drain address headers
and ACKs (the controller never drives SDA high there), push-pull data words
with odd-parity T bits at 12.5 MHz, target-driven reads ended by T = 0, and
the CCCs RSTDAA, SETDASA, GETPID, GETBCR, GETDCR."""

import cocotb
from cocotb.triggers import ClockCycles

from lib.i3c import I3cBus, I3cTarget
from lib.soc import load, read_shared, start, wait_irq


@cocotb.test()
async def test_i3c_controller(dut):
    host = await start(dut)
    tgt = I3cTarget(0x50, pid=0x04A1_2345_6789, bcr=0x06, dcr=0x44)
    tgt.regs[0x13] = 0xD4
    bus = I3cBus(dut, sda=0, scl=1, target=tgt)
    bus.start()
    await load(host, "i3c")
    await host.ctrl(0x01)
    await wait_irq(dut, 1, timeout=10_000)
    await wait_irq(dut, 0, timeout=200_000)
    await ClockCycles(dut.clk, 50)
    bus.stop()
    res = await read_shared(host, 0, 7)
    dut._log.info(f"target log {tgt.log}")
    dut._log.info(f"chip results {[hex(x) for x in res]}")
    assert not bus.errors, bus.errors[:5]
    assert res[0] == 0x600D013C, f"done {res[0]:#x}"
    assert res[1] == 0x7F, f"ACK bits {res[1]:#x} (expected steps 0-6 acknowledged, 7 not)"
    pid = tgt.pid.to_bytes(6, "big")
    assert res[2] == int.from_bytes(pid[:4], "little") and res[3] == int.from_bytes(pid[4:], "little"), "PID"
    assert res[4] == 0x06 | 0x44 << 8, f"BCR/DCR {res[4]:#x}"
    assert res[5] == int.from_bytes(bytes([0xA1, 0xB2, 0xC3, 0xD4]), "little") and res[6] == 4, \
        f"private read {res[5]:#x}, {res[6]} bytes"
    assert ("setdasa", 0x30) in tgt.log and ("write", [0x10, 0xA1, 0xB2, 0xC3]) in tgt.log, tgt.log
    assert tgt.dyn is None, "RSTDAA did not reset the dynamic address"
