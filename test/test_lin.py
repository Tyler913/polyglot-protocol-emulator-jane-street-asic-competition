# Copyright (c) 2026 Tyler Hong
# SPDX-License-Identifier: Apache-2.0
"""LIN master (sw/examples/lin_master) on a UART-configured lite engine,
against an independent LIN slave on a wired-AND bus (test/lib/lin.py):
break length, sync, protected IDs, enhanced and classic checksums, a slave
response, and the master's read-back of its own bytes."""

import cocotb

from lib.lin import LinBus
from lib.soc import load, read_shared, start, wait_irq
from lib.uart import bit_cycles


@cocotb.test()
async def test_lin_master(dut):
    host = await start(dut)
    bit = bit_cycles(19200)
    bus = LinBus(dut, txd_uo=3, rxd_ui=1, bit_cycles=bit, responses={0x21: b"\xa5\x5a"},
                 lengths={0x10: 4, 0x3C: 8}, classic=(0x3C,))
    bus.start()
    await load(host, "lin_master")
    await host.ctrl(0x01)
    await wait_irq(dut, 1, timeout=int(130 * bit) * 3)
    bus.stop()
    assert not bus.errors, bus.errors
    assert bus.frames == [(0x10, b"\x11\x22\x33\x44", True), (0x21, b"\xa5\x5a", True),
                          (0x3C, bytes([0x7F, 0x06, 0xB2, 0x00, 0xFF, 0x7F, 0xFF, 0xFF]), True)], bus.frames
    r = await read_shared(host, 0, 4)
    assert r == [0x5AA5, 1, 0, 0x600D0011], [hex(x) for x in r]
