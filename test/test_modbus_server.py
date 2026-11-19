# Copyright (c) 2026 Tyler Hong
# SPDX-License-Identifier: Apache-2.0
"""Modbus RTU server on a lite engine (sw/examples/modbus_server), at the
specification's default 19200 baud 8E1, judged by an independent client
(lib/modbus.py ModbusClient on the serial line model): functions 03, 06, 16,
exceptions 01 and 02, broadcast without reply, and frames that must be
dropped silently: other address, bad CRC, a parity error, and a 2-character
gap inside a frame (more than the 1.5 allowed)."""

import cocotb
from cocotb.triggers import ClockCycles

from lib.modbus import ModbusClient
from lib.pins import bit, ui_pins
from lib.serial import Fmt
from lib.soc import load, read_shared, start, wait_irq

ME = 0x11


@cocotb.test()
async def test_modbus_server(dut):
    host = await start(dut)
    regs = [0x1000 + 0x111 * i for i in range(16)]
    cl = ModbusClient(dut, ui_pins(dut), 3, lambda: bit(dut.uo_out, 2), 19200, Fmt("8E1"))
    steps = [
        ("read 0..3", [ME, 3, 0, 0, 0, 4], {}, bytes([ME, 3, 8]) + b"".join(r.to_bytes(2, "big") for r in regs[:4])),
        ("write 5", [ME, 6, 0, 5, 0x12, 0x34], {}, bytes([ME, 6, 0, 5, 0x12, 0x34])),
        ("write 8..10", [ME, 0x10, 0, 8, 0, 3, 6, 0, 1, 0, 2, 0, 3], {}, bytes([ME, 0x10, 0, 8, 0, 3])),
        ("read 8..10", [ME, 3, 0, 8, 0, 3], {}, bytes([ME, 3, 6, 0, 1, 0, 2, 0, 3])),
        ("read 14..17", [ME, 3, 0, 14, 0, 4], {}, bytes([ME, 0x83, 2])),
        ("function 0x2B", [ME, 0x2B, 0x0E, 1, 0], {}, bytes([ME, 0xAB, 1])),
        ("other address", [0x22, 3, 0, 0, 0, 1], {}, None),
        ("bad CRC", [ME, 3, 0, 0, 0, 1, 0x00, 0x00], dict(crc=False), None),
        ("broadcast write 2", [0, 6, 0, 2, 0xBE, 0xEF], {}, None),
        ("read 2", [ME, 3, 0, 2, 0, 1], {}, bytes([ME, 3, 2, 0xBE, 0xEF])),
        ("2-char gap", [ME, 3, 0, 0, 0, 1], dict(gap_after=(2, 2.0)), None),
        ("parity error", [ME, 3, 0, 0, 0, 1], dict(bad_parity=(3,)), None),
        ("read 5", [ME, 3, 0, 5, 0, 1], {}, bytes([ME, 3, 2, 0x12, 0x34])),
    ]
    await load(host, "modbus_server", [ME, len(steps)] + [0] * 6 + regs)
    await host.ctrl(0x01)
    await wait_irq(dut, 1, timeout=10_000)
    await ClockCycles(dut.clk, 200_000)               # line idle > 3.5 characters
    for name, frame, kw, want in steps:
        got = await cl.request(frame, **kw)
        dut._log.info("%-18s -> %s", name, got.hex() if got else None)
        assert got == want, f"{name}: got {got.hex() if got else None}, want {want.hex() if want else None}"
        await ClockCycles(dut.clk, round(2 * cl.char))
    await wait_irq(dut, 0, timeout=400_000)
    assert not cl.rx.errors, cl.rx.errors[:5]
    res = await read_shared(host, 0, 48)
    assert res[2] == 0x600D0009, hex(res[2])
    assert (res[3], res[4]) == (8, 5), f"answered {res[3]}, dropped {res[4]}"
    want_regs = list(regs)
    want_regs[5], want_regs[8:11], want_regs[2] = 0x1234, [1, 2, 3], 0xBEEF
    assert list(res[32:48]) == want_regs, [hex(x) for x in res[32:48]]
