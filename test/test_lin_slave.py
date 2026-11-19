# Copyright (c) 2026 Tyler Hong
# SPDX-License-Identifier: Apache-2.0
"""LIN 2.x slave node on a lite engine (sw/examples/lin_slave), judged by an
independent LIN master model (lib/lin.py LinMaster, ISO 17987): published
IDs answered with the enhanced checksum, a subscribed ID received, the
diagnostic Read by Identifier (0x3C / 0x3D, classic checksum), and headers
the node must ignore: unknown ID, PID parity error, wrong sync byte."""

import cocotb
from cocotb.triggers import ClockCycles

from lib.lin import LinMaster
from lib.soc import load, read_shared, start, wait_irq

BIT = 50e6 / 19200


@cocotb.test()
async def test_lin_slave(dut):
    host = await start(dut)
    m = LinMaster(dut, txd_uo=3, rxd_ui=4, bit_cycles=BIT)
    steps = [
        ("0x21 response", dict(fid=0x21, n_resp=2), (bytes([0, 0xA5]), True)),
        ("0x10 publish", dict(fid=0x10, data=[0x11, 0x22, 0x33, 0x44]), None),
        ("0x22 response", dict(fid=0x22, n_resp=4), (bytes([0xEF, 0xBE, 0xAD, 0xDE]), True)),
        ("unknown 0x05", dict(fid=0x05, n_resp=2), None),
        ("PID parity error", dict(fid=0x21, n_resp=2, bad_parity=True), None),
        ("bad sync", dict(fid=0x21, n_resp=2, bad_sync=True), None),
        ("0x3C read by id", dict(fid=0x3C, data=[0x0A, 0x06, 0xB2, 0x00, 0x34, 0x12, 0x78, 0x56], classic=True), None),
        ("0x3D response", dict(fid=0x3D, n_resp=8, classic=True), (bytes([0x0A, 0x06, 0xF2, 0x34, 0x12, 0x78, 0x56, 0x01]), True)),
        ("0x21 again", dict(fid=0x21, n_resp=2), (bytes([1, 0xA5]), True)),
    ]
    await load(host, "lin_slave", [len(steps)] + [0] * 7 + [0xDEADBEEF])
    m.start()
    await host.ctrl(0x01)
    await wait_irq(dut, 1, timeout=10_000)
    await ClockCycles(dut.clk, round(5 * BIT))
    for name, kw, want in steps:
        got = await m.frame(**kw)
        dut._log.info("%-18s -> %s", name, (got[0].hex(), got[1]) if got else None)
        assert got == want, f"{name}: got {got}, want {want}"
        await ClockCycles(dut.clk, round(4 * BIT))   # inter-frame space
    await wait_irq(dut, 0, timeout=round(40 * BIT))
    res = await read_shared(host, 0, 8)
    assert res[2] == 0x600D000A, hex(res[2])
    assert (res[3], res[4], res[5], res[6], res[7]) == (4, 3, 0x44332211, 1, 0), [hex(x) for x in res]
