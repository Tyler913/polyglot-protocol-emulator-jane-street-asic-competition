# Copyright (c) 2026 Tyler Hong
# SPDX-License-Identifier: Apache-2.0
"""Quad-SPI flash read with all four engines as one 4-bit port
(sw/examples/qspi_flash), judged by an independent flash model
(lib/qspi_flash.py): command 0x6B, 24-bit address, 8 dummy clocks, then a
nibble per clock on IO0..IO3; no bus contention on IO0."""

import random

import cocotb
from cocotb.triggers import ClockCycles

from lib.qspi_flash import QspiFlash
from lib.soc import load, read_shared, start, wait_irq


@cocotb.test()
async def test_qspi_flash_read(dut):
    host = await start(dut)
    rng = random.Random(3)
    mem = bytes(rng.randrange(256) for _ in range(4096))
    fl = QspiFlash(dut, sck_uo=0, cs_uo=1, io0_uio=0, io_ui=(0, 1, 2), mem=mem)
    fl.start()
    addr = 0x0A5C
    await load(host, "qspi_flash", [addr, 64])
    await host.ctrl(0x01)
    await wait_irq(dut, 1, timeout=10_000)
    await wait_irq(dut, 0, timeout=100_000)
    await ClockCycles(dut.clk, 100)
    fl.stop()
    assert not fl.errors, fl.errors[:5]
    assert fl.commands == [(0x6B, addr, 64)], fl.commands
    res = await read_shared(host, 0, 24)
    assert res[2] == 0x600D0010, hex(res[2])
    got = b"".join(w.to_bytes(4, "little") for w in res[8:24])
    assert got == mem[addr:addr + 64], f"read {got.hex()}\nwant {mem[addr:addr + 64].hex()}"
