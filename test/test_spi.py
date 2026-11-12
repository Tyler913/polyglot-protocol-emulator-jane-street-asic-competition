# Copyright (c) 2026 Tyler Hong
# SPDX-License-Identifier: Apache-2.0
"""SPI built from Wire Engines, judged by independent SPI peer models.

- Controller (sw/examples/spi_ram): writes and reads back an SPI RAM model
  in all four modes and two clock rates.
- Target with external clock (sw/tests/spi_target): a controller model
  clocks the engine.
"""

import cocotb
from cocotb.triggers import ClockCycles

from lib.soc import load, read_shared, start, wait_irq
from lib.spi import SpiController, SpiRam

PATTERN = bytes((0x3C + 17 * i) & 0xFF for i in range(16))


async def spi_ram_case(dut, mode, hz, addr):
    host = await start(dut)
    ram = SpiRam(dut, mode, sck=2, mosi=3, cs=4, miso_ui=1)
    await load(host, "spi_ram", [mode, hz, addr])
    ram.start()
    await host.ctrl(0x01)
    await wait_irq(dut, 1, timeout=200_000)
    ram.stop()

    assert not ram.errors, ram.errors
    assert ram.idle_edges <= 1, f"{ram.idle_edges} SCK edges while CS was high (only the settle to CPOL expected)"
    assert len(ram.frames) == 2, f"expected 2 CS frames, saw {len(ram.frames)}"
    assert ram.frames[0] == bytes([0x02, addr]) + PATTERN, f"write frame {ram.frames[0].hex()}"
    assert ram.frames[1][:2] == bytes([0x03, addr]), f"read frame {ram.frames[1].hex()}"
    assert bytes(ram.mem[(addr + i) & 0xFF] for i in range(16)) == PATTERN, "RAM model content wrong"

    res = await read_shared(host, 4, 1)
    back = b"".join(w.to_bytes(4, "little") for w in await read_shared(host, 8, 4))
    assert back == PATTERN, f"firmware read back {back.hex()}"
    assert res[0] == 0x600D0000, f"firmware result {res[0]:#010x}"


@cocotb.test()
async def test_spi_ram_mode0(dut):
    await spi_ram_case(dut, 0, 1_000_000, 0x40)


@cocotb.test()
async def test_spi_ram_mode1(dut):
    await spi_ram_case(dut, 1, 1_000_000, 0x10)


@cocotb.test()
async def test_spi_ram_mode2(dut):
    await spi_ram_case(dut, 2, 1_000_000, 0xF8)  # address wraps inside the 256-byte RAM


@cocotb.test()
async def test_spi_ram_mode3_fast(dut):
    """5 MHz SCK (10 cycles per bit)."""
    await spi_ram_case(dut, 3, 5_000_000, 0x80)


async def spi_target_case(dut, mode):
    host = await start(dut)
    data = [0x11, 0x22, 0xC3, 0x7E, 0x00, 0xFF, 0x96, 0x01]
    ctl = SpiController(dut, mode, sck_ui=2, mosi_ui=3, miso_uo=5, half=16)
    await load(host, "spi_target", [mode, len(data)])
    await host.ctrl(0x01)
    await wait_irq(dut, 1, timeout=5_000)
    await ClockCycles(dut.clk, 20)
    rx = await ctl.xfer(data, gap=8)
    await wait_irq(dut, 0, timeout=5_000)

    expect = [0xA5, 0x5A] + [(~b) & 0xFF for b in data[:-2]]
    assert rx == expect, f"MISO {[hex(x) for x in rx]} != {[hex(x) for x in expect]}"
    res = await read_shared(host, 0, 1)
    got = b"".join(w.to_bytes(4, "little") for w in await read_shared(host, 8, 2))
    assert res[0] == 0x600D0002, f"firmware result {res[0]:#010x}"
    assert list(got) == data, f"engine received {got.hex()}"


@cocotb.test()
async def test_spi_target_mode0(dut):
    await spi_target_case(dut, 0)


@cocotb.test()
async def test_spi_target_mode3(dut):
    await spi_target_case(dut, 3)
