# Copyright (c) 2026 Tyler Hong
# SPDX-License-Identifier: Apache-2.0
"""SPI built from Wire Engines, judged by independent SPI peer models.

- Controller (sw/examples/spi_ram): writes and reads back an SPI RAM model
  in all four modes and two clock rates.
- Target with external clock and chip select (sw/tests/spi_target): a
  controller model clocks the engine, including an aborted frame.
"""

import cocotb
from cocotb.triggers import ClockCycles

from lib.soc import load, read_shared, start, wait_irq
from lib.spi import SpiController, SpiRam

PATTERN = bytes((0x3C + 17 * i) & 0xFF for i in range(16))


async def spi_ram_case(dut, mode, hz, addr, rx_delay=0):
    host = await start(dut)
    ram = SpiRam(dut, mode, sck=2, mosi=3, cs=4, miso_ui=1)
    await load(host, "spi_ram", [mode, hz, addr, rx_delay])
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


@cocotb.test()
async def test_spi_ram_mode0_12m5_delay(dut):
    """12.5 MHz SCK (4 cycles per bit = F/4): MISO sampled 3 cycles late (RX_DELAY)."""
    await spi_ram_case(dut, 0, 12_500_000, 0x20, rx_delay=3)


async def spi_target_case(dut, mode):
    host = await start(dut)
    ctl = SpiController(dut, mode, sck_ui=2, mosi_ui=3, miso_uo=5, cs_ui=4, half=16)
    await load(host, "spi_target", [mode])
    await host.ctrl(0x01)
    await wait_irq(dut, 1, timeout=5_000)
    await ClockCycles(dut.clk, 20)

    def expect(data):
        return ([0xA5, 0x5A] + [(~b) & 0xFF for b in data])[: len(data)]

    frame1 = [0x11, 0x22, 0xC3, 0x7E, 0x00, 0xFF]
    await ctl.select(True)
    rx1 = await ctl.xfer(frame1, gap=8)
    await ctl.select(False)
    await ClockCycles(dut.clk, 300)

    # An aborted frame: CS rises after 4 bits. The next frame must start clean.
    await ctl.select(True)
    await ctl.xfer([0xF0], bits=4)
    await ctl.select(False)
    await ClockCycles(dut.clk, 300)

    frame2 = [0x96, 0x01, 0x80]
    await ctl.select(True)
    rx2 = await ctl.xfer(frame2, gap=8)
    await ctl.select(False)
    await ClockCycles(dut.clk, 300)

    assert rx1 == expect(frame1), f"frame 1 MISO {[hex(x) for x in rx1]} != {[hex(x) for x in expect(frame1)]}"
    assert rx2 == expect(frame2), f"frame 2 MISO {[hex(x) for x in rx2]} != {[hex(x) for x in expect(frame2)]}"
    frames, last_n, total = await read_shared(host, 1, 3)
    got = b"".join(w.to_bytes(4, "little") for w in await read_shared(host, 8, 1))
    assert (frames, last_n, total) == (3, 3, 9), f"frames {frames}, last frame {last_n} bytes, total {total}"
    assert list(got[:3]) == frame2, f"last frame received {got[:3].hex()}"


@cocotb.test()
async def test_spi_target_mode0(dut):
    await spi_target_case(dut, 0)


@cocotb.test()
async def test_spi_target_mode3(dut):
    await spi_target_case(dut, 3)
