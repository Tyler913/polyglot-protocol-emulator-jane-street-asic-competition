# Copyright (c) 2026 Tyler Hong
# SPDX-License-Identifier: Apache-2.0
"""Bring-up test (gate G1): a C program built with stock GCC is loaded over
SPI, read back, started, and observed toggling a pin."""

import os
import subprocess
from pathlib import Path

import cocotb
from cocotb.clock import Clock
from cocotb.triggers import ClockCycles, RisingEdge

from lib.host import HostSpi

SW_DIR = Path(__file__).resolve().parent.parent / "sw"
CLK_NS = 20  # 50 MHz


def build_firmware(name):
    """Build sw/examples/<name> unless FIRMWARE_PREBUILT is set; return the image bytes."""
    if not os.environ.get("FIRMWARE_PREBUILT"):
        subprocess.run(["make", "-C", str(SW_DIR), f"build/{name}.bin"], check=True)
    return (SW_DIR / "build" / f"{name}.bin").read_bytes()


async def reset(dut):
    cocotb.start_soon(Clock(dut.clk, CLK_NS, unit="ns").start())
    dut.ena.value = 1
    dut.ui_in.value = 1 << 7  # CS_N idle high
    dut.uio_in.value = 0
    dut.rst_n.value = 0
    await ClockCycles(dut.clk, 10)
    dut.rst_n.value = 1
    await ClockCycles(dut.clk, 5)


@cocotb.test()
async def test_load_readback(dut):
    """Host writes a pattern into SRAM and reads it back unchanged."""
    await reset(dut)
    host = HostSpi(dut)
    pattern = bytes((i * 37 + 11) & 0xFF for i in range(64))
    await host.write(0x0100, pattern)
    back = await host.read(0x0100, len(pattern))
    assert back == pattern, f"read back {back.hex()} != {pattern.hex()}"
    assert await host.status() == 0x00, "CPU should be held in reset after power-up"


@cocotb.test()
async def test_memtest(dut):
    """C firmware exercises byte/halfword/word loads and stores through the bus fabric."""
    image = build_firmware("memtest")
    await reset(dut)
    host = HostSpi(dut)
    await host.write(0x0000, image)
    await host.write(0x0F00, bytes(8))  # clear the result words
    await host.ctrl(0x01)
    await ClockCycles(dut.clk, 6000)
    await host.ctrl(0x00)  # halt, then read the result
    res = await host.read(0x0F00, 8)
    marker = int.from_bytes(res[0:4], "little")
    errors = int.from_bytes(res[4:8], "little")
    assert marker == 0x600D0000, f"memtest marker {marker:#010x}, errors {errors}"


@cocotb.test()
async def test_blink(dut):
    """C firmware toggles uo[0] every 100 cycles without drift."""
    image = build_firmware("blink")
    await reset(dut)
    host = HostSpi(dut)

    await host.write(0x0000, image)
    assert await host.read(0x0000, len(image)) == image, "image read-back mismatch"

    await host.ctrl(0x01)  # RUN
    assert (await host.status()) & 0x03 == 0x01, "CPU should be running without a trap"

    # Record the cycle of each edge on uo[0]
    edges = []
    cycle = 0
    last = int(dut.uo_out.value) & 1
    while len(edges) < 9 and cycle < 5000:
        await RisingEdge(dut.clk)
        cycle += 1
        v = int(dut.uo_out.value) & 1
        if v != last:
            edges.append(cycle)
            last = v
    assert len(edges) == 9, f"expected 9 edges on uo[0], saw {len(edges)}"

    gaps = [b - a for a, b in zip(edges, edges[1:])]
    dut._log.info(f"edge gaps (cycles): {gaps}")
    assert all(80 <= g <= 120 for g in gaps), f"edge gap out of range: {gaps}"
    # Software deadlines accumulate (t += 100), so there is no long-term drift.
    total = edges[-1] - edges[0]
    assert abs(total - 800) <= 20, f"8 half-periods took {total} cycles, expected ~800"
