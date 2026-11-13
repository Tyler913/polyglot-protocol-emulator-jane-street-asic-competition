# Copyright (c) 2026 Tyler Hong
# SPDX-License-Identifier: Apache-2.0
"""Helpers to bring the chip up, load firmware and exchange data with it."""

import os
import shutil
import subprocess
from pathlib import Path

import cocotb
from cocotb.clock import Clock
from cocotb.triggers import ClockCycles, ReadOnly, RisingEdge

from .host import HostSpi
from .pins import bit, ui_pins

SW_DIR = Path(__file__).resolve().parents[2] / "sw"
PREBUILT_DIR = Path(__file__).resolve().parents[1] / "firmware"
CLK_NS = 20  # 50 MHz
F_CPU = 50_000_000
SHARED = 0x0F00  # PG_SHARED in protocol.h
HOST_IRQ = 6  # uo[6]


def build_firmware(name):
    """Return the image bytes for sw/{examples,tests}/<name>.

    With a RISC-V GCC on PATH (riscv-none-elf-gcc, or $CROSS + gcc) the image is
    built from source. Without one (e.g. the Tiny Tapeout CI runners), or with
    FIRMWARE_PREBUILT set, the committed copy in test/firmware/ is used instead;
    `make -C sw prebuilt` refreshes those copies."""
    cross = os.environ.get("CROSS", "riscv-none-elf-")
    if not os.environ.get("FIRMWARE_PREBUILT") and shutil.which(cross + "gcc"):
        subprocess.run(["make", "-s", "-C", str(SW_DIR), f"build/{name}.bin"], check=True)
        image = (SW_DIR / "build" / f"{name}.bin").read_bytes()
        prebuilt = PREBUILT_DIR / f"{name}.bin"
        if prebuilt.exists() and prebuilt.read_bytes() != image:
            cocotb.log.warning("test/firmware/%s.bin is stale: run `make -C sw prebuilt`", name)
        return image
    return (PREBUILT_DIR / f"{name}.bin").read_bytes()


async def start(dut, clk_ns=CLK_NS):
    """Start the clock (default 50 MHz) and reset the chip. Returns a HostSpi."""
    cocotb.start_soon(Clock(dut.clk, clk_ns, unit="ns").start())
    dut.ena.value = 1
    ui_pins(dut).reset(1 << 7)  # host CS_N idle high, all other inputs low
    dut.uio_in.value = 0
    dut.rst_n.value = 0
    await ClockCycles(dut.clk, 10)
    dut.rst_n.value = 1
    await ClockCycles(dut.clk, 5)
    return HostSpi(dut)


def words_to_bytes(words):
    return b"".join(int(w & 0xFFFFFFFF).to_bytes(4, "little") for w in words)


async def load(host, name, params=()):
    """Load firmware and the parameter words at PG_SHARED[0..]; the CPU stays halted."""
    image = build_firmware(name)
    await host.write(0x0000, image)
    await host.write(SHARED, words_to_bytes(list(params) + [0] * (64 - len(params))))


async def read_shared(host, index, count):
    data = await host.read(SHARED + 4 * index, 4 * count)
    return [int.from_bytes(data[4 * i : 4 * i + 4], "little") for i in range(count)]


def host_irq(dut):
    return bit(dut.uo_out, HOST_IRQ)


async def wait_irq(dut, level=1, timeout=100_000):
    """Wait until the firmware sets HOST_IRQ to level; return the cycles waited."""
    for n in range(timeout):
        await RisingEdge(dut.clk)
        await ReadOnly()
        if host_irq(dut) == level:
            await RisingEdge(dut.clk)
            return n
    raise AssertionError(f"HOST_IRQ did not become {level} within {timeout} cycles")
