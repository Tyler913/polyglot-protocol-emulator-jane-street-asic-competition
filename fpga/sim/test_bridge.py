# Copyright (c) 2026 Tyler Hong
# SPDX-License-Identifier: Apache-2.0
"""The FPGA UART-to-host-SPI bridge with the chip: a PC-side model sends
requests over the UART (2 Mbaud in simulation) exactly as
tools/polyglot_host.py UartBridge does, loads a program, reads it back, runs
it and watches HOST_IRQ through the status request."""

import sys
from pathlib import Path

import cocotb
from cocotb.clock import Clock
from cocotb.triggers import ClockCycles, FallingEdge, Timer

sys.path.insert(0, str(Path(__file__).resolve().parents[2] / "test"))
from lib.soc import build_firmware  # noqa: E402

DIV = 25  # 50 MHz / 2 Mbaud
CHUNK = 32


class PcUart:
    def __init__(self, dut):
        self.dut = dut
        dut.rxd.value = 1
        self.rx = []
        cocotb.start_soon(self._receiver())

    async def send(self, b):
        for bit in [0] + [(b >> i) & 1 for i in range(8)] + [1]:
            self.dut.rxd.value = bit
            await ClockCycles(self.dut.clk, DIV)

    async def _receiver(self):
        while True:
            await FallingEdge(self.dut.txd)
            await ClockCycles(self.dut.clk, DIV // 2)
            assert int(self.dut.txd.value) == 0, "start bit glitch"
            v = 0
            for i in range(8):
                await ClockCycles(self.dut.clk, DIV)
                v |= int(self.dut.txd.value) << i
            await ClockCycles(self.dut.clk, DIV)
            assert int(self.dut.txd.value) == 1, "framing error"
            self.rx.append(v)

    async def recv(self, n):
        for _ in range(200_000):
            if len(self.rx) >= n:
                out, self.rx = self.rx[:n], self.rx[n:]
                return bytes(out)
            await ClockCycles(self.dut.clk, 10)
        raise AssertionError(f"bridge returned {len(self.rx)} of {n} bytes")

    async def transfer(self, data):
        """One SPI transaction, sent in chunks; each chunk's reply is read before the next."""
        n = len(data)
        await self.send(n >> 8)
        await self.send(n & 0xFF)
        out = b""
        for k in range(0, n, CHUNK):
            part = data[k:k + CHUNK]
            for b in part:
                await self.send(b)
            out += await self.recv(len(part))
        return out

    async def status(self):
        await self.send(0)
        await self.send(0)
        return (await self.recv(1))[0]


@cocotb.test()
async def test_bridge_load_run(dut):
    cocotb.start_soon(Clock(dut.clk, 20, unit="ns").start())
    dut.rst_n.value = 0
    dut.rxd.value = 1
    await ClockCycles(dut.clk, 10)
    dut.rst_n.value = 1
    pc = PcUart(dut)
    await ClockCycles(dut.clk, 10)

    assert await pc.status() == 0, "HOST_IRQ high after reset"
    image = build_firmware("ws2812")
    await pc.transfer(bytes([0x03, 0]))                       # halt
    await pc.transfer(bytes([0x01, 0, 0]) + image)            # write the program
    params = (4).to_bytes(4, "little") + bytes(252)
    await pc.transfer(bytes([0x01, 0x0F, 0x00]) + params)     # PG_SHARED[0] = 4 LEDs
    back = (await pc.transfer(bytes([0x02, 0, 0, 0]) + bytes(len(image))))[4:]
    assert back == image, "read-back mismatch"
    await pc.transfer(bytes([0x03, 1]))                       # run
    for _ in range(100):
        if await pc.status() == 1:
            break
        await Timer(10, unit="us")
    else:
        raise AssertionError("HOST_IRQ never rose")
    dut._log.info("loaded %d bytes, read back, ran; HOST_IRQ seen over the bridge", len(image))


@cocotb.test()
async def test_bridge_hello_running(dut):
    """The bring-up program: HOST_IRQ toggles and the counter in PG_SHARED[1]
    can be read over the bridge while the CPU runs."""
    cocotb.start_soon(Clock(dut.clk, 20, unit="ns").start())
    dut.rst_n.value = 0
    dut.rxd.value = 1
    await ClockCycles(dut.clk, 10)
    dut.rst_n.value = 1
    pc = PcUart(dut)
    await ClockCycles(dut.clk, 10)

    image = build_firmware("hello")
    await pc.transfer(bytes([0x03, 0]))
    await pc.transfer(bytes([0x01, 0, 0]) + image)
    await pc.transfer(bytes([0x01, 0x0F, 0x00]) + (20).to_bytes(4, "little") + bytes(4))  # 20 us
    await pc.transfer(bytes([0x03, 1]))
    seen, counts = set(), []
    for _ in range(6):
        seen.add(await pc.status())
        r = await pc.transfer(bytes([0x02, 0x0F, 0x04, 0]) + bytes(4))
        counts.append(int.from_bytes(r[4:], "little"))
        await Timer(30, unit="us")
    assert seen == {0, 1}, f"HOST_IRQ levels seen: {seen}"
    assert counts == sorted(counts) and counts[-1] > counts[0], counts
    dut._log.info("hello counter read while running: %s", counts)
