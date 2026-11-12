# Copyright (c) 2026 Tyler Hong
# SPDX-License-Identifier: Apache-2.0
"""UART built from a Wire Engine (sw/examples/uart_echo), judged by an
independent UART model: banner, echo with upper-casing, exact bit timing,
back-to-back frames and a framing error."""

import cocotb
from cocotb.triggers import ClockCycles

from lib.soc import load, read_shared, start, wait_irq
from lib.uart import UartDriver, UartMonitor, bit_cycles

TX_UO = 1  # chip TX on uo[1]
RX_UI = 0  # chip RX on ui[0]


async def run_echo(dut, baud, text, gap_bits=0):
    bit = bit_cycles(baud)
    host = await start(dut)
    mon = UartMonitor(dut, lambda: (int(dut.uo_out.value) >> TX_UO) & 1, bit)
    drv = UartDriver(dut, RX_UI, bit)
    await load(host, "uart_echo", [baud])
    mon.start()
    await host.ctrl(0x01)
    await wait_irq(dut, 1, timeout=150 * bit)  # banner (10 frames) blocks on the 4-deep TX FIFO

    banner = b"Polyglot\r\n"
    await drv.send(text, gap_bits=gap_bits)
    total = len(banner) + len(text)
    for _ in range(100 * bit):  # echoes trail the input by up to ~5 frames (banner backlog)
        if len(mon.bytes) >= total:
            break
        await ClockCycles(dut.clk, 1)
    await ClockCycles(dut.clk, 2 * bit)
    return host, mon, drv, banner


@cocotb.test()
async def test_uart_echo_1mbaud(dut):
    """1 Mbaud, bytes sent back to back with no idle time."""
    text = b"hello, wire engine 0! abc xyz"
    host, mon, drv, banner = await run_echo(dut, 1_000_000, text)
    got = bytes(mon.bytes)
    assert not mon.errors, mon.errors
    assert got == banner + text.upper(), f"received {got!r}"

    # A frame with a low stop bit must be reported, and the next byte still works
    await drv.send(b"q", stop=0)
    await ClockCycles(dut.clk, 3 * bit_cycles(1_000_000))
    await drv.send(b"k")
    await ClockCycles(dut.clk, 15 * bit_cycles(1_000_000))
    assert bytes(mon.bytes[-1:]) == b"K", f"after the bad frame: {bytes(mon.bytes)!r}"
    # The CPU is blocked in uart_getc (bus idle), so the host can read the counters
    echoed, framing = await read_shared(host, 1, 2)
    assert (echoed, framing) == (len(text) + 1, 1), f"echoed {echoed}, framing errors {framing}"


@cocotb.test()
async def test_uart_echo_115200(dut):
    """Standard 115200 baud (434 cycles per bit at 50 MHz)."""
    text = b"ok?"
    _, mon, _, banner = await run_echo(dut, 115_200, text, gap_bits=2)
    assert not mon.errors, mon.errors
    assert bytes(mon.bytes) == banner + b"OK?", f"received {bytes(mon.bytes)!r}"
