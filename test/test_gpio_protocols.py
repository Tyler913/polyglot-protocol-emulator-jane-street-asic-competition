# Copyright (c) 2026 Tyler Hong
# SPDX-License-Identifier: Apache-2.0
"""Protocols done in C on plain GPIO, capture and timed GPIO, each against a
model written from its description:

* quadrature (sw/examples/quadrature): A/B Gray-code steps forward and back
  at 1 us and 0.5 us per edge; the position must match the steps.
* Wiegand 26-bit (sw/examples/wiegand): a reader model sends a card, the
  chip checks parity and sends it on; a monitor decodes the chip's DATA0/
  DATA1 pulses and checks pulse width (20..100 us) and spacing (0.2..20 ms).
* UART autobaud (sw/examples/uart_autobaud): 'U' at an unknown rate, then
  echo at the measured rate (strict bit-grid check of the echo)."""

import cocotb
from cocotb.triggers import ClockCycles, Timer

from lib.edges import EdgeRecorder
from lib.pins import ui_pins
from lib.soc import load, read_shared, start
from lib.uart import UartDriver, UartMonitor, bit_cycles


@cocotb.test()
async def test_quadrature(dut):
    host = await start(dut)
    pins = ui_pins(dut)
    pins.set(0, 0)
    pins.set(1, 0)
    await load(host, "quadrature")
    await host.ctrl(0x01)
    await Timer(20, unit="us")
    gray = [0b00, 0b01, 0b11, 0b10]  # (A << 0) | (B << 1): A leads B going forward
    state, pos = 0, 0
    for n, d, ns in ((100, +1, 1000), (37, -1, 500), (13, +1, 500), (60, -1, 1000)):
        for _ in range(n):
            state = (state + d) % 4
            pos += d
            pins.set(0, gray[state] & 1)
            pins.set(1, gray[state] >> 1)
            await Timer(ns, unit="ns")
    await Timer(5, unit="us")
    p, err, passes = await read_shared(host, 0, 3)
    p = p - (1 << 32) if p >> 31 else p
    assert err == 0, f"{err} invalid transitions seen"
    assert p == pos, f"position {p}, want {pos}"
    assert passes > 0


def wiegand26(facility, card):
    v = (facility << 16) | card
    hi, lo = v >> 12, v & 0xFFF
    pe = bin(hi).count("1") & 1          # even parity over the first 12 data bits
    po = 1 - (bin(lo).count("1") & 1)    # odd parity over the last 12
    return [pe] + [(v >> (23 - i)) & 1 for i in range(24)] + [po]


@cocotb.test()
async def test_wiegand(dut):
    host = await start(dut)
    pins = ui_pins(dut)
    pins.set(0, 1)
    pins.set(1, 1)
    d0, d1 = EdgeRecorder(dut, "uo_out", 0), EdgeRecorder(dut, "uo_out", 1)
    await load(host, "wiegand")
    await host.ctrl(0x01)
    await Timer(50, unit="us")
    d0.start()
    d1.start()
    bits = wiegand26(123, 45678)
    for b in bits:  # reader: 50 us pulses every 0.5 ms
        pins.set(b, 0)
        await Timer(50, unit="us")
        pins.set(b, 1)
        await Timer(450, unit="us")
    await Timer(17, unit="ms")
    r = await read_shared(host, 0, 4)
    assert r == [123, 45678, 1, 0x600D3126], r
    pulses = sorted([(t, 0, w) for t, w in d0.pulses(0)] + [(t, 1, w) for t, w in d1.pulses(0)])
    assert [b for _, b, _ in pulses] == bits, "bits sent on"
    for (t, _, w), (u, _, _) in zip(pulses, pulses[1:] + [(None, 0, 0)]):
        assert 20_000 <= w <= 100_000, f"pulse {w} ns"
        if u is not None:
            assert 200_000 <= u - t <= 20_000_000, f"interval {u - t} ns"


async def autobaud(dut, baud):
    host = await start(dut)
    bit = bit_cycles(baud)
    mon = UartMonitor(dut, lambda: (int(dut.uo_out.value) >> 1) & 1, bit)
    drv = UartDriver(dut, 0, bit)
    await load(host, "uart_autobaud")
    await host.ctrl(0x01)
    await ClockCycles(dut.clk, 200)
    mon.start()
    await drv.send(b"U")
    await ClockCycles(dut.clk, int(4 * bit))
    await drv.send(b"Hi!")
    await ClockCycles(dut.clk, int(25 * bit))
    mon.stop()
    fix8 = (await read_shared(host, 0, 1))[0]
    assert abs(fix8 / 256 - bit) < 0.2, f"{baud}: measured {fix8 / 256:.2f} cycles, want {bit:.2f}"
    assert not mon.errors, mon.errors
    assert bytes(mon.bytes) == b"UHi!", f"{baud}: {bytes(mon.bytes)}"


@cocotb.test()
async def test_uart_autobaud_57600(dut):
    await autobaud(dut, 57600)


@cocotb.test()
async def test_uart_autobaud_1m(dut):
    await autobaud(dut, 1_000_000)


@cocotb.test()
async def test_uart_autobaud_460800(dut):
    await autobaud(dut, 460800)
