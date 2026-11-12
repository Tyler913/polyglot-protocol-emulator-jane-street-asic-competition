# Copyright (c) 2026 Tyler Hong
# SPDX-License-Identifier: Apache-2.0
"""WS2812 LEDs from a plain NRZ Wire Engine stream (sw/examples/ws2812):
a protocol with no hardware support, judged by an independent WS2812 timing
model (pulse widths and periods from the datasheet)."""

import cocotb

from lib.soc import CLK_NS, load, start, wait_irq
from lib.ws2812 import Ws2812Model


@cocotb.test()
async def test_ws2812_chain(dut):
    host = await start(dut)
    m = Ws2812Model(dut, uo_bit=3, clk_ns=CLK_NS)
    m.start()
    await load(host, "ws2812", [4])
    await host.ctrl(0x01)
    await wait_irq(dut, 1, timeout=20_000)
    m.stop()
    assert not m.errors, m.errors[:5]
    assert len(m.frames) == 1, f"{len(m.frames)} latched frames, {len(m.bits)} bits pending"
    leds = m.leds(m.frames[0])
    assert leds == [(0x00, 0xFF, 0x00), (0xFF, 0x00, 0x00), (0x00, 0x00, 0xFF), (0x12, 0x34, 0x56)], leds
