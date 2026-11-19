# Copyright (c) 2026 Tyler Hong
# SPDX-License-Identifier: Apache-2.0
"""BiSS-C master on the SPI engine (sw/examples/biss), judged by an
independent BiSS-C encoder model (lib/biss.py): 26-bit positions, error and
warning bits, CRC-6, a corrupted CRC detected, and SL delays of 7 and 37
cycles (0.7 and 3.7 clock periods of cable delay) found by the Start bit."""

import cocotb

from lib.biss import BissEncoder
from lib.soc import load, read_shared, start, wait_irq

FRAMES = [(0x2ABCDEF, 1, 1, False), (0x0000001, 1, 1, False), (0x3FFFFFF, 0, 1, False),
          (0x1234567, 1, 0, False), (0x1555555, 1, 1, True), (0x0F0F0F0, 1, 1, False)]


async def run(dut, delay):
    host = await start(dut)
    enc = BissEncoder(dut, ma_uo=0, sl_ui=3, nbits=26, delay=delay)
    enc.queue = list(FRAMES)
    enc.start()
    await load(host, "biss", [len(FRAMES), 26])
    await host.ctrl(0x01)
    await wait_irq(dut, 1, timeout=10_000)
    try:
        await wait_irq(dut, 0, timeout=200_000)
    finally:
        enc.stop()
        r = enc.rises[:60]
        dut._log.info("frames served %d, MA rise gaps %s, errors %s", len(enc.served),
                      [b - a for a, b in zip(r, r[1:])], enc.errors[:3])
    assert not enc.errors, enc.errors[:5]
    assert enc.max_gap == 10, f"MA period up to {enc.max_gap} cycles inside a frame (5 MHz = 10)"
    res = await read_shared(host, 0, 8 + 2 * len(FRAMES))
    assert res[2] == 0x600D000E, hex(res[2])
    for k, (pos, ne, nw, bad) in enumerate(FRAMES):
        p, st = res[8 + 2 * k], res[9 + 2 * k]
        want = 8 | (0 if bad else 1) | (ne << 1) | (nw << 2)
        assert (p, st & 0xFF) == (pos, want), f"frame {k}: pos {p:#x} status {st:#x}"
        dut._log.info("frame %d: start bit after %d clocks", k, st >> 8)


@cocotb.test()
async def test_biss_short_cable(dut):
    await run(dut, delay=7)


@cocotb.test()
async def test_biss_long_cable(dut):
    await run(dut, delay=37)
