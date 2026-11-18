# Copyright (c) 2026 Tyler Hong
# SPDX-License-Identifier: Apache-2.0
"""SD card in SPI mode (sw/examples/sd_spi): initialization at 400 kHz and a
block read at 12.5 MHz against an independent card model (test/lib/sdcard.py)
that checks every command's CRC7, which the engine's CRC unit appends."""

import cocotb

from lib.sdcard import SdCard, block_data
from lib.soc import load, read_shared, start, wait_irq


@cocotb.test()
async def test_sd_spi(dut):
    host = await start(dut)
    card = SdCard(dut, sck=2, mosi=3, cs=4, miso_ui=1)
    card.start()
    await load(host, "sd_spi")
    await host.ctrl(0x01)
    try:
        await wait_irq(dut, 1, timeout=3_000_000)
    finally:
        card.stop()
        dut._log.info("commands %s errors %s times %s", card.cmds, card.errors, card.cmd_times)
    assert not card.errors, card.errors
    assert [c[0] for c in card.cmds] == [0, 8, 55, 41, 55, 41, 55, 41, 58, 17], card.cmds
    r = await read_shared(host, 0, 8)
    want = list(int.from_bytes(block_data(7)[4 * i : 4 * i + 4], "little") for i in range(4))
    assert r == [1, 0xC0FF8000, 1] + want + [0x600D05D0], [hex(x) for x in r]
