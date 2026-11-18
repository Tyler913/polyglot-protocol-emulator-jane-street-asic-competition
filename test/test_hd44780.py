# Copyright (c) 2026 Tyler Hong
# SPDX-License-Identifier: Apache-2.0
"""HD44780 character LCD (sw/examples/hd44780) on the 4-bit bus from timed
GPIO, against an independent controller model (test/lib/hd44780.py) that
checks every bus timing, the initialization sequence and execution times."""

import cocotb

from lib.hd44780 import Hd44780
from lib.soc import load, read_shared, start, wait_irq


@cocotb.test()
async def test_hd44780(dut):
    host = await start(dut)
    lcd = Hd44780(dut)
    lcd.start()
    await load(host, "hd44780", [200])  # 200 us power-on wait instead of 40 ms (sim time)
    await host.ctrl(0x01)
    await wait_irq(dut, 1, timeout=1_000_000)
    lcd.stop()
    assert not lcd.errors, lcd.errors[:5]
    assert lcd.lines2 and lcd.display_on
    assert lcd.text(0) == "Polyglot", lcd.text(0)
    assert lcd.text(1) == "RISC-V C", lcd.text(1)
    assert (await read_shared(host, 1, 1))[0] == 0x600D44C0
