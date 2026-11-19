# Copyright (c) 2026 Tyler Hong
# SPDX-License-Identifier: Apache-2.0
"""DHT22 / AM2302 on the capture unit (sw/examples/dht22), judged by an
independent sensor model (lib/dht22.py): start-signal length, 40 pulse-
width-coded bits, a negative temperature and a checksum error."""

import cocotb

from lib.dht22 import US, Dht22
from lib.soc import load, read_shared, start, wait_irq

READINGS = [(652, 235, False), (1000, -101, False), (473, 189, True)]


@cocotb.test()
async def test_dht22(dut):
    host = await start(dut)
    s = Dht22(dut, pin=3)
    s.readings = list(READINGS)
    s.start()
    await load(host, "dht22", [len(READINGS)])
    await host.ctrl(0x01)
    await wait_irq(dut, 1, timeout=10_000)
    await wait_irq(dut, 0, timeout=3_000_000)
    s.stop()
    assert not s.errors, s.errors
    assert all(1000 * US <= x <= 1200 * US for x in s.starts), s.starts
    res = await read_shared(host, 0, 16)
    assert res[2] == 0x600D000F, hex(res[2])
    for k, (h, t, bad) in enumerate(READINGS):
        v, st = res[8 + 2 * k], res[9 + 2 * k]
        temp = (v >> 16) - (1 << 16) if v >> 31 else v >> 16
        assert st == (2 if bad else 1), f"reading {k}: status {st}"
        assert (v & 0xFFFF, temp) == (h, t), f"reading {k}: {v & 0xFFFF}, {temp}"
