# Copyright (c) 2026 Tyler Hong
# SPDX-License-Identifier: Apache-2.0
"""SDI-12 data recorder on a lite engine (sw/examples/sdi12), judged by an
independent SDI-12 sensor model (lib/sdi12.py): one wire shared both ways,
1200 baud 7E1 negative logic, BREAK and marking lengths, the recorder
releasing the line, and the command sequence ?!, aI!, aM!, service request,
aD0! with the values parsed."""

import cocotb

from lib.sdi12 import Sdi12Sensor
from lib.soc import load, read_shared, start, wait_irq


@cocotb.test()
async def test_sdi12_recorder(dut):
    host = await start(dut)
    s = Sdi12Sensor(dut, pin=1, address="3")
    s.start()
    await load(host, "sdi12")
    await host.ctrl(0x01)
    await wait_irq(dut, 1, timeout=10_000)
    await wait_irq(dut, 0, timeout=60_000_000)
    s.stop()
    res = await read_shared(host, 0, 24)
    dut._log.info("commands %s, sent %s, result %#x", s.commands, s.sent, res[2])
    assert not s.errors, s.errors[:5]
    assert s.commands == ["?!", "3I!", "3M!", "3D0!"], s.commands
    assert res[2] == 0x600D000C, hex(res[2])
    assert res[3] == ord("3")
    assert res[4] == 1 | (2 << 16), hex(res[4])
    vals = [v - (1 << 32) if v & 0x80000000 else v for v in res[5:7]]
    assert vals == [21500, -3250], vals
    ident = b"".join(w.to_bytes(4, "little") for w in res[16:22]).rstrip(b"\0").decode()
    assert ident == "314EXAMPLESOIL01100SN42", ident
