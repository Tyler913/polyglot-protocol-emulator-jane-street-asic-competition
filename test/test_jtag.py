# Copyright (c) 2026 Tyler Hong
# SPDX-License-Identifier: Apache-2.0
"""IEEE 1149.1 JTAG host (sw/examples/jtag): a lite Wire Engine shifts TDI/TDO
on TCK and C moves TMS between pushes, judged by an independent model of a
two-device chain (lib/jtag.py: 16-state TAP controllers, IR capture, IDCODE,
BYPASS, a user data register) that checks TMS/TDI setup and hold."""

import cocotb

from lib.jtag import JtagChain, TapDevice
from lib.soc import load, read_shared, start, wait_irq

ID0 = 0x4BA00477
ID1 = 0x06413041


async def run(dut, hz):
    host = await start(dut)
    dev0 = TapDevice(ir_len=4, idcode=ID0, idcode_ir=0xE, user_ir=0xA, user=0x96)
    dev1 = TapDevice(ir_len=5, idcode=ID1, idcode_ir=0x01, user_ir=0x08)
    chain = JtagChain(dut, tck=2, tms=4, tdi=3, tdo=1, devices=[dev0, dev1])
    chain.start()
    await load(host, "jtag", [hz])
    await host.ctrl(0x01)
    try:
        await wait_irq(dut, 1, timeout=400 * (50_000_000 // hz) + 50_000)
    finally:
        chain.stop()
        assert not chain.errors, chain.errors[:5]

    id1, id0, irs, ndev, user_old, user_back, status = await read_shared(host, 1, 7)
    assert status == 0x600D0000, f"status {status:#010x}"
    assert (id1, id0) == (ID1, ID0), f"IDCODEs {id1:#010x} {id0:#010x}"
    assert irs == 0x21, f"captured IR {irs:#05x}"
    assert ndev == 2, f"{ndev} devices in BYPASS"
    assert user_old == 0x96 and user_back == 0x5A, f"USER {user_old:#04x} {user_back:#04x}"
    assert dev0.user == 0xC3 and dev0.ir == 0xA and dev1.ir == 0x1F
    assert dev1.updates == [], "device 1 was not in BYPASS"
    dut._log.info("JTAG at %d Hz: %d TCK cycles", hz, chain.rises)


@cocotb.test()
async def test_jtag_1mhz(dut):
    await run(dut, 1_000_000)


@cocotb.test()
async def test_jtag_5mhz(dut):
    await run(dut, 5_000_000)
