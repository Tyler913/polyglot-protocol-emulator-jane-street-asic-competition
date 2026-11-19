# Copyright (c) 2026 Tyler Hong
# SPDX-License-Identifier: Apache-2.0
"""IO-Link master port on a lite engine (sw/examples/iolink_master), judged
by an independent IO-Link device model (lib/iolink.py): wake-up pulse width,
transmission-rate search (COM3 fails, the COM2 device answers), A.1.6
checksums both ways, Direct Parameter page 1 read in STARTUP, MasterCycleTime
and MasterCommands (DevicePreoperate, DeviceOperate), then cyclic TYPE_2
process data in OPERATE at the MasterCycleTime (the device's MinCycleTime,
2.3 ms, typical for COM2)."""

import cocotb

from lib.iolink import IoLinkDevice
from lib.soc import load, read_shared, start, wait_irq

PAGE = {0x02: 0x17, 0x03: 0x05, 0x04: 0x11, 0x05: 0x10, 0x06: 0x00,
        0x07: 0x01, 0x08: 0x2C, 0x09: 0x12, 0x0A: 0x34, 0x0B: 0x56}
PD = [bytes([0x03, 0xE8 + k]) for k in range(4)]


@cocotb.test()
async def test_iolink_master(dut):
    host = await start(dut)
    dev = IoLinkDevice(dut, txd_uo=4, rxd_ui=1, page=PAGE, pd_values=PD)
    dev.start()
    await load(host, "iolink_master", [4])
    await host.ctrl(0x01)
    await wait_irq(dut, 1, timeout=10_000)
    await wait_irq(dut, 0, timeout=3_000_000)
    dev.stop()
    for st, m, r in dev.log:
        dut._log.info("%-10s %-8s -> %s", st, m, r)
    assert not dev.errors, dev.errors[:5]
    res = await read_shared(host, 0, 32)
    assert res[2] == 0x600D000B, hex(res[2])
    assert res[3] == 38400, f"rate {res[3]}"
    assert res[4] == 1, f"{res[4]} failed messages (want 1: the COM3 attempt)"
    assert list(res[8:18]) == [PAGE[a] for a in range(2, 12)], [hex(x) for x in res[8:18]]
    assert dev.state == "OPERATE" and dev.master_cycle == 0x17
    assert list(res[20:24]) == [int.from_bytes(p, "big") for p in PD], [hex(x) for x in res[20:24]]
    assert list(res[28:32]) == [0] * 4, res[28:32]
    gaps = [b - a for a, b in zip(dev.operate_times, dev.operate_times[1:])]
    dut._log.info("OPERATE cycle starts every %s cycles", gaps)
    assert all(abs(g - 115_000) <= 200 for g in gaps), gaps       # MasterCycleTime 2.3 ms
