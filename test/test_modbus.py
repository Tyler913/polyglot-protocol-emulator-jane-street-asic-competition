# Copyright (c) 2026 Tyler Hong
# SPDX-License-Identifier: Apache-2.0
"""Modbus RTU client (sw/examples/modbus_master) against an independent
server model (test/lib/modbus.py): CRC-16, RTU silent intervals and
inter-byte gaps, write single register, read holding registers, and an
exception reply."""

import cocotb

from lib.modbus import ModbusServer
from lib.soc import load, read_shared, start, wait_irq


@cocotb.test()
async def test_modbus_rtu(dut):
    host = await start(dut)
    srv = ModbusServer(dut, tx_uo=1, rx_ui=0, baud=115200, address=0x11,
                       regs={0: 0xBEEF, 1: 0x0001, 2: 0x8000, 3: 0x7FFF, 5: 0})
    srv.start()
    await load(host, "modbus_master")
    await host.ctrl(0x01)
    await wait_irq(dut, 1, timeout=1_500_000)
    srv.stop()
    assert not srv.errors, srv.errors
    assert len(srv.requests) == 3, srv.requests
    assert srv.regs[5] == 0x1234
    r = await read_shared(host, 0, 7)
    assert r == [1, 0xBEEF, 0x0001, 0x8000, 0x7FFF, 2, 0x600D0B05], [hex(x) for x in r]
