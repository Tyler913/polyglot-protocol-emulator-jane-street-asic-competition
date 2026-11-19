# Copyright (c) 2026 Tyler Hong
# SPDX-License-Identifier: Apache-2.0
"""OBD-II over K-line, ISO 14230-4 fast init (sw/examples/kline), judged by
an independent ECU model (lib/kline.py): wake-up pattern timing (TiniL,
TWuP, W5), tester byte spacing P4 and request spacing P3, read-back on the
single wire, checksums, StartCommunication, then engine and vehicle speed."""

import cocotb

from lib.kline import KLineEcu
from lib.soc import load, read_shared, start, wait_irq


@cocotb.test()
async def test_kline_obd(dut):
    host = await start(dut)
    ecu = KLineEcu(dut, tx_uo=5, rx_ui=2)
    ecu.start()
    await load(host, "kline")
    await host.ctrl(0x01)
    await wait_irq(dut, 1, timeout=10_000)
    await wait_irq(dut, 0, timeout=40_000_000)
    ecu.stop()
    res = await read_shared(host, 0, 8)
    dut._log.info("requests %s, result %#x", ecu.requests, res[2])
    assert not ecu.errors, ecu.errors[:5]
    assert ecu.requests == ["c133f18166", "c233f1010cf3", "c233f1010df4"], ecu.requests
    assert res[2] == 0x600D000D, hex(res[2])
    assert (res[3], res[4], res[5], res[6]) == (0xEF8F, 0x1AF8, 88, 0x10), [hex(x) for x in res[3:7]]
