# Copyright (c) 2026 Tyler Hong
# SPDX-License-Identifier: Apache-2.0
"""10BASE-T transmit from a Wire Engine (sw/tests/eth_tx) at a 40 MHz clock,
judged by an independent IEEE 802.3 decoder (test/lib/eth.py): Manchester
symbols, differential pair, preamble/SFD, the CRC-32 FCS (zlib) and the
TP_IDL tail."""

import cocotb

from lib.eth import EthTxMonitor
from lib.soc import load, start, wait_irq

FRAME = (bytes.fromhex("ffffffffffff020000000001") + b"\x88\xb5"
         + b"Polyglot 10BASE-T from plain C".ljust(46, b"\0"))


@cocotb.test()
async def test_eth_10baset_tx(dut):
    host = await start(dut, clk_ns=25)  # 40 MHz
    mon = EthTxMonitor(dut, tdp=6, tdn=7, half=2, tail_bits=2)
    mon.start()
    await load(host, "eth_tx")
    await host.ctrl(0x01)
    await wait_irq(dut, 1, timeout=20_000)
    mon.stop()

    assert not mon.errors, mon.errors[:5]
    assert len(mon.frames) == 1, f"expected one frame, saw {len(mon.frames)}"
    data, fcs_ok = mon.frames[0]
    assert data[:-4] == FRAME, f"frame {data[:-4].hex()}"
    assert fcs_ok, f"FCS {data[-4:].hex()} wrong"
