# Copyright (c) 2026 Tyler Hong
# SPDX-License-Identifier: Apache-2.0
"""10BASE-T normal link pulses from the ping node (sw/examples/eth_ping): 52 ms
of idle line at 60 MHz, decoded by the transmit monitor (test/lib/eth.py)."""

import cocotb
from cocotb.triggers import Timer

from lib.eth import EthTxMonitor
from lib.soc import load, start


@cocotb.test()
async def test_eth_link_pulses(dut):
    """While idle the node sends normal link pulses: TD+ high for 100 ns
    (+/- 20 %), every 16 ms (8..24 ms per IEEE 802.3 clause 14)."""
    host = await start(dut, clk_ns=16.666)
    mon = EthTxMonitor(dut, tdp=6, tdn=7, half=3, tail_bits=2)
    mon.start()
    await load(host, "eth_ping")
    await host.ctrl(0x01)
    await Timer(52, unit="ms")
    mon.stop()
    assert not mon.errors, mon.errors[:5]
    starts = [t for t, _ in mon.link_pulses]
    widths = [w for _, w in mon.link_pulses]
    assert len(starts) == 3, mon.link_pulses
    assert all(5 <= w <= 7 for w in widths), widths  # 6 cycles = 100 ns at 60 MHz
    gaps_ms = [(b - a) * 16.666e-6 for a, b in zip(starts, starts[1:])]
    assert all(8 <= g <= 24 for g in gaps_ms), gaps_ms
