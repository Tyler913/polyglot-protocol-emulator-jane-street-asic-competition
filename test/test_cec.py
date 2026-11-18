# Copyright (c) 2026 Tyler Hong
# SPDX-License-Identifier: Apache-2.0
"""HDMI-CEC initiator (sw/examples/cec) against an independent follower
(test/lib/cec.py) at logical address 0: start and data bit timing, EOM,
the follower's ACK stretching and the initiator's reading of it, and a
message to an absent address (no ACK: the initiator stops after the header)."""

import cocotb

from lib.cec import CecFollower
from lib.soc import load, read_shared, start, wait_irq


@cocotb.test()
async def test_cec(dut):
    host = await start(dut)
    tv = CecFollower(dut, pin=2, address=0)
    tv.start()
    await load(host, "cec")
    await host.ctrl(0x01)
    await wait_irq(dut, 1, timeout=12_000_000)
    tv.stop()
    frames = tv.decode()
    assert not tv.errors, tv.errors[:5]
    assert frames == [(b"\x40\x04", [True, True]), (b"\x40\x47PG", [True] * 4), (b"\x49", [False])], frames  # no ACK: the initiator stops
    r = await read_shared(host, 0, 4)
    assert r == [1, 1, 0, 0x600D0CEC], r
