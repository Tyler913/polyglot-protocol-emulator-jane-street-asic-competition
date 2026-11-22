# Copyright (c) 2026 Tyler Hong
# SPDX-License-Identifier: Apache-2.0
"""ISO 7816-3 smart-card reader (sw/examples/iso7816), judged by an
independent card model (lib/smartcard.py) that times everything in card
clocks: the card clock must run without gaps while C also handles the I/O
line; reset, ATR, GET CHALLENGE with procedure byte and status word."""

import cocotb

from lib.smartcard import SmartCard
from lib.soc import load, read_shared, start, wait_irq

ATR = [0x3B, 0x02, 0x14, 0x50]
CHAL = [0x1F, 0x2E, 0x3D, 0x4C, 0x5B, 0x6A, 0x79, 0x88]


@cocotb.test()
async def test_iso7816(dut):
    await run(dut)


async def run(dut):
    host = await start(dut)
    card = SmartCard(dut, clk_uo=0, rst_uo=1, io_uio=2, atr=ATR, challenge=CHAL)
    card.start()
    await load(host, "iso7816")
    await host.ctrl(0x01)
    await wait_irq(dut, 1, timeout=10_000)
    await wait_irq(dut, 0, timeout=20_000_000)
    card.stop()
    dut._log.info("card clock periods %s, card received %s", card.periods, bytes(card.received).hex())
    assert not card.errors, card.errors[:5]
    assert card.periods[1:] == [50, 50, 50], card.periods
    assert card.received == [0x00, 0x84, 0x00, 0x00, 0x08], card.received
    res = await read_shared(host, 0, 12)
    dut._log.info("chip results %s", [hex(x) for x in res[2:12]])
    assert res[2] == 0x600D0015, hex(res[2])
    assert res[3] == 4 and res[4] == int.from_bytes(bytes(ATR), "little"), [hex(x) for x in res[3:5]]
    assert res[8] == 0x9000 and res[11] == 0, [hex(x) for x in res[8:12]]
    got = (res[9] | res[10] << 32).to_bytes(8, "little")
    assert list(got) == CHAL, got.hex()
