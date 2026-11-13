# Copyright (c) 2026 Tyler Hong
# SPDX-License-Identifier: Apache-2.0
"""Four Wire Engines at once (sw/tests/quad_uart): four independent UARTs,
each judged by its own UART model, with all four receivers busy at the
same time."""

import cocotb
from cocotb.triggers import ClockCycles

from lib.soc import load, read_shared, start, wait_irq
from lib.uart import UartDriver, UartMonitor, bit_cycles

BAUD = 1_000_000


@cocotb.test()
async def test_quad_uart(dut):
    """Engines 0..3 as UARTs at 1 Mbaud: banners, then concurrent RX and echo (+1)."""
    bit = bit_cycles(BAUD)
    host = await start(dut)
    mons = [UartMonitor(dut, (lambda n=n: (int(dut.uo_out.value) >> (n + 1)) & 1), bit) for n in range(4)]
    drvs = [UartDriver(dut, n, bit) for n in range(4)]
    await load(host, "quad_uart", [BAUD])
    for m in mons:
        m.start()
    await host.ctrl(0x01)
    await wait_irq(dut, 1, timeout=int(80 * bit))

    texts = [b"engine zero", b"ENGINE ONE!", b"two 2 two 2", b"three/3/3/3"]
    tasks = [cocotb.start_soon(drvs[n].send(texts[n])) for n in range(4)]
    for t in tasks:
        await t
    await ClockCycles(dut.clk, int(40 * bit))

    for n in range(4):
        want = b"W%d\r\n" % n + bytes(c + 1 for c in texts[n])
        assert not mons[n].errors, f"engine {n}: {mons[n].errors}"
        assert bytes(mons[n].bytes) == want, f"engine {n}: received {bytes(mons[n].bytes)!r}"
    echoed, framing = await read_shared(host, 1, 2)
    assert (echoed, framing) == (sum(map(len, texts)), 0), f"echoed {echoed}, framing errors {framing}"
