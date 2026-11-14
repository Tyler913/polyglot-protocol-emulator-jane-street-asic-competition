# Copyright (c) 2026 Tyler Hong
# SPDX-License-Identifier: Apache-2.0
"""ARM Serial Wire Debug host (sw/examples/swd) on a lite Wire Engine, judged by
an independent SW-DP + MEM-AP target model (lib/swd.py): JTAG-to-SWD switch,
DPIDR, FAULT + ABORT, power-up handshake, posted AP reads, memory writes and
reads through TAR/DRW with WAIT retries, and no contention at turnarounds."""

import cocotb

from lib.soc import load, read_shared, start, wait_irq
from lib.swd import ACK_FAULT, SwdTarget

DPIDR = 0x2BA01477
APIDR = 0x24770011
CPUID = 0x410FC241


async def run(dut, hz, wait_drw):
    host = await start(dut)
    tgt = SwdTarget(dut, swclk=0, swdio=1, dpidr=DPIDR, apidr=APIDR, wait_drw=wait_drw)
    tgt.mem[0xE000ED00] = CPUID
    tgt.start()
    await load(host, "swd", [hz])
    await host.ctrl(0x01)
    try:
        await wait_irq(dut, 1, timeout=3000 * (50_000_000 // hz) + 50_000)
    finally:
        tgt.stop()
        assert not tgt.errors, tgt.errors[:5]  # a protocol error explains a hang best

    assert tgt.mode == "swd", f"target never switched to SWD ({tgt.mode})"
    r = await read_shared(host, 1, 11)
    dpidr, apidr, ctrl_stat, w0, w1, w2, w3, cpuid, retries, fault_ack, status = r
    expect = [0xC0DE0000 + 0x1111 * i for i in range(4)]
    assert dpidr == DPIDR, f"DPIDR {dpidr:#010x}"
    assert fault_ack == ACK_FAULT, f"AP access before power-up answered {fault_ack}"
    assert ctrl_stat & 0xF0000000 == 0xF0000000, f"CTRL/STAT {ctrl_stat:#010x}"
    assert apidr == APIDR, f"AP IDR {apidr:#010x}"
    assert [tgt.mem.get(0x20000000 + 4 * i) for i in range(4)] == expect, "target memory not written"
    assert [w0, w1, w2, w3] == expect, [hex(x) for x in (w0, w1, w2, w3)]
    assert cpuid == CPUID, f"CPUID {cpuid:#010x}"
    assert retries == wait_drw, f"{retries} WAIT retries, expected {wait_drw}"
    assert status == 0x600D0000, f"status {status:#010x}"
    waits = [e for e in tgt.log if e[0] in ("read", "write") and e[4] == 2]
    assert len(waits) == wait_drw
    dut._log.info("SWD at %d Hz: %d packets, %d WAIT", hz,
                  sum(e[0] in ("read", "write") for e in tgt.log), len(waits))


@cocotb.test()
async def test_swd_1mhz_wait(dut):
    await run(dut, 1_000_000, wait_drw=3)


@cocotb.test()
async def test_swd_5mhz(dut):
    await run(dut, 5_000_000, wait_drw=0)
