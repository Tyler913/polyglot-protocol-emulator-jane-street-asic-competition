# Copyright (c) 2026 Tyler Hong
# SPDX-License-Identifier: Apache-2.0
"""A USB low-speed HID keyboard in plain C (sw/examples/usb_hid) is enumerated
by an independent USB host model (test/lib/usb.py) and types "Hi".

Checks: bus reset, GET_DESCRIPTOR (device, configuration, HID report) at
address 0 and 5, SET_ADDRESS, SET_CONFIGURATION, SET_IDLE, interrupt IN
reports with alternating data toggles; every device answer starts within
the USB turnaround limit (7.5 bit times) and every packet passes the line,
stuffing and CRC checks of the bus monitor.
"""

import cocotb

from lib.soc import F_CPU, load, read_shared, start, wait_irq
from lib.usb import UsbBus, UsbHost, UsbMonitor

BIT = F_CPU / 1_500_000

DEV_DESC = bytes([18, 1, 0x10, 0x01, 0, 0, 0, 8, 0x09, 0x12, 0x01, 0x00, 0x00, 0x01, 0, 0, 0, 1])
REPORTS = [bytes([0x02, 0, 0x0B, 0, 0, 0, 0, 0]), bytes(8), bytes([0, 0, 0x0C, 0, 0, 0, 0, 0]), bytes(8)]


def req(rtype, request, value, index, length):
    return bytes([rtype, request, value & 0xFF, value >> 8, index & 0xFF, index >> 8, length & 0xFF, length >> 8])


@cocotb.test()
async def test_usb_hid_enumeration(dut):
    host = await start(dut)
    bus = UsbBus(dut, dm=4, dp=5, bit=BIT)
    mon = UsbMonitor(bus)
    bus.start()
    await load(host, "usb_hid")
    await host.ctrl(0x01)
    await wait_irq(dut, 1, timeout=20_000)

    h = UsbHost(bus, mon)
    await h.reset(500)  # 10 us SE0 (a real host holds 10 ms; scaled for simulation)
    try:
        await enumerate_and_type(dut, h, host, bus, mon)
    except AssertionError:
        for p in mon.packets[-12:]:
            dut._log.info(f"t={p['start']} {p['bytes'].hex()} {p['errors']}")
        dut._log.info(f"shared {await read_shared(host, 1, 5)}")
        raise


async def enumerate_and_type(dut, h, host, bus, mon):
    dev = await h.control_read(0, req(0x80, 6, 0x0100, 0, 64))
    assert dev == DEV_DESC, f"device descriptor {dev.hex()}"
    await h.control_write(0, req(0x00, 5, 5, 0, 0))  # SET_ADDRESS 5

    assert await h.control_read(5, req(0x80, 6, 0x0100, 0, 18)) == DEV_DESC
    conf = await h.control_read(5, req(0x80, 6, 0x0200, 0, 255))
    assert len(conf) == 34 and conf[1] == 2 and conf[2] == 34, f"configuration descriptor {conf.hex()}"
    assert conf[9 + 5 : 9 + 8] == bytes([3, 1, 1]), "not a HID boot keyboard interface"
    report = await h.control_read(5, req(0x81, 6, 0x2200, 0, conf[9 + 9 + 7]))
    assert len(report) == 63 and report[:4] == bytes([0x05, 0x01, 0x09, 0x06]), f"report descriptor {report.hex()}"

    await h.control_write(5, req(0x00, 9, 1, 0, 0))  # SET_CONFIGURATION 1
    await h.control_write(5, req(0x21, 0x0A, 0, 0, 0))  # HID SET_IDLE

    got = []
    for i in range(4):
        await h.gap(40)
        got.append(await h.in_(5, 1, i & 1))
    await h.gap(40)
    bus.stop()

    assert not bus.errors, bus.errors[:5]
    assert not mon.errors, mon.errors[:5]
    assert got == REPORTS, f"reports {[r.hex() for r in got]}"
    worst = max(h.turnarounds)
    dut._log.info(f"{len(h.turnarounds)} device answers, turnaround {min(h.turnarounds):.2f}..{worst:.2f} bit times")
    assert 2.0 <= min(h.turnarounds) and worst <= 7.5, f"turnaround out of range: {h.turnarounds}"

    addr, config, nrep, resets, bad = await read_shared(host, 1, 5)
    assert (addr, config, resets, bad) == (5, 1, 1, 0), f"addr {addr} config {config} resets {resets} bad CRC {bad}"
