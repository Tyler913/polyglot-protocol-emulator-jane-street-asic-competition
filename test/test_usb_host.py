# Copyright (c) 2026 Tyler Hong
# SPDX-License-Identifier: Apache-2.0
"""The chip as a USB low-speed host (sw/examples/usb_host): it resets the bus
and enumerates an independent keyboard model (test/lib/usb_device.py), then
polls its interrupt endpoint through NAKs. Checks: every packet passes the
bus monitor (bit grid, stuffing, PID, CRC16), token CRC5, the request
sequence, the host's ACK within 7.5 bit times of our DATA, a reset longer
than 2.5 us, keep-alives, and what the firmware read."""

import cocotb

from lib.soc import F_CPU, load, read_shared, start, wait_irq
from lib.usb import UsbBus, UsbMonitor
from lib.usb_device import UsbKeyboard

BIT = F_CPU / 1_500_000
REPORTS = [bytes(8), bytes([0x02, 0, 0x0B, 0, 0, 0, 0, 0]), bytes(8), bytes([0, 0, 0x0C, 0, 0, 0, 0, 0])]


@cocotb.test()
async def test_usb_ls_host(dut):
    host = await start(dut)
    bus = UsbBus(dut, dm=4, dp=5, bit=BIT)
    mon = UsbMonitor(bus)
    kbd = UsbKeyboard(bus, mon, REPORTS, naks=2)
    bus.start()
    kbd.start()
    await load(host, "usb_host", [200, 1])  # 200 us reset, short recovery waits (simulation)
    await host.ctrl(0x01)
    try:
        await wait_irq(dut, 1, timeout=3_000_000)
    finally:
        bus.stop()
        kbd.stop()
        for p in mon.packets[-8:]:
            dut._log.info(f"t={p['start']} {p['bytes'].hex()} {p['errors']}")
    assert not bus.errors, bus.errors[:5]
    assert not mon.errors, mon.errors[:5]
    assert not kbd.errors, kbd.errors[:5]
    assert kbd.resets and kbd.resets[0] > 100, f"reset {kbd.resets}"
    reqs = [bytes(r[:2]) for r in kbd.requests]
    assert reqs == [b"\x80\x06", b"\x00\x05", b"\x80\x06", b"\x80\x06", b"\x00\x09"], reqs
    assert max(kbd.host_turnarounds) <= 7.5, kbd.host_turnarounds
    r = await read_shared(host, 0, 8)
    dut._log.info(f"results {[hex(x) for x in r]}; host turnaround {min(kbd.host_turnarounds):.2f}.."
                  f"{max(kbd.host_turnarounds):.2f} bit times, {kbd.keepalives} keep-alives")
    assert r[0] == 0x00021209, f"VID/PID {r[0]:#x}"
    assert r[1] == 34 and r[2] == 0x010103, r[1:3]
    assert r[3:5] == [0x000B0002, 0x000C0000], [hex(x) for x in r[3:5]]
    assert r[5] == 2 and r[6] == 0 and r[7] == 0x600D05B0, r[5:]
