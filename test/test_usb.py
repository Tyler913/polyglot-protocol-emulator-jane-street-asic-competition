# Copyright (c) 2026 Tyler Hong
# SPDX-License-Identifier: Apache-2.0
"""USB low-speed line layer built from a Wire Engine (sw/tests/usb_phy),
judged by an independent USB LS model (test/lib/usb.py).

1. The chip sends a DATA0 packet: SYNC, NRZI, stuffing (the payload has
   0xFF 0xFF), CRC16 appended by the engine, SE0/J end of packet, bit timing
   at 1.5 Mbit/s from a 50 MHz clock (33.33 cycles per bit, fractional).
2. The host model sends a DATA1 packet (1% slow); the chip decodes it
   (NRZI, destuffing, CRC16 residual) and its pre-queued ACK goes out by
   itself within the USB turnaround window (2 to 7.5 bit times).
"""

import cocotb
from cocotb.triggers import ClockCycles

from lib.soc import F_CPU, load, read_shared, start, wait_irq
from lib.usb import UsbBus, UsbMonitor, crc16_bytes, data_packet

BIT = F_CPU / 1_500_000
PAYLOAD_TX = bytes([0x80, 0x06, 0x00, 0x01, 0xFF, 0xFF, 0x40, 0x00])
PAYLOAD_RX = bytes([0x12, 0xFF, 0xFF, 0xFF, 0x00, 0x7E, 0x81, 0xC3])


@cocotb.test()
async def test_usb_ls_phy(dut):
    host = await start(dut)
    bus = UsbBus(dut, dm=4, dp=5, bit=BIT)
    mon = UsbMonitor(bus)
    bus.start()
    await load(host, "usb_phy")
    await host.ctrl(0x01)
    await wait_irq(dut, 1, timeout=60_000)

    assert len(mon.packets) == 1, f"expected the DATA0 packet, saw {len(mon.packets)}"
    p = mon.packets[0]
    assert p["bytes"] == bytes([0xC3]) + PAYLOAD_TX + crc16_bytes(PAYLOAD_TX), f"DATA0 bytes {p['bytes'].hex()}"

    await ClockCycles(dut.clk, 200)
    await bus.send(data_packet(0x4B, PAYLOAD_RX), rate_error=-0.01)
    await wait_irq(dut, 0, timeout=60_000)
    await ClockCycles(dut.clk, 100)
    bus.stop()

    assert not bus.errors, bus.errors[:5]
    assert not mon.errors, mon.errors[:5]
    assert len(mon.packets) == 3, f"expected DATA0, host DATA1, ACK; saw {len(mon.packets)}"
    rxp, ack = mon.packets[1], mon.packets[2]
    assert ack["bytes"] == bytes([0xD2]), f"handshake {ack['bytes'].hex()}"
    gap = (ack["start"] - rxp["eop_end"]) / BIT
    dut._log.info(f"ACK starts {gap:.2f} bit times after the host's EOP")
    assert 2.0 <= gap <= 7.5, f"turnaround {gap:.2f} bit times outside 2..7.5"

    n, residual, status = await read_shared(host, 1, 3)
    got = b"".join(w.to_bytes(4, "little") for w in await read_shared(host, 8, 4))[:n]
    expect = bytes([0x4B]) + PAYLOAD_RX + crc16_bytes(PAYLOAD_RX)  # SYNC consumed by the engine
    assert got == expect, f"chip received {got.hex()} != {expect.hex()}"
    assert residual == 0x800D, f"RX CRC residual {residual:#06x}"
    assert not status & (1 << 26), "stuff error flagged"
