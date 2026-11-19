# Copyright (c) 2026 Tyler Hong
# SPDX-License-Identifier: Apache-2.0
"""Independent USB low-speed device (a HID boot keyboard) for testing the chip
as a host, written from USB 2.0 chapters 8 and 9 and the HID 1.11 boot
protocol, not from the RTL. It uses the line model in usb.py (UsbBus drives
the bus for it, UsbMonitor decodes every packet) and checks what a device
can see of a host: token CRC5, DATA PID/toggle sequence and CRC16 (by the
monitor), SETUP followed by DATA0, the host's handshake turnaround after
our DATA (<= 7.5 bit times), bus reset length and keep-alives (an EOP at
least every 3 ms, or the device would suspend).

Endpoint 1 answers NAK the first `naks` times, then returns the reports.
At full speed (rate=12e6) the device also records every start-of-frame
token (time, frame number) and checks its CRC5."""

import cocotb
from cocotb.triggers import ClockCycles

from .usb import J, K, SE0, crc_bits, data_packet

PID_OUT, PID_IN, PID_SETUP = 0xE1, 0x69, 0x2D
PID_DATA0, PID_DATA1, PID_ACK, PID_NAK = 0xC3, 0x4B, 0xD2, 0x5A

DEV_DESC = bytes([18, 1, 0x10, 0x01, 0, 0, 0, 8, 0x09, 0x12, 0x02, 0x00, 0x00, 0x01, 0, 0, 0, 1])
CONF_DESC = bytes([9, 2, 34, 0, 1, 1, 0, 0x80, 50,
                   9, 4, 0, 0, 1, 3, 1, 1, 0,
                   9, 0x21, 0x11, 0x01, 0, 1, 0x22, 63, 0,
                   7, 5, 0x81, 3, 8, 0, 10])


def crc5_ok(field16):
    bits = [(field16 >> i) & 1 for i in range(11)]
    c = crc_bits(bits, 5, 0x05, 0x1F) ^ 0x1F
    sent = (field16 >> 11) & 0x1F  # LSB-first on the wire: bit 11 is the CRC's MSB
    return sent == sum(((c >> (4 - i)) & 1) << i for i in range(5))


class UsbKeyboard:
    def __init__(self, bus, mon, reports, naks=2, turn_bits=3, rate=1.5e6):
        self.bus, self.mon, self.bit = bus, mon, bus.bit
        self.rate = rate
        self.sofs = []
        self.clk = bus.dut.clk
        self.reports = list(reports)
        self.naks = naks
        self.turn = turn_bits
        self.addr, self.config = 0, 0
        self.errors, self.requests = [], []
        self.host_turnarounds = []
        self.resets, self.keepalives = [], 0
        self._se0_since, self._last_eop = None, None
        bus.listeners.append(self._line)

    def start(self):
        self._task = cocotb.start_soon(self._run())

    def stop(self):
        self._task.cancel()

    def _line(self, t, s):
        if s == SE0 and self._se0_since is None:
            self._se0_since = t
        elif s != SE0 and self._se0_since is not None:
            n = (t - self._se0_since) / self.bit
            if n > 4:  # a reset (> 2.5 us), not an EOP
                self.resets.append(n)
                self.addr, self.config = 0, 0
            elif self.mon.cur is None:
                self.keepalives += 1  # an EOP with no packet in front
            if self._last_eop is not None and t - self._last_eop > 3e-3 * self.rate * self.bit and self.config:
                self.errors.append(f"bus idle for {(t - self._last_eop) / self.bit / self.rate * 1e3:.2f} ms: "
                                   "device would suspend")
            self._last_eop = t
            self._se0_since = None

    async def _send(self, pkt, after):
        while self.bus.t < after + round(self.turn * self.bit):
            await ClockCycles(self.clk, 1)
        await self.bus.send(pkt)

    async def _next_packet(self, seen):
        while len(self.mon.packets) <= seen:
            await ClockCycles(self.clk, 1)
        return self.mon.packets[seen]

    async def _run(self):
        seen = 0
        setup_data, ctl_in, toggle0, ep1_toggle = None, b"", 1, 0
        pending_addr = None
        while True:
            p = await self._next_packet(seen)
            seen += 1
            b = p["bytes"]
            if b and len(b) == 3 and b[0] == 0xA5:                 # SOF
                field = b[1] | (b[2] << 8)
                if not crc5_ok(field):
                    self.errors.append(f"SOF CRC5 wrong: {b.hex()}")
                self.sofs.append((p["start"], field & 0x7FF))
                continue
            if not b or len(b) != 3 or b[0] not in (PID_SETUP, PID_IN, PID_OUT):
                continue  # our own packets and handshakes are handled below
            field = b[1] | (b[2] << 8)
            if not crc5_ok(field):
                self.errors.append(f"token CRC5 wrong: {b.hex()}")
                continue
            addr, ep = field & 0x7F, (field >> 7) & 0xF
            if addr != self.addr:
                continue
            if b[0] == PID_SETUP:
                d = await self._next_packet(seen)
                seen += 1
                if d["bytes"][0] != PID_DATA0 or len(d["bytes"]) != 11:
                    self.errors.append(f"SETUP not followed by DATA0[8]: {d['bytes'].hex()}")
                    continue
                req = d["bytes"][1:9]
                self.requests.append(req)
                await self._send(bytes([PID_ACK]), d["eop_end"])
                seen = len(self.mon.packets)
                rtype, r, value, length = req[0], req[1], req[2] | (req[3] << 8), req[6] | (req[7] << 8)
                ctl_in, toggle0 = b"", 1
                if rtype == 0x80 and r == 6 and value >> 8 == 1:
                    ctl_in = DEV_DESC[:length]
                elif rtype == 0x80 and r == 6 and value >> 8 == 2:
                    ctl_in = CONF_DESC[:length]
                elif rtype == 0 and r == 5:
                    pending_addr = value & 0x7F
                elif rtype == 0 and r == 9:
                    self.config = value
                self.ctl_status_in = rtype & 0x80 == 0
                continue
            if b[0] == PID_IN and ep == 0:
                chunk = ctl_in[:8]
                await self._send(data_packet(PID_DATA1 if toggle0 else PID_DATA0, chunk), p["eop_end"])
                seen = len(self.mon.packets)
                h = await self._next_packet(seen)
                seen += 1
                self.host_turnarounds.append((h["start"] - self.mon.packets[seen - 2]["eop_end"]) / self.bit)
                if h["bytes"] == bytes([PID_ACK]):
                    ctl_in = ctl_in[8:] if chunk else ctl_in
                    toggle0 ^= 1
                    if not chunk and pending_addr is not None:  # status stage of SET_ADDRESS done
                        self.addr, pending_addr = pending_addr, None
                else:
                    self.errors.append(f"host answered our DATA with {h['bytes'].hex()}")
                continue
            if b[0] == PID_OUT and ep == 0:
                d = await self._next_packet(seen)
                seen += 1
                if d["bytes"] != bytes([PID_DATA1, 0, 0]):
                    self.errors.append(f"status OUT with {d['bytes'].hex()}")
                await self._send(bytes([PID_ACK]), d["eop_end"])
                seen = len(self.mon.packets)
                continue
            if b[0] == PID_IN and ep == 1 and self.config:
                if self.naks:
                    self.naks -= 1
                    await self._send(bytes([PID_NAK]), p["eop_end"])
                    seen = len(self.mon.packets)
                    continue
                rep = self.reports[0] if self.reports else bytes(8)
                await self._send(data_packet(PID_DATA1 if ep1_toggle else PID_DATA0, rep), p["eop_end"])
                seen = len(self.mon.packets)
                h = await self._next_packet(seen)
                seen += 1
                self.host_turnarounds.append((h["start"] - self.mon.packets[seen - 2]["eop_end"]) / self.bit)
                if h["bytes"] == bytes([PID_ACK]):
                    ep1_toggle ^= 1
                    if self.reports:
                        self.reports.pop(0)
                continue
