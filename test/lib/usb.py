# Copyright (c) 2026 Tyler Hong
# SPDX-License-Identifier: Apache-2.0
"""Independent USB low-speed (1.5 Mbit/s) and full-speed (12 Mbit/s) line model, written from the USB 2.0
specification (chapter 7/8: NRZI, bit stuffing after six ones, SYNC, PIDs,
CRC5/CRC16, SE0-based EOP), not from the RTL.

Low speed: idle J = D- high, D+ low (the device's pull-up is on D-).
K = D- low, D+ high. SE0 = both low. Full speed (full_speed=True): J and K
swap lines (pull-up on D+); everything else is the same.

UsbBus resolves the two uio pins every clock: the chip drives them when its
output enables are set, the host model when it is sending; nobody driving
leaves J (pull-up). Driving from both sides at once is an error.
UsbMonitor decodes every packet on the bus (from transitions, robust to
small rate errors), checks bit timing, SYNC, stuffing, PID check bits, CRC
and the EOP, and records packets with their start/end times.
"""

import cocotb
from cocotb.triggers import ClockCycles, FallingEdge

J, K, SE0 = "J", "K", "0"


def crc_bits(bits, width, poly, init):
    c = init
    m = (1 << width) - 1
    for b in bits:
        fb = b ^ ((c >> (width - 1)) & 1)
        c = (c << 1) & m
        if fb:
            c ^= poly
    return c


def bytes_lsb_bits(data):
    return [(x >> i) & 1 for x in data for i in range(8)]


def crc16_bytes(payload):
    """The two CRC16 bytes as sent on the wire (LSB-first bytes)."""
    c = crc_bits(bytes_lsb_bits(payload), 16, 0x8005, 0xFFFF) ^ 0xFFFF
    bits = [(c >> (15 - i)) & 1 for i in range(16)]
    return bytes(sum(bits[8 * k + i] << i for i in range(8)) for k in range(2))


def data_packet(pid, payload):
    return bytes([pid]) + bytes(payload) + crc16_bytes(payload)


def stuff(bits):
    out, run = [], 0
    for b in bits:
        out.append(b)
        run = run + 1 if b else 0
        if run == 6:
            out.append(0)
            run = 0
    return out


def line_states(packet):
    """SYNC + packet bytes -> list of J/K states (NRZI, stuffed), then EOP."""
    bits = stuff(bytes_lsb_bits(bytes([0x80]) + packet))
    states, s = [], J
    for b in bits:
        if b == 0:
            s = K if s == J else J
        states.append(s)
    return states + [SE0, SE0, J]


class UsbBus:
    def __init__(self, dut, dm, dp, bit, full_speed=False):
        self.dut, self.dm, self.dp, self.bit = dut, dm, dp, bit
        # (D-, D+) levels of J and K
        self.jk = {J: (0, 1), K: (1, 0)} if full_speed else {J: (1, 0), K: (0, 1)}
        self.host = None  # J/K/SE0 while the host drives, else None
        self.state = J
        self.t = 0
        self.errors = []
        self.listeners = []

    def start(self):
        self._task = cocotb.start_soon(self._run())

    def stop(self):
        self._task.cancel()

    async def _run(self):
        while True:
            await FallingEdge(self.dut.clk)
            self.t += 1
            out, oe = int(self.dut.uio_out.value), int(self.dut.uio_oe.value)
            c_dm, c_dp = (oe >> self.dm) & 1, (oe >> self.dp) & 1
            if c_dm != c_dp:
                self.errors.append(f"t={self.t}: chip enables only one of D+/D-")
            if c_dm:
                if self.host is not None:
                    self.errors.append(f"t={self.t}: chip and host drive the bus together")
                dm, dp = (out >> self.dm) & 1, (out >> self.dp) & 1
                s = J if (dm, dp) == self.jk[J] else K if (dm, dp) == self.jk[K] else SE0 if (dm, dp) == (0, 0) else "SE1"
                if s == "SE1":
                    self.errors.append(f"t={self.t}: chip drives SE1")
            else:
                s = self.host or J
            self.state = s
            dm, dp = self.jk.get(s, (0, 0))
            val = int(self.dut.uio_in.value) & ~((1 << self.dm) | (1 << self.dp))
            self.dut.uio_in.value = val | (dm << self.dm) | (dp << self.dp)
            for f in self.listeners:
                f(self.t, s)

    async def send(self, packet, rate_error=0.0):
        """Host sends SYNC + packet + EOP with exact (fractional) bit timing."""
        bit = self.bit / (1.0 + rate_error)
        pos, n = 0.0, 0
        for s in line_states(packet):
            self.host = s
            pos += bit
            k = round(pos) - n
            n += k
            await ClockCycles(self.dut.clk, k)
        self.host = None


class UsbMonitor:
    def __init__(self, bus, tol=3):
        self.bus, self.bit, self.tol = bus, bus.bit, tol
        self.packets = []  # dicts: start, eop_start, eop_end, bytes, errors
        self.errors = []
        self.prev = J
        self.cur = None
        bus.listeners.append(self._step)

    def _step(self, t, s):
        if s == self.prev:
            return
        if self.cur is None:
            if self.prev == J and s == K:
                self.cur = {"start": t, "edges": [(t, K)], "errors": []}
        else:
            self.cur["edges"].append((t, s))
            if self.prev == SE0 and s == J:
                self._finish(t)
        self.prev = s

    def _finish(self, t_end):
        p = self.cur
        self.cur = None
        edges = p["edges"]
        bits = []
        se0_at = None
        for (t0, s0), (t1, s1) in zip(edges, edges[1:]):
            n = round((t1 - t0) / self.bit)
            if abs((t1 - t0) - n * self.bit) > self.tol:
                p["errors"].append(f"edge at {t1} is {(t1 - t0) - n * self.bit:+.1f} cycles off the bit grid")
            if s0 == SE0:
                if not 1.5 <= (t1 - t0) / self.bit <= 2.5:
                    p["errors"].append(f"SE0 lasted {(t1 - t0) / self.bit:.2f} bit times")
                continue
            bits += [0] + [1] * (n - 1)
            if s1 == SE0:
                se0_at = t1
        # destuff
        out, run, i = [], 0, 0
        while i < len(bits):
            b = bits[i]
            out.append(b)
            run = run + 1 if b else 0
            if run == 6:
                if i + 1 < len(bits) and bits[i + 1] != 0:
                    p["errors"].append("stuff bit missing")
                i += 1
                run = 0
            i += 1
        if len(out) % 8:
            p["errors"].append(f"{len(out)} bits is not a whole number of bytes")
        data = bytes(sum(out[8 * k + j] << j for j in range(8)) for k in range(len(out) // 8))
        if not data or data[0] != 0x80:
            p["errors"].append(f"bad SYNC {data[:1].hex()}")
        data = data[1:]
        if data:
            pid = data[0]
            if (pid & 0xF) != (~pid >> 4) & 0xF:
                p["errors"].append(f"PID check bits wrong: {pid:#04x}")
            if pid & 0x3 == 0x3 and len(data) >= 3:  # DATA0/DATA1/...: CRC16 over the payload
                res = crc_bits(bytes_lsb_bits(data[1:]), 16, 0x8005, 0xFFFF)
                if res != 0x800D:
                    p["errors"].append(f"CRC16 residual {res:#06x}")
        p.update(bytes=data, eop_start=se0_at, eop_end=t_end)
        self.packets.append(p)
        self.errors += [f"packet {len(self.packets)}: {e}" for e in p["errors"]]


def token_packet(pid, addr, ep):
    """Token: PID, 7-bit address, 4-bit endpoint, CRC5 (sent MSB first)."""
    bits = [((addr | (ep << 7)) >> i) & 1 for i in range(11)]
    c = crc_bits(bits, 5, 0x05, 0x1F) ^ 0x1F
    bits += [(c >> (4 - i)) & 1 for i in range(5)]
    return bytes([pid, sum(bits[i] << i for i in range(8)), sum(bits[8 + i] << i for i in range(8))])


class UsbHost:
    """A minimal low-speed host: transactions and control transfers, with the
    device's response time checked against the 7.5 bit-time limit."""

    PID_OUT, PID_IN, PID_SETUP = 0xE1, 0x69, 0x2D
    PID_DATA0, PID_DATA1, PID_ACK = 0xC3, 0x4B, 0xD2

    def __init__(self, bus, mon):
        self.bus, self.mon, self.bit = bus, mon, bus.bit
        self.clk = bus.dut.clk
        self.turnarounds = []  # device response gaps in bit times

    transfer_gap = 0  # extra cycles before each control transfer and interrupt poll

    async def gap(self, bits):
        await ClockCycles(self.clk, max(1, round(bits * self.bit)))

    async def next_transfer(self):
        """Host software starts a new transfer: stages inside one transfer follow
        each other after the bit-time gaps above, transfers after transfer_gap."""
        if self.transfer_gap:
            await ClockCycles(self.clk, self.transfer_gap)

    async def reset(self, cycles):
        self.bus.host = SE0
        await ClockCycles(self.clk, cycles)
        self.bus.host = None
        await self.gap(60)

    async def _send(self, packet):
        await self.bus.send(packet)
        return len(self.mon.packets)  # the host packet is recorded by now

    async def _response(self, n0, timeout_bits=16):
        """The device packet after packet n0, or None if it does not start in time."""
        end = self.mon.packets[n0 - 1]["eop_end"]
        for _ in range(round(timeout_bits * self.bit)):
            if self.mon.cur is not None or len(self.mon.packets) > n0:
                break
            await ClockCycles(self.clk, 1)
        else:
            return None
        while len(self.mon.packets) <= n0:
            await ClockCycles(self.clk, 1)
        p = self.mon.packets[n0]
        self.turnarounds.append((p["start"] - end) / self.bit)
        return p

    async def setup(self, addr, data8):
        await self._send(token_packet(self.PID_SETUP, addr, 0))
        await self.gap(4)
        n = await self._send(data_packet(self.PID_DATA0, data8))
        r = await self._response(n)
        assert r is not None and r["bytes"] == bytes([self.PID_ACK]), f"SETUP not acknowledged: {r and r['bytes'].hex()}"

    async def in_(self, addr, ep, toggle):
        if ep:
            await self.next_transfer()  # interrupt poll
        n = await self._send(token_packet(self.PID_IN, addr, ep))
        r = await self._response(n)
        assert r is not None, f"no answer to IN addr {addr} ep {ep}"
        pid = r["bytes"][0]
        assert pid == (self.PID_DATA1 if toggle else self.PID_DATA0), f"IN answered with PID {pid:#04x}, toggle {toggle}"
        await self.gap(4)
        await self._send(bytes([self.PID_ACK]))
        return r["bytes"][1:-2]

    async def out_zlp(self, addr, ep):
        await self._send(token_packet(self.PID_OUT, addr, ep))
        await self.gap(4)
        n = await self._send(data_packet(self.PID_DATA1, b""))
        r = await self._response(n)
        assert r is not None and r["bytes"] == bytes([self.PID_ACK]), "status OUT not acknowledged"

    async def control_read(self, addr, setup8):
        length = setup8[6] | (setup8[7] << 8)
        await self.next_transfer()
        await self.setup(addr, setup8)
        data, toggle = b"", 1
        while len(data) < length:
            await self.gap(30)
            chunk = await self.in_(addr, 0, toggle)
            data += chunk
            toggle ^= 1
            if len(chunk) < 8:
                break
        await self.gap(30)
        await self.out_zlp(addr, 0)
        await self.gap(30)
        return data

    async def control_write(self, addr, setup8):
        await self.next_transfer()
        await self.setup(addr, setup8)
        await self.gap(30)
        zlp = await self.in_(addr, 0, 1)
        assert zlp == b"", f"status stage returned data {zlp.hex()}"
        await self.gap(30)
