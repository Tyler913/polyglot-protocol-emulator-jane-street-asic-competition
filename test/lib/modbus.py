# Copyright (c) 2026 Tyler Hong
# SPDX-License-Identifier: Apache-2.0
"""Independent Modbus RTU server, written from "MODBUS over Serial Line
V1.02" (RTU framing: silent interval of 3.5 characters before a frame, at
most 1.5 characters between bytes, fixed 1.75 ms / 0.75 ms above 19200 baud;
CRC-16 polynomial 0xA001 reflected, init 0xFFFF, low byte first) and the
application protocol V1.1b3 (functions 0x03 and 0x06, exception 0x02),
not from the RTL. It listens on the chip's TX with a UartMonitor and
answers through a UartDriver."""

import cocotb
from cocotb.triggers import ClockCycles
from cocotb.utils import get_sim_time

from .uart import UartDriver, UartMonitor


def crc16(data):
    c = 0xFFFF
    for b in data:
        c ^= b
        for _ in range(8):
            c = (c >> 1) ^ 0xA001 if c & 1 else c >> 1
    return c


def with_crc(f):
    c = crc16(f)
    return bytes(f) + bytes([c & 0xFF, c >> 8])


class ModbusServer:
    def __init__(self, dut, tx_uo, rx_ui, baud, address, regs, f_clk=50_000_000):
        self.dut, self.addr, self.regs = dut, address, dict(regs)
        self.bit = f_clk / baud
        self.char = 10 * self.bit  # 8N1 (the spec's 11-bit character assumes parity or 2 stop bits)
        self.t35 = 1.75e-3 * f_clk if baud > 19200 else 3.5 * self.char
        self.t15 = 0.75e-3 * f_clk if baud > 19200 else 1.5 * self.char
        self.mon = UartMonitor(dut, lambda: (int(dut.uo_out.value) >> tx_uo) & 1, self.bit)
        self.drv = UartDriver(dut, rx_ui, self.bit)
        self.requests, self.errors = [], []
        self.last_end = -10**9  # cycle where the last reply ended
        self.clk_ns = 1e9 / f_clk

    def start(self):
        self.t0 = get_sim_time("ns")
        self.mon.start()
        self._task = cocotb.start_soon(self._run())

    def stop(self):
        self.mon.stop()
        self._task.cancel()

    def _now(self):
        """Cycles since start(), on the same base as the monitor's byte times."""
        return round((get_sim_time("ns") - self.t0) / self.clk_ns)

    async def _run(self):
        seen = 0
        step = int(self.bit)
        while True:
            # collect one request: bytes until a silence longer than t3.5
            while len(self.mon.bytes) == seen:
                await ClockCycles(self.dut.clk, step)
            first = seen
            quiet = 0
            while quiet < self.t35 + self.char:
                await ClockCycles(self.dut.clk, step)
                if len(self.mon.bytes) > seen:
                    seen, quiet = len(self.mon.bytes), 0
                else:
                    quiet += step
            frame = bytes(self.mon.bytes[first:seen])
            times = self.mon.times[first:seen]
            if times[0] - self.last_end < self.t35 - 2:
                self.errors.append(f"only {(times[0] - self.last_end) / self.char:.2f} characters of silence before a request")
            for a, b in zip(times, times[1:]):
                if b - a - self.char > self.t15:
                    self.errors.append(f"gap of {(b - a - self.char) / self.char:.2f} characters inside a frame")
            self.requests.append(frame)
            if len(frame) < 4 or crc16(frame[:-2]) != frame[-2] | (frame[-1] << 8):
                self.errors.append(f"CRC of {frame.hex()}")
                continue
            if frame[0] != self.addr:
                continue
            reply = with_crc(self._serve(frame[:-2]))
            await self.drv.send(reply)
            self.last_end = self._now()

    def _serve(self, req):
        a, fn = req[0], req[1]
        if fn == 0x06:
            reg, val = (req[2] << 8) | req[3], (req[4] << 8) | req[5]
            self.regs[reg] = val
            return bytes(req)
        if fn == 0x03:
            reg, cnt = (req[2] << 8) | req[3], (req[4] << 8) | req[5]
            if any(r not in self.regs for r in range(reg, reg + cnt)):
                return bytes([a, 0x83, 0x02])
            out = bytes([a, 0x03, 2 * cnt])
            for r in range(reg, reg + cnt):
                out += bytes([self.regs[r] >> 8, self.regs[r] & 0xFF])
            return out
        return bytes([a, fn | 0x80, 0x01])


class ModbusClient:
    """Independent Modbus RTU client for testing a server: sends request
    frames (8E1 by default, the specification's default) and collects the
    reply, which ends after 3.5 characters of silence. Written from the same
    specification, on the generic serial line model."""

    def __init__(self, dut, ui, rx_ui, get_tx, baud, fmt, f_clk=50_000_000):
        from .serial import SerialRx, SerialTx
        self.dut = dut
        self.bit = f_clk / baud
        self.char = fmt.bits * self.bit
        self.tx = SerialTx(dut, ui, rx_ui, self.bit, fmt)
        self.rx = SerialRx(dut, get_tx, self.bit, fmt)
        self.rx.start()

    async def request(self, frame, crc=True, gap_after=None, bad_parity=(), wait_chars=12):
        """Send frame (+ CRC unless crc=False). gap_after=(i, chars): idle
        for that long after byte i. Returns the reply bytes (CRC checked and
        removed) or None if none came within wait_chars after the request."""
        data = with_crc(frame) if crc else bytes(frame)
        n0 = len(self.rx.frames)
        if gap_after:
            i, chars = gap_after
            await self.tx.send(data[:i + 1], bad_parity=bad_parity)
            await ClockCycles(self.dut.clk, round(chars * self.char))
            await self.tx.send(data[i + 1:])
        else:
            await self.tx.send(data, bad_parity=bad_parity)
        # wait for a reply to start, then for 3.5 characters of silence
        t_end = wait_chars * self.char
        waited = 0
        while len(self.rx.frames) == n0 and waited < t_end:
            await ClockCycles(self.dut.clk, 1000)
            waited += 1000
        if len(self.rx.frames) == n0:
            return None
        while True:
            n = len(self.rx.frames)
            await ClockCycles(self.dut.clk, round(3.5 * self.char))
            if len(self.rx.frames) == n:
                break
        reply = bytes(v for v, _ in self.rx.frames[n0:] if v is not None)
        assert len(reply) >= 4 and crc16(reply[:-2]) == reply[-2] | (reply[-1] << 8), f"bad reply {reply.hex()}"
        return reply[:-2]
