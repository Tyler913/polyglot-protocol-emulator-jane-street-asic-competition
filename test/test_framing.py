# Copyright (c) 2026 Tyler Hong
# SPDX-License-Identifier: Apache-2.0
"""Synchronous framings from engine settings, each decoded from the pins by a
model written from its standard:

* HDLC (sw/examples/hdlc, ISO/IEC 13239): flags, zero deletion after five
  ones, FCS-16 checked with the reflected CRC-16/X.25 (poly 0x8408), sampled
  on the rising edge of the transmit clock.
* I2S (sw/examples/i2s, Philips I2S bus specification): WS and SD sampled on
  rising BCLK, the MSB one clock after each WS change, 48 kHz frame rate.
* MIDI (sw/examples/midi, MIDI 1.0): 31250 baud, running status.
* I2S receive (sw/examples/i2s_rx): an I2S transmitter model drives BCLK, WS
  and SD (MSB one clock after each WS change); the chip must recover every
  left/right sample."""

import cocotb
from cocotb.triggers import ClockCycles, Timer

from cocotb.utils import get_sim_time

from lib.pins import bit, ui_pins
from lib.soc import load, read_shared, start, wait_irq
from lib.uart import UartMonitor, bit_cycles


class ClockedSampler:
    """Samples data bits of uo_out on rising edges of a clock bit of uo_out."""

    def __init__(self, dut, clk, data):
        self.dut, self.clk, self.data = dut, clk, data
        self.samples, self.times = [], []

    def start(self):
        self._task = cocotb.start_soon(self._run())

    def stop(self):
        self._task.cancel()

    async def _run(self):
        last = 0
        while True:
            await self.dut.uo_out.value_change
            try:
                c = bit(self.dut.uo_out, self.clk)
            except ValueError:
                continue
            if c and not last:
                self.samples.append(tuple(bit(self.dut.uo_out, d) for d in self.data))
                self.times.append(get_sim_time("ns"))
            last = c


def fcs16(data):
    c = 0xFFFF
    for b in data:
        c ^= b
        for _ in range(8):
            c = (c >> 1) ^ 0x8408 if c & 1 else c >> 1
    return c ^ 0xFFFF


def hdlc_decode(bits, errors):
    frames, cur, ones, in_frame = [], None, 0, False
    i = 0
    s = "".join(map(str, bits))
    while i < len(bits):
        if s.startswith("01111110", i):
            if cur is not None and len(cur) >= 16:
                if len(cur) % 8:
                    errors.append(f"frame of {len(cur)} bits")
                frames.append(bytes(sum(cur[8 * k + j] << j for j in range(8)) for k in range(len(cur) // 8)))
            cur, ones = [], 0
            i += 8
            continue
        b = bits[i]
        if cur is not None:
            if ones == 5:
                if b != 0:
                    errors.append(f"six ones inside a frame at bit {i}")
                ones = 0  # deleted zero
            else:
                cur.append(b)
                ones = ones + 1 if b else 0
        i += 1
    out = []
    for f in frames:
        if fcs16(f[:-2]) != f[-2] | (f[-1] << 8):
            errors.append(f"FCS of {f.hex()}")
        out.append(f[:-2])
    return out


@cocotb.test()
async def test_hdlc(dut):
    host = await start(dut)
    smp = ClockedSampler(dut, clk=1, data=[0])
    smp.start()
    await load(host, "hdlc")
    await host.ctrl(0x01)
    await wait_irq(dut, 1, timeout=200_000)
    smp.stop()
    errors = []
    frames = hdlc_decode([s[0] for s in smp.samples], errors)
    assert not errors, errors
    assert frames == [b"\x03\x00Polyglot", bytes([0xFF, 0x13, 0x7E, 0x7D, 0xFF, 0xFF, 0x3F, 0x1F, 0x00]),
                      b"\x01\x11"], frames
    assert (await read_shared(host, 0, 1))[0] == 0x600D4D1C


@cocotb.test()
async def test_i2s(dut):
    host = await start(dut)
    smp = ClockedSampler(dut, clk=3, data=[4, 2])  # (WS, SD) at rising BCLK
    smp.start()
    n = 12
    await load(host, "i2s", [n])
    await host.ctrl(0x01)
    await wait_irq(dut, 1, timeout=200_000)
    smp.stop()
    ws = [1] + [s[0] for s in smp.samples]  # WS idles high before BCLK starts
    sd = [0] + [s[1] for s in smp.samples]
    left, right = [], []
    for i in range(1, len(ws)):
        if ws[i] != ws[i - 1]:
            word = sd[i + 1 : i + 17]
            if len(word) < 16:
                break
            v = sum(b << (15 - k) for k, b in enumerate(word))
            v = v - 65536 if v >= 32768 else v
            (right if ws[i] else left).append(v)
    want_l = [1000 * k - 8000 for k in range(n)]
    want_r = [(0x5A5A ^ k) for k in range(n)]
    assert left == want_l, left
    assert right == want_r, right
    # 48 kHz: 32 BCLK periods per frame
    period = (smp.times[-1] - smp.times[1]) / (len(smp.times) - 2)
    assert abs(1e9 / (32 * period) - 48000) < 30, f"frame rate {1e9 / (32 * period):.1f} Hz"
    assert (await read_shared(host, 1, 1))[0] == 0x600D0125


@cocotb.test()
async def test_midi(dut):
    host = await start(dut)
    mon = UartMonitor(dut, lambda: (int(dut.uo_out.value) >> 1) & 1, bit_cycles(31250))
    mon.start()
    await load(host, "midi")
    await host.ctrl(0x01)
    await wait_irq(dut, 1, timeout=1_000_000)
    mon.stop()
    assert not mon.errors, mon.errors
    msgs, status, data = [], None, []
    for b in mon.bytes:  # MIDI 1.0 parser with running status
        if b & 0x80:
            status, data = b, []
            continue
        data.append(b)
        need = 1 if status & 0xF0 in (0xC0, 0xD0) else 2
        if len(data) == need:
            msgs.append((status, *data))
            data = []
    notes = [m for m in msgs if m[0] == 0x90]
    assert msgs[0] == (0xC0, 5), msgs
    assert [(n[1], n[2]) for n in notes] == [(60, 100), (60, 0), (64, 100), (64, 0), (67, 100), (67, 0), (72, 100), (72, 0)]
    assert msgs[-1] == (0xB0, 123, 0)
    assert mon.bytes.count(0x90) == 1, "running status not used"
    assert (await read_shared(host, 0, 1))[0] == 0x600D0001


async def i2s_source(dut, frames, bit_ns):
    """Philips I2S transmitter: data changes on falling BCLK, WS one clock before the MSB."""
    from lib.pins import ui_pins

    p = ui_pins(dut)
    half = bit_ns / 2
    words = []
    for left, right in frames:
        words += [(0, left), (1, right)]
    stream = []  # (ws, sd) per bit, WS leading the data by one bit
    bits = []
    for ch, v in words:
        bits += [(ch, (v >> (15 - i)) & 1) for i in range(16)]
    for i in range(len(bits)):
        ws_next = bits[i + 1][0] if i + 1 < len(bits) else bits[i][0]
        stream.append((ws_next, bits[i][1]))
    for i in range(40):  # idle clocks (right channel, zeros); WS falls one clock before the first MSB
        p.set(2, 0)
        p.set(3, 0 if i == 39 else 1)
        p.set(1, 0)
        await Timer(round(half), unit="ns")
        p.set(2, 1)
        await Timer(round(half), unit="ns")
    for ws, sd in stream:
        p.set(2, 0)  # falling edge: change WS and SD
        p.set(3, ws)
        p.set(1, sd)
        await Timer(round(half), unit="ns")
        p.set(2, 1)
        await Timer(round(half), unit="ns")


@cocotb.test()
async def test_i2s_rx(dut):
    host = await start(dut)
    n = 12
    frames = [((300 * k + 7) & 0xFFFF, (0xA5C3 ^ (k * 0x111)) & 0xFFFF) for k in range(n + 6)]
    await load(host, "i2s_rx", [n])
    await host.ctrl(0x01)
    cocotb.start_soon(i2s_source(dut, frames, 1e9 / (48000 * 32)))
    await wait_irq(dut, 1, timeout=400_000)
    got = await read_shared(host, 16, n)
    pairs = [(g & 0xFFFF, g >> 16) for g in got]
    k0 = frames.index(pairs[0]) if pairs[0] in frames else None
    assert k0 is not None, f"first frame {pairs[0]} not sent"
    assert pairs == frames[k0 : k0 + n], (pairs, frames[k0 : k0 + n])


def hdlc_encode(payload, bad_fcs=False):
    """Bits (first on the line first) of one frame between flags: data and
    FCS LSB first, a 0 inserted after every five consecutive ones."""
    c = fcs16(payload) ^ (0x0001 if bad_fcs else 0)
    data = bytes(payload) + bytes([c & 0xFF, c >> 8])
    raw = [(b >> j) & 1 for b in data for j in range(8)]
    out, ones = [], 0
    for b in raw:
        out.append(b)
        ones = ones + 1 if b else 0
        if ones == 5:
            out.append(0)
            ones = 0
    return out


FLAG = [0, 1, 1, 1, 1, 1, 1, 0]


@cocotb.test()
async def test_hdlc_rx(dut):
    """HDLC receive (sw/examples/hdlc_rx): the engine collects raw bits on the
    line clock, C destuffs, finds flags and aborts and checks the FCS. Stream
    from an independent encoder: good frames (one full of stuffed bits), a
    bad FCS, an aborted frame, shared flags between frames."""
    host = await start(dut)
    ui = ui_pins(dut)
    good = [b"\x03\x00Polyglot", bytes([0xFF, 0x13, 0x7E, 0x7D, 0xFF, 0xFF, 0x3F, 0x1F, 0x00]), b"\x01\x11"]
    bits = FLAG * 3 + hdlc_encode(good[0]) + FLAG + hdlc_encode(good[1]) + FLAG
    bits += hdlc_encode(b"\x05\x00bad", bad_fcs=True) + FLAG
    bits += hdlc_encode(b"\x07\x00aborted")[:30] + [1] * 9 + FLAG * 2      # abort, then idle flags
    bits += hdlc_encode(good[2]) + FLAG * 3
    await load(host, "hdlc_rx", [3])
    ui.set(1, 0)
    await host.ctrl(0x01)
    await wait_irq(dut, 1, timeout=10_000)
    half = 25                                         # 1 Mbit/s
    for b in bits + FLAG * 8:
        ui.set(0, b)                                  # data changes while RXC is low
        await ClockCycles(dut.clk, half)
        ui.set(1, 1)                                  # rising edge: the receiver samples
        await ClockCycles(dut.clk, half)
        ui.set(1, 0)
    await wait_irq(dut, 0, timeout=10_000)
    res = await read_shared(host, 0, 8 + 12 * 3)
    assert res[2] == 0x600D0013, hex(res[2])
    assert (res[3], res[4]) == (1, 1), f"aborts {res[3]}, bad FCS {res[4]}"
    for k, f in enumerate(good):
        n = res[8 + 12 * k]
        got = b"".join(w.to_bytes(4, "little") for w in res[9 + 12 * k: 20 + 12 * k])[:n]
        assert got == f, f"frame {k}: {got!r}"
