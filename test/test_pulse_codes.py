# Copyright (c) 2026 Tyler Hong
# SPDX-License-Identifier: Apache-2.0
"""Pulse-width protocols from plain NRZ engine streams, each judged from the
recorded edges by a decoder written from its specification:

* DShot600 (sw/examples/dshot): 1.667 us bits, 75 % / 37.5 % duty, 16-bit
  frames with the XOR-nibble checksum (Betaflight DShot description).
* SAE J2716 SENT (sw/examples/sent): 3 us ticks, falling-edge-to-falling-edge
  pulse lengths, low time >= 4 ticks, 56-tick sync, status + 6 data nibbles,
  CRC-4 by the J2716 lookup table with the extra zero nibble."""

import cocotb

from lib.edges import EdgeRecorder
from lib.soc import load, read_shared, start, wait_irq


def dshot_decode(rec, errors):
    bit_ns = 1e9 / 600_000
    highs = rec.pulses(1)
    frames, cur, last = [], [], None
    for t, w in highs:
        if last is not None and t - last > 2 * bit_ns:
            frames.append(cur)
            cur = []
        if last is not None and t - last < 2 * bit_ns and abs((t - last) - bit_ns) > 30:
            errors.append(f"bit period {t - last:.0f} ns")
        duty = w / bit_ns
        if abs(duty - 0.75) < 0.05:
            cur.append(1)
        elif abs(duty - 0.375) < 0.05:
            cur.append(0)
        else:
            errors.append(f"duty {duty:.3f}")
        last = t
    frames.append(cur)
    out = []
    for f in frames:
        if len(f) != 16:
            errors.append(f"frame of {len(f)} bits")
            continue
        v = sum(b << (15 - i) for i, b in enumerate(f))
        d = v >> 4
        if (d ^ (d >> 4) ^ (d >> 8)) & 0xF != v & 0xF:
            errors.append(f"checksum of {v:#06x}")
        out.append((d >> 1, d & 1))
    return out


@cocotb.test()
async def test_dshot600(dut):
    host = await start(dut)
    rec = EdgeRecorder(dut, "uo_out", 5)
    rec.start()
    await load(host, "dshot")
    await host.ctrl(0x01)
    await wait_irq(dut, 1, timeout=100_000)
    rec.stop()
    errors = []
    frames = dshot_decode(rec, errors)
    assert not errors, errors[:5]
    assert frames == [(0, 0), (48, 0), (1046, 1), (2047, 0)], frames
    assert (await read_shared(host, 0, 1))[0] == 0x600DD540


CRC4_TABLE = [0, 13, 7, 10, 14, 3, 9, 4, 1, 12, 6, 11, 15, 2, 8, 5]  # SAE J2716 (2010) appendix


def sent_decode(rec, errors, tick_ns=3000):
    falls = [t for t, v in rec.edges if v == 0]
    lows = {t: w for t, w in rec.pulses(0)}
    msgs, cur = [], None
    for a, b in zip(falls, falls[1:]):
        ticks = (b - a) / tick_ns
        if lows.get(a, 0) < 4 * tick_ns - 25:
            errors.append(f"low time {lows.get(a, 0):.0f} ns < 4 ticks")
        n = round(ticks)
        if abs(ticks - n) > 0.02 * n:
            errors.append(f"pulse of {ticks:.2f} ticks")
        if n == 56:
            cur = []
            msgs.append(cur)
        elif cur is not None and len(cur) < 8:
            if not 12 <= n <= 27:
                errors.append(f"nibble pulse of {n} ticks")
            cur.append(n - 12)
    out = []
    for m in msgs:
        if len(m) != 8:
            errors.append(f"message of {len(m)} nibbles")
            continue
        crc = 5
        for x in m[1:7]:
            crc = x ^ CRC4_TABLE[crc]
        crc = CRC4_TABLE[crc]
        if crc != m[7]:
            errors.append(f"CRC {m[7]} != {crc}")
        a = (m[1] << 8) | (m[2] << 4) | m[3]
        b = m[4] | (m[5] << 4) | (m[6] << 8)
        out.append((m[0], a, b))
    return out


@cocotb.test()
async def test_sent(dut):
    host = await start(dut)
    rec = EdgeRecorder(dut, "uo_out", 0)
    rec.start()
    await load(host, "sent", [0x5A3, 0x0F1])
    await host.ctrl(0x01)
    await wait_irq(dut, 1, timeout=1_000_000)
    rec.stop()
    errors = []
    msgs = sent_decode(rec, errors)
    assert not errors, errors[:5]
    assert msgs == [(0, 0x5A3, 0x0F1), (1, 0x5A4, 0x0F0), (2, 0x5A5, 0x0EF)], msgs
    assert (await read_shared(host, 2, 1))[0] == 0x600D5E47
