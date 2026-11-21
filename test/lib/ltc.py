# Copyright (c) 2026 Tyler Hong
# SPDX-License-Identifier: Apache-2.0
"""Independent SMPTE 12M linear timecode (LTC) model, written from the
standard's public description, not from our RTL.

A frame is 80 bits, biphase mark coded (a transition at every bit start, one
more in the middle of a 1), sent at 80 bits per video frame. Bits 0..63 hold
BCD time fields, LSB first (frame units 0-3, frame tens 8-9, drop frame 10,
seconds units 16-19, tens 24-26, minutes units 32-35, tens 40-42, hours units
48-51, tens 56-57) with user bits and flags in between; bit 27 (30 fps) is the
polarity correction bit that makes the number of zeros in the frame even, so
every frame starts with the same line level. Bits 64..79 are the sync word
0011 1111 1111 1101.
"""

SYNC = [0, 0, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 0, 1]


def frame_bits(h, m, s, f, user=0):
    b = [0] * 64

    def put(pos, n, v):
        for i in range(n):
            b[pos + i] = (v >> i) & 1
    put(0, 4, f % 10)
    put(8, 2, f // 10)
    put(16, 4, s % 10)
    put(24, 3, s // 10)
    put(32, 4, m % 10)
    put(40, 3, m // 10)
    put(48, 4, h % 10)
    put(56, 2, h // 10)
    for k, pos in enumerate((4, 12, 20, 28, 36, 44, 52, 60)):
        if pos == 28:
            continue
        put(pos, 4, (user >> (4 * k)) & 15)
    bits = b + SYNC
    if bits.count(0) % 2:
        bits[27] = 1
    return bits


def fields(bits):
    def get(pos, n):
        return sum(bits[pos + i] << i for i in range(n))
    return (get(56, 2) * 10 + get(48, 4), get(40, 3) * 10 + get(32, 4),
            get(24, 3) * 10 + get(16, 4), get(8, 2) * 10 + get(0, 4))


def bmc_halves(bits, lvl=0):
    out = []
    for x in bits:
        lvl ^= 1
        out.append(lvl)
        if x:
            lvl ^= 1
        out.append(lvl)
    return out
