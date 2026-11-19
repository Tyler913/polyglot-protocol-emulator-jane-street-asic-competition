# Copyright (c) 2026 Tyler Hong
# SPDX-License-Identifier: Apache-2.0
"""Independent CRSF (TBS Crossfire / ExpressLRS serial protocol) model,
written from the published CRSF frame description, not from the RTL:
[sync 0xC8][len][type][payload][crc], len = type + payload + crc bytes,
CRC-8 polynomial 0xD5 (DVB-S2), initial value 0, over type and payload.
RC_CHANNELS_PACKED (0x16): 16 channels x 11 bits, LSB first.
LINK_STATISTICS (0x14): 10 bytes, uplink RSSI 1 first, LQ third.
BATTERY_SENSOR (0x08): voltage and current (0.1 units, big endian, 16 bit),
capacity used (24 bit, mAh), remaining (%)."""

from .serial import Fmt

FMT = Fmt("8N1")
BIT_CYCLES = 50e6 / 420000


def crc8(data):
    c = 0
    for b in data:
        for i in range(7, -1, -1):          # bit by bit, MSB first
            fb = ((c >> 7) & 1) ^ ((b >> i) & 1)
            c = (c << 1) & 0xFF
            if fb:
                c ^= 0xD5
    return c


def frame(ftype, payload, bad_crc=False):
    body = [ftype] + list(payload)
    return [0xC8, len(body) + 1] + body + [crc8(body) ^ (0x01 if bad_crc else 0)]


def rc_channels(ch):
    bits = [(c >> i) & 1 for c in ch for i in range(11)]
    return frame(0x16, [sum(bits[8 * j + i] << i for i in range(8)) for j in range(22)])


def link_stats(rssi1, lq):
    return frame(0x14, [rssi1, 0, lq, 10, 0, 2, 3, 80, 90, 7])


def parse(stream):
    """Frames (type, payload) from a byte stream; raises on a bad frame."""
    out, i = [], 0
    while i < len(stream):
        assert stream[i] == 0xC8, f"sync {stream[i]:#x} at {i}"
        n = stream[i + 1]
        body = stream[i + 2: i + 1 + n]
        assert crc8(body) == stream[i + 1 + n], f"CRC of frame at {i}"
        out.append((body[0], body[1:]))
        i += 2 + n
    return out
