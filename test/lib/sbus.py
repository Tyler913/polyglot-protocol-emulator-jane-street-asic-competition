# Copyright (c) 2026 Tyler Hong
# SPDX-License-Identifier: Apache-2.0
"""Independent Futaba S.BUS frame model, written from the widely published
description of the protocol (Futaba does not publish a specification), not
from the RTL: 100000 baud, 8E2, inverted line; 25-byte frames: 0x0F, 16
channels x 11 bits packed LSB first, flags (bit 0 channel 17, bit 1 channel
18, bit 2 frame lost, bit 3 failsafe), 0x00. Bits are handled as an explicit
bit list here, not with an accumulator as in the C code."""

from .serial import Fmt

FMT = Fmt("8E2", invert=True)
BIT_CYCLES = 500  # 10 us at 50 MHz


def encode(channels, flags=0):
    assert len(channels) == 16 and all(0 <= c < 2048 for c in channels)
    bits = [(c >> i) & 1 for c in channels for i in range(11)]
    data = [sum(bits[8 * j + i] << i for i in range(8)) for j in range(22)]
    return [0x0F] + data + [flags & 0xFF, 0x00]


def decode(frame):
    """(channels, flags) of a 25-byte frame; raises on a bad header/footer."""
    assert len(frame) == 25 and frame[0] == 0x0F and frame[24] == 0x00, frame
    bits = [(b >> i) & 1 for b in frame[1:23] for i in range(8)]
    ch = [sum(bits[11 * k + i] << i for i in range(11)) for k in range(16)]
    return ch, frame[23]
