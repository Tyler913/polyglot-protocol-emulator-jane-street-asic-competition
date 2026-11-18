# Copyright (c) 2026 Tyler Hong
# SPDX-License-Identifier: Apache-2.0
"""What an IR receiver module outputs for NEC and RC-5 remote frames (low
while a burst is received, high otherwise), written from the NEC format
(9 ms + 4.5 ms leader, 562.5 us bursts, 562.5 / 1687.5 us spaces, address,
~address, command, ~command LSB first; repeat = 9 ms + 2.25 ms + burst) and
the Philips RC-5 format (889 us half bits, 2 start bits, toggle, 5 address
bits, 6 command bits MSB first; a 1 is space then burst). Durations can be
scaled to emulate a remote whose clock is off."""

from cocotb.triggers import Timer

from .pins import ui_pins


class IrDriver:
    def __init__(self, dut, ui_bit, scale=1.0):
        self.pins, self.bit, self.scale = ui_pins(dut), ui_bit, scale
        self.pins.set(ui_bit, 1)

    async def _play(self, seq):
        for level, us in seq:
            self.pins.set(self.bit, level)
            await Timer(round(us * self.scale * 1000), unit="ns")
        self.pins.set(self.bit, 1)

    async def nec(self, addr, cmd):
        seq = [(0, 9000), (1, 4500)]
        v = addr | ((addr ^ 0xFF) << 8) | (cmd << 16) | ((cmd ^ 0xFF) << 24)
        for i in range(32):
            seq += [(0, 562.5), (1, 1687.5 if (v >> i) & 1 else 562.5)]
        seq += [(0, 562.5)]
        await self._play(seq)

    async def nec_repeat(self):
        await self._play([(0, 9000), (1, 2250), (0, 562.5)])

    async def rc5(self, toggle, addr, cmd):
        bits = [1, 1, toggle] + [(addr >> (4 - i)) & 1 for i in range(5)] + [(cmd >> (5 - i)) & 1 for i in range(6)]
        halves = []
        for b in bits:
            halves += [1, 0] if b else [0, 1]
        seq = []
        for h in halves:  # merge equal neighbours into one duration
            if seq and seq[-1][0] == h:
                seq[-1] = (h, seq[-1][1] + 889)
            else:
                seq.append((h, 889))
        if seq[0][0] == 1:
            seq = seq[1:]  # the leading idle-level half is just idle
        await self._play(seq)
