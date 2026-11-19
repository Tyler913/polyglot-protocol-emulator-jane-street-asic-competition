# Copyright (c) 2026 Tyler Hong
# SPDX-License-Identifier: Apache-2.0
"""Independent DHT22 / AM2302 sensor, written from the AOSONG AM2302
datasheet, not from the RTL: open-drain line with a pull-up. The host's
start signal is a low of at least 800 us (typically 1 ms); 30 us after the
host releases the line the sensor answers: low 80 us, high 80 us, 40 bits
(each low 50 us, then high 27 us for 0 or 70 us for 1, MSB first), then low
50 us and release. Data: humidity x 10 (16 bit), temperature x 10 (16 bit,
bit 15 = minus), checksum = sum of the four bytes."""

import cocotb
from cocotb.triggers import ClockCycles, ReadOnly, RisingEdge

US = 50


def encode(hum10, temp10, bad_sum=False):
    t = (abs(temp10) & 0x7FFF) | (0x8000 if temp10 < 0 else 0)
    b = [hum10 >> 8, hum10 & 0xFF, t >> 8, t & 0xFF]
    s = (sum(b) + (1 if bad_sum else 0)) & 0xFF
    return [(x >> (7 - i)) & 1 for x in b + [s] for i in range(8)]


class Dht22:
    def __init__(self, dut, pin):
        self.dut, self.pin = dut, pin
        self.readings = []       # (hum10, temp10, bad_sum) per start signal
        self.low = 0             # sensor pulls the line low
        self.starts = []         # host start-signal lengths (cycles)
        self.errors = []

    def start(self):
        self._task = cocotb.start_soon(self._run())

    def stop(self):
        self._task.cancel()

    async def _line(self):
        await RisingEdge(self.dut.clk)
        await ReadOnly()
        oe = (int(self.dut.uio_oe.value) >> self.pin) & 1
        out = (int(self.dut.uio_out.value) >> self.pin) & 1
        host_low = oe and not out
        await RisingEdge(self.dut.clk)
        line = 0 if (host_low or self.low) else 1
        v = int(self.dut.uio_in.value) & ~(1 << self.pin) & 0xFF
        self.dut.uio_in.value = v | (line << self.pin)
        return host_low

    async def _run(self):
        host_low_since = None
        while True:
            host_low = await self._line()
            if host_low and host_low_since is None:
                host_low_since = 0
            if host_low:
                host_low_since += 2
                continue
            if host_low_since is None:
                continue
            length, host_low_since = host_low_since, None
            self.starts.append(length)
            if length < 800 * US:
                self.errors.append(f"start signal {length / US:.0f} us (needs >= 800)")
                continue
            if not self.readings:
                continue
            hum, temp, bad = self.readings.pop(0)
            await self._hold(0, 30)                 # line released: pulled up
            await self._hold(1, 80)                 # response: low 80 us
            await self._hold(0, 80)                 # high 80 us
            for b in encode(hum, temp, bad):
                await self._hold(1, 50)             # bit: low 50 us
                await self._hold(0, 70 if b else 27)  # high: 1 = 70 us, 0 = 27 us
            await self._hold(1, 50)                 # end: low 50 us, then release
            self.low = 0

    async def _hold(self, low, us):  # low = 1: the sensor pulls the line low
        self.low = low
        for _ in range(round(us * US / 2)):
            await self._line()
