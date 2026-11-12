# Copyright (c) 2026 Tyler Hong
# SPDX-License-Identifier: Apache-2.0
"""Host-side model of the SPI loader protocol (see src/pg_host_spi.v).

Pins: ui[5] = SCK, ui[6] = MOSI, ui[7] = CS_N, uo[7] = MISO. SPI mode 0, MSB first.
"""

from cocotb.triggers import ClockCycles

from .pins import ui_pins

CMD_WRITE = 0x01
CMD_READ = 0x02
CMD_CTRL = 0x03
CMD_STATUS = 0x04

SCK, MOSI, CS_N = 5, 6, 7
MISO = 7


class HostSpi:
    """Bit-bangs the host SPI port. half_period is in system clock cycles (>= 4)."""

    def __init__(self, dut, half_period=4):
        self.dut = dut
        self.half = half_period
        self.ui = ui_pins(dut)
        self._set(CS_N, 1)

    def _set(self, bit, v):
        self.ui.set(bit, v)

    async def _xfer(self, data):
        """Clock bytes out on MOSI; return the bytes seen on MISO."""
        clk = self.dut.clk
        self._set(CS_N, 0)
        await ClockCycles(clk, self.half)
        rx = []
        for byte in data:
            r = 0
            for i in range(7, -1, -1):
                self._set(MOSI, (byte >> i) & 1)
                await ClockCycles(clk, self.half)
                self._set(SCK, 1)
                r = (r << 1) | ((int(self.dut.uo_out.value) >> MISO) & 1)
                await ClockCycles(clk, self.half)
                self._set(SCK, 0)
            rx.append(r)
        await ClockCycles(clk, self.half)
        self._set(CS_N, 1)
        await ClockCycles(clk, 2 * self.half)
        return rx

    async def write(self, addr, data):
        await self._xfer([CMD_WRITE, addr >> 8, addr & 0xFF, *data])

    async def read(self, addr, n):
        rx = await self._xfer([CMD_READ, addr >> 8, addr & 0xFF, 0] + [0] * n)
        return bytes(rx[4:])

    async def ctrl(self, value):
        await self._xfer([CMD_CTRL, value])

    async def status(self):
        return (await self._xfer([CMD_STATUS, 0]))[1]
