# Copyright (c) 2026 Tyler Hong
# SPDX-License-Identifier: Apache-2.0
"""Host driver for the Polyglot chip, for MicroPython on the Tiny Tapeout demo
board's RP2350 (also runs under CPython with any pin objects).

The chip's host port is an SPI target (mode 0, MSB first, SCK <= clk/8) on
ui[5] = SCK, ui[6] = MOSI, ui[7] = CS_N and uo[7] = MISO; HOST_IRQ is uo[6].
Commands (see src/pg_host_spi.v and docs/info.md):
    0x01 WRITE  addr_hi addr_lo data...
    0x02 READ   addr_hi addr_lo dummy -> data...
    0x03 CTRL   value          (bit 0 = RUN)
    0x04 STATUS xx             (second byte returns status)

Usage on the demo board (pin access through the Tiny Tapeout SDK; check the
names against the SDK version you have):

    from ttboard.demoboard import DemoBoard
    from polyglot_host import Polyglot, TTPins
    tt = DemoBoard.get()
    tt.shuttle.tt_um_tyler913_polyglot.enable()
    tt.clock_project_PWM(50_000_000)
    chip = Polyglot(TTPins(tt))
    chip.run_program(open("usb_hid.bin", "rb").read())

Status: mirrors the cocotb host model (test/lib/host.py) that all tests use;
not yet run on silicon.
"""

SHARED = 0x0F00  # PG_SHARED: last 256 bytes of SRAM


class TTPins:
    """Pin access through the Tiny Tapeout demo board SDK (ui_in / uo_out)."""

    def __init__(self, tt):
        self.tt = tt

    def set_sck(self, v):
        self.tt.ui_in[5] = v

    def set_mosi(self, v):
        self.tt.ui_in[6] = v

    def set_cs(self, v):
        self.tt.ui_in[7] = v

    def miso(self):
        return int(self.tt.uo_out[7])

    def irq(self):
        return int(self.tt.uo_out[6])


class Polyglot:
    def __init__(self, pins, delay=None):
        self.p = pins
        self.delay = delay or (lambda: None)  # optional half-period delay
        self.p.set_cs(1)
        self.p.set_sck(0)

    def _xfer(self, data):
        p, d = self.p, self.delay
        p.set_cs(0)
        d()
        out = bytearray(len(data))
        for k, byte in enumerate(data):
            r = 0
            for i in range(7, -1, -1):
                p.set_mosi((byte >> i) & 1)
                d()
                p.set_sck(1)
                r = (r << 1) | p.miso()
                d()
                p.set_sck(0)
            out[k] = r
        d()
        p.set_cs(1)
        d()
        return out

    def write(self, addr, data):
        self._xfer(bytes([0x01, addr >> 8, addr & 0xFF]) + bytes(data))

    def read(self, addr, n):
        return bytes(self._xfer(bytes([0x02, addr >> 8, addr & 0xFF, 0]) + bytes(n))[4:])

    def ctrl(self, value):
        self._xfer(bytes([0x03, value]))

    def status(self):
        return self._xfer(bytes([0x04, 0]))[1]

    def halt(self):
        self.ctrl(0)

    def run(self):
        self.ctrl(1)

    def load(self, image, params=()):
        """Halt, write the program at 0 and the parameter words at PG_SHARED."""
        self.halt()
        self.write(0, image)
        words = list(params) + [0] * (64 - len(params))
        self.write(SHARED, b"".join(int(w & 0xFFFFFFFF).to_bytes(4, "little") for w in words))
        if self.read(0, len(image)) != bytes(image):
            raise RuntimeError("program read-back mismatch")

    def run_program(self, image, params=()):
        self.load(image, params)
        self.run()

    def shared(self, index, count=1):
        """PG_SHARED words (reliable while the firmware is parked or blocked)."""
        b = self.read(SHARED + 4 * index, 4 * count)
        return [int.from_bytes(b[4 * i : 4 * i + 4], "little") for i in range(count)]

    def irq(self):
        return self.p.irq()
