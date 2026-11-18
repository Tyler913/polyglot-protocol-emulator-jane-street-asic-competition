# Copyright (c) 2026 Tyler Hong
# SPDX-License-Identifier: Apache-2.0
"""Independent PS/2 host model (the computer side), written from the same
protocol description as ps2.py (A. Chapweske, "The PS/2 Mouse/Keyboard
Protocol"), not from the RTL. It runs on lib/i2c.OpenDrainBus (step()).

Device to host: reads DATA on each falling CLK edge; checks start, odd
parity, stop, the clock rate (10..16.7 kHz: each half 30..50 us) and that
DATA changes only while CLK is high.
Host to device (send()): after the device answered the previous byte, holds CLK low for `inhibit_us` (>= 100), pulls DATA
low, releases CLK; then sets each bit while CLK is low (after a falling
edge): 8 data bits, odd parity, stop (released); checks that the device
pulls DATA low as the acknowledge before the 11th falling edge and
releases it afterwards."""


class Ps2Host:
    def __init__(self, bus, clk, data, clk_ns, inhibit_us=120):
        self.bus, self.clk, self.data, self.ns = bus, clk, data, clk_ns
        self.inhibit = int(inhibit_us * 1000 / clk_ns)
        self.name = "host"
        self.received, self.errors, self.acked = [], [], []
        self.pclk, self.pdata = 1, 1
        self.bits = []
        self.t_fall = self.t_rise = None
        self.state = "idle"
        self.out = []
        self.awaiting = None  # bytes received when the last command was sent

    def send(self, b):
        self.out.append(b)

    def _set(self, pin, low):
        self.bus.set_low(self.name + str(pin), pin, low)

    def step(self, t):
        clk, data = self.bus.level[self.clk], self.bus.level[self.data]
        fall, rise = self.pclk and not clk, clk and not self.pclk
        if self.state == "idle" and self.out and clk and data and not self.bits and \
                self.awaiting is None and (self.t_rise is None or t - self.t_rise > self.inhibit):
            self.state, self.t0 = "inhibit", t
            self._set(self.clk, True)
        elif self.state == "inhibit" and t - self.t0 >= self.inhibit:
            self._set(self.data, True)                   # request to send (start bit)
            self._set(self.clk, False)
            b = self.out[0]
            par = 1 - (bin(b).count("1") & 1)
            self.tx, self.k, self.state = [(b >> i) & 1 for i in range(8)] + [par, 1], 0, "tx"
        elif self.state == "tx":
            if fall:
                if self.k < 10:
                    self._set(self.data, self.tx[self.k] == 0)   # change DATA while CLK is low
                    self.k += 1
                else:                                    # 11th falling edge: ACK must be there
                    if data != 0:
                        self.errors.append(f"t={t}: no ACK from the device")
                    self.state = "ack"
            if rise and self.k == 10 and not hasattr(self, "_sawstop"):
                self._sawstop = True
        elif self.state == "ack" and rise:
            self.state = "ackwait"
        elif self.state == "ackwait":
            if data == 1:
                self.acked.append(self.out.pop(0))
                self.state = "idle"
                self.awaiting = len(self.received)   # wait for the device's answer (0xFA) first
                del self._sawstop
        elif self.state == "idle":
            # device to host
            if self.bits and data != self.pdata and not clk:
                self.errors.append(f"t={t}: device changed DATA while CLK was low")
            if fall:
                if self.t_rise is not None and self.bits and not 30_000 <= (t - self.t_rise) * self.ns <= 50_000:
                    self.errors.append(f"t={t}: CLK high for {(t - self.t_rise) * self.ns / 1000:.1f} us")
                self.t_fall = t
                self.bits.append(data)
                if len(self.bits) == 11:
                    b = self.bits
                    v = sum(x << i for i, x in enumerate(b[1:9]))
                    if b[0] != 0 or b[10] != 1 or (sum(b[1:9]) + b[9]) % 2 != 1:
                        self.errors.append(f"t={t}: bad frame {b}")
                    self.received.append(v)
                    self.bits = []
                    if self.awaiting is not None and len(self.received) > self.awaiting:
                        self.awaiting = None
            if rise:
                if self.t_fall is not None and self.bits and not 30_000 <= (t - self.t_fall) * self.ns <= 50_000:
                    self.errors.append(f"t={t}: CLK low for {(t - self.t_fall) * self.ns / 1000:.1f} us")
                self.t_rise = t
        self.pclk, self.pdata = clk, data
