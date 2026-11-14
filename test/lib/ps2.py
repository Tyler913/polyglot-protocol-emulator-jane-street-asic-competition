# Copyright (c) 2026 Tyler Hong
# SPDX-License-Identifier: Apache-2.0
"""Independent PS/2 keyboard model (device side), written from the PS/2
protocol description (A. Chapweske, "The PS/2 Mouse/Keyboard Protocol"),
not from the RTL or the firmware. Both lines are open drain with pull-ups
(lib/i2c.OpenDrainBus); the device always generates the clock.

Device to host: 11-bit frames (start 0, 8 data bits LSB first, odd parity,
stop 1). The device changes DATA while CLK is high and the host reads it
on the falling edge. CLK period `period` cycles (10..16.7 kHz).

Host to device: the host holds CLK low for >= 100 us (checked), pulls DATA
low (request to send) and releases CLK. The device then clocks: the host
sets each bit while CLK is low and the device reads it on the rising edge:
8 data bits, odd parity, stop (released DATA). On the 11th clock the device
pulls DATA low (ACK). The model checks that the host changes DATA only while
CLK is low, the parity and the stop bit, then answers each command byte
with 0xFA (acknowledge), and records the commands.

If the host pulls CLK low while the device sends (inhibit), the device
aborts the frame and sends it again later.
"""

T_INHIBIT_MIN_US = 100


class Ps2Keyboard:
    def __init__(self, bus, clk, data, clk_ns, period=4000, reply_delay=2000):
        self.bus, self.clk, self.data, self.ns = bus, clk, data, clk_ns
        self.period = period              # cycles per clock (4000 at 50 MHz = 12.5 kHz)
        self.reply_delay = reply_delay    # cycles between frames
        self.name = "kbd"
        self.queue = []                   # bytes to send to the host
        self.sent = []                    # bytes the host received completely
        self.commands = []                # bytes received from the host
        self.errors = []
        self.state = "idle"
        self.t_next = 0
        self.inhibit_from = None
        self.my_clk_low = False
        self.my_data_low = False
        self.pdata = 1
        self.pclk = 1

    def send(self, *bytes_):
        self.queue.extend(bytes_)

    def _clk(self, low):
        self.my_clk_low = low
        self.bus.set_low(self.name + "_clk", self.clk, low)

    def _data(self, low):
        self.my_data_low = low
        self.bus.set_low(self.name + "_data", self.data, low)

    @staticmethod
    def frame(b):
        par = 1 - (bin(b).count("1") & 1)
        return [0] + [(b >> i) & 1 for i in range(8)] + [par, 1]

    def step(self, t):
        clk, data = self.bus.level[self.clk], self.bus.level[self.data]
        host_clk_low = (clk == 0) and not self.my_clk_low
        # Host DATA changes are allowed only while CLK is low (host-to-device)
        if self.state == "rx" and data != self.pdata and not self.my_data_low and clk == 1:
            self.errors.append(f"t={t}: host changed DATA while CLK was high")
        self.pdata, self.pclk = data, clk

        if self.state in ("idle", "gap"):
            if host_clk_low:
                if self.inhibit_from is None:
                    self.inhibit_from = t
                return
            if self.inhibit_from is not None:          # host released CLK
                low_us = (t - self.inhibit_from) * self.ns / 1000
                self.inhibit_from = None
                if data == 0:                          # request to send
                    if low_us < T_INHIBIT_MIN_US:
                        self.errors.append(f"t={t}: CLK inhibit of {low_us:.1f} us before a request (< 100 us)")
                    self.state, self.k, self.bits = "rx", 0, []
                    self.t_next = t + self.period // 2
                    return
                self.t_next = max(self.t_next, t + self.reply_delay)
            if self.state == "gap" and t >= self.t_next:
                self.state = "idle"
            if self.state == "idle" and self.queue and t >= self.t_next and clk and data:
                self.state, self.k, self.bits = "tx", 0, self.frame(self.queue[0])
                self.phase = 0
                self.t_next = t
            return

        if self.state == "tx":
            if host_clk_low and not self.my_clk_low:   # inhibited: abort, retry later
                self._data(False)
                self.state = "idle"
                self.inhibit_from = t
                return
            if t < self.t_next:
                return
            q = self.period // 4
            if self.phase == 0:                        # CLK high: present bit k
                self._data(self.bits[self.k] == 0)
                self.phase, self.t_next = 1, t + q
            elif self.phase == 1:                      # falling edge: host reads
                self._clk(True)
                self.phase, self.t_next = 2, t + 2 * q
            elif self.phase == 2:                      # rising edge
                self._clk(False)
                self.k += 1
                if self.k == 11:
                    self._data(False)
                    self.sent.append(self.queue.pop(0))
                    self.state, self.t_next = "gap", t + self.reply_delay
                else:
                    self.phase, self.t_next = 0, t + q
            return

        if self.state == "rx":
            if t < self.t_next:
                return
            h = self.period // 2
            if not self.my_clk_low:                    # falling edge: host sets the next bit
                self._clk(True)
                self.t_next = t + h
                return
            self._clk(False)                           # rising edge: read DATA
            self.k += 1
            self.t_next = t + h
            if self.k <= 10:
                self.bits.append(data)
                if self.k == 10:
                    d = self.bits[:8]
                    b = sum(v << i for i, v in enumerate(d))
                    if (sum(d) + self.bits[8]) % 2 != 1:
                        self.errors.append(f"t={t}: parity error in host byte {b:#04x}")
                    if self.bits[9] != 1:
                        self.errors.append(f"t={t}: stop bit 0 in host byte {b:#04x}")
                    self.commands.append(b)
                    self._data(True)                   # ACK during the 11th clock
            else:                                      # end of the 11th clock
                self._data(False)
                self.queue.insert(0, 0xFA)             # acknowledge the command
                self.state, self.t_next = "gap", t + self.reply_delay
