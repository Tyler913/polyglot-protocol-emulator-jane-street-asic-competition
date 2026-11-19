# Copyright (c) 2026 Tyler Hong
# SPDX-License-Identifier: Apache-2.0
"""Independent MIL-STD-1553B remote terminal (RT), written from the standard
(Manchester II bi-phase at 1 Mbit/s; 3-bit sync: command/status positive
1.5 bits then negative 1.5 bits, data the opposite; 16 data bits MSB first;
odd parity; command word = RT address (5), T/R, subaddress (5), word count
(5, 0 = 32); status word = RT address in bits 15..11; RT response time 4 to
12 us), not from the RTL.

Bus model: TXA high = positive, TXB high = negative, both low = idle; both
high is an error (beyond 2 cycles of skew). RXA (to the chip) = bus
positive, including the BC's own words, as a real transceiver shows them.
The RT samples each half-bit at its middle, from the first transition out
of idle."""

import cocotb
from cocotb.triggers import ReadOnly, RisingEdge

from .pins import bit, ui_pins

CHIP = 25  # cycles per half bit at 50 MHz


def word_chips(data, cmd_sync):
    c = [1, 1, 1, -1, -1, -1] if cmd_sync else [-1, -1, -1, 1, 1, 1]
    bits = [(data >> (15 - i)) & 1 for i in range(16)]
    bits.append(1 - (sum(bits) & 1))          # odd parity
    for b in bits:
        c += [1, -1] if b else [-1, 1]
    return c


def decode(chips):
    """(sync, data) of 40 chips; raises on a bad word."""
    s = tuple(chips[:6])
    sync = "cmd" if s == (1, 1, 1, -1, -1, -1) else "data" if s == (-1, -1, -1, 1, 1, 1) else None
    assert sync, f"bad sync {s}"
    bits = []
    for i in range(17):
        p = tuple(chips[6 + 2 * i: 8 + 2 * i])
        assert p in ((1, -1), (-1, 1)), f"not Manchester at bit {i}: {p}"
        bits.append(1 if p == (1, -1) else 0)
    assert sum(bits) % 2 == 1, "parity"
    return sync, sum(b << (15 - i) for i, b in enumerate(bits[:16]))


class Rt1553:
    def __init__(self, dut, txa_uo, txb_uo, rxa_ui, address, tx_data):
        self.dut, self.txa, self.txb, self.rxa = dut, txa_uo, txb_uo, rxa_ui
        self.addr, self.tx_data = address, list(tx_data)
        self.ui = ui_pins(dut)
        self.messages = []        # words decoded from the BC, per message
        self.received = {}        # subaddress -> data words
        self.responses = []       # (gap in us, words) the RT sent
        self.errors = []
        self.drive = []           # chip levels the RT sends
        self.glitches = []        # activity shorter than a word, outside messages
        self.ui.set(rxa_ui, 0)

    def start(self):
        self._task = cocotb.start_soon(self._run())

    def stop(self):
        self._task.cancel()

    def _handle(self, words, t_end):
        if not words or words[0][0] != "cmd":
            self.errors.append(f"message does not start with a command word: {words}")
            return
        c = words[0][1]
        rt, tr, sa, wc = c >> 11, (c >> 10) & 1, (c >> 5) & 31, c & 31 or 32
        if rt != self.addr:
            return
        status = [self.addr << 11]
        if tr == 0:
            data = [w for s, w in words[1:]]
            if len(data) != wc or any(s != "data" for s, _ in words[1:]):
                self.errors.append(f"receive command for {wc} words got {words[1:]}")
            self.received[sa] = data
            out = status
        else:
            out = status + self.tx_data[:wc]
        chips = []
        for i, w in enumerate(out):
            chips += word_chips(w, i == 0)
        self.drive = chips
        self.responses.append(out)

    async def _run(self):
        t, chips, start, msg, idle_since = 0, [], None, [], None
        resp_at, k = None, 0
        while True:
            await RisingEdge(self.dut.clk)
            await ReadOnly()
            t += 1
            a, b = bit(self.dut.uo_out, self.txa), bit(self.dut.uo_out, self.txb)
            if a and b:
                self._both = getattr(self, "_both", 0) + 1
                if self._both > 2:
                    self.errors.append(f"TXA and TXB both high at {t}")
            else:
                self._both = 0
            lvl = 1 if a and not b else -1 if b and not a else 0
            if start is None and lvl != 0:
                start, chips, msg = t, [], []
                nxt = start + CHIP // 2               # next half-bit center
            if start is not None:
                if t >= nxt:                          # (this loop advances 2 cycles per pass)
                    nxt += CHIP
                    if chips or lvl != 0:             # idle after a word is not a new word
                        chips.append(lvl)
                    if len(chips) == 40:
                        try:
                            msg.append(decode(chips))
                        except AssertionError as e:
                            self.errors.append(f"BC word: {e}")
                        chips = []
                if lvl == 0:
                    idle_since = idle_since or t
                    if t - idle_since > CHIP:            # idle for a half bit: message over
                        if chips:
                            self.glitches.append((start, len(chips)))
                        if msg:
                            self.messages.append(msg)
                            self._handle(msg, t)
                        if self.drive:
                            resp_at, k = t + 6 * 50 - CHIP, 0   # ~6 us after the BC's last half bit
                        start, chips, idle_since = None, [], None
                else:
                    idle_since = None
            rt_lvl = 0
            if resp_at is not None and t >= resp_at:
                k = (t - resp_at) // CHIP
                if k < len(self.drive):
                    rt_lvl = self.drive[k]
                else:
                    resp_at, self.drive = None, []
            await RisingEdge(self.dut.clk)
            t += 1
            self.ui.set(self.rxa, 1 if (lvl > 0 or rt_lvl > 0) else 0)
