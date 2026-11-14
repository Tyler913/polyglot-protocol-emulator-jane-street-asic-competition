# Copyright (c) 2026 Tyler Hong
# SPDX-License-Identifier: Apache-2.0
"""Independent DMX512 receiver model, from ANSI E1.11 (transmitter timing):
- bit time 4 us (250 kbit/s, 3.92..4.08 us); a slot is a start bit (low),
  8 data bits LSB first and 2 stop bits (high);
- a packet starts with a BREAK (low, transmitted >= 92 us; anything low for
  longer than a slot's 9 low bits but < 88 us is an error) and a MARK AFTER
  BREAK (high, transmitted >= 12 us, < 1 s);
- slot 0 is the start code, then up to 512 channel slots.
Every edge inside a slot must sit on the bit grid of its start edge (+/- 2 %
of a bit). The line is ignored until it first goes high (reset state)."""

import cocotb
from cocotb.triggers import FallingEdge


class Dmx512Model:
    def __init__(self, dut, uo_bit, clk_ns):
        self.dut, self.bit, self.ns = dut, uo_bit, clk_ns
        self.packets = []  # complete packets: slot lists (slot 0 = start code)
        self.cur = None    # the packet being received
        self.errors = []

    def start(self):
        self._task = cocotb.start_soon(self._run())

    def stop(self):
        self._task.cancel()

    async def _run(self):
        t, ns = 0, self.ns
        bit_c = 4000 / ns                  # cycles per bit
        tol = 0.02 * bit_c
        level, t_edge = None, 0
        state = "reset"
        slot_start, edges = None, []

        def level_at(k):                   # level in bit k of the slot, from its edges
            v = 0
            for e, lv in edges:
                if e <= slot_start + (k + 0.5) * bit_c:
                    v = lv
            return v

        def finish_slot():
            n = len(self.cur)
            data = sum(level_at(1 + i) << i for i in range(8))
            if not (level_at(9) and level_at(10)):
                self.errors.append(f"slot {n}: stop bits not high")
            for e, _ in edges[1:]:
                k = (e - slot_start) / bit_c
                if abs(k - round(k)) * bit_c > tol:
                    self.errors.append(f"slot {n}: edge {k:.3f} bits after the start bit")
            self.cur.append(data)

        while True:
            await FallingEdge(self.dut.clk)
            t += 1
            v = (int(self.dut.uo_out.value) >> self.bit) & 1
            if level is None:
                level = v
            if state == "slot" and t - slot_start >= 11 * bit_c:   # 11 bit times since the start bit
                if len(edges) == 1 and v == 0:
                    state = "low"              # still low: a BREAK, not a slot
                else:
                    finish_slot()
                    state = "mark"
            if v == level:
                continue
            dur = (t - t_edge) * ns    # how long the previous level lasted
            if state == "reset":
                if v:
                    state = "mark"
            elif v:                    # rising edge: end of a low period
                if dur >= 88_000:
                    if dur < 92_000:
                        self.errors.append(f"BREAK of {dur / 1000:.1f} us (< 92 us)")
                    if self.cur is not None:
                        self.packets.append(self.cur)
                    self.cur, state = [], "mab"
                elif dur > 9 * 4080 and state != "slot":
                    self.errors.append(f"low for {dur / 1000:.1f} us: neither data nor BREAK")
                if state == "slot":
                    edges.append((t, 1))
            else:                      # falling edge
                if state == "mab":
                    if dur < 12_000:
                        self.errors.append(f"MARK AFTER BREAK of {dur / 1000:.1f} us (< 12 us)")
                    state = "mark"
                if state == "mark" and self.cur is not None:
                    state, slot_start, edges = "slot", t, [(t, 0)]
                elif state == "slot":
                    edges.append((t, 0))
            level, t_edge = v, t

    def all_packets(self):
        """Complete packets plus the last one (no BREAK has ended it yet)."""
        return self.packets + ([self.cur] if self.cur else [])
