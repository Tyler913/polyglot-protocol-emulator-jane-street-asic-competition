# Copyright (c) 2026 Tyler Hong
# SPDX-License-Identifier: Apache-2.0
"""Independent NEC infrared receiver model (an IR receiver module plus an NEC
decoder), from the NEC protocol description, not from the firmware:
- carrier 38 kHz (period 26.3 us, checked to +/- 2 %), duty 25..60 %;
- a mark is a burst of carrier pulses (a demodulator sees it from the first
  rising to the last falling edge), a space is the gap between bursts;
- frame: 9 ms mark, 4.5 ms space, 32 bits LSB first (each a 562.5 us mark
  and a 562.5 us or 1687.5 us space), final 562.5 us mark; repeat code:
  9 ms mark, 2.25 ms space, 562.5 us mark. Durations within +/- 10 %;
- the 32 bits are address, ~address, command, ~command (checked)."""

import cocotb
from cocotb.triggers import FallingEdge


class NecReceiver:
    def __init__(self, dut, uo_bit, clk_ns):
        self.dut, self.bit, self.ns = dut, uo_bit, clk_ns
        self.edges = []  # (t_ns, level)
        self.errors = []

    def start(self):
        self._task = cocotb.start_soon(self._run())

    def stop(self):
        self._task.cancel()

    async def _run(self):
        t, prev = 0, 0
        while True:
            await FallingEdge(self.dut.clk)
            t += 1
            v = (int(self.dut.uo_out.value) >> self.bit) & 1
            if v != prev:
                self.edges.append((t * self.ns, v))
                prev = v

    def decode(self):
        """Return [("frame", addr, cmd) | ("repeat",)], checking timing on the way."""
        rises = [t for t, v in self.edges if v == 1]
        falls = [t for t, v in self.edges if v == 0]
        if not rises:
            return []
        period = 1e9 / 38000
        bursts, start, last_r = [], rises[0], rises[0]
        fall_of = dict(zip(rises, falls))           # each rise and its following fall
        for r in rises[1:]:
            gap = r - last_r
            if gap < 2 * period:                     # same burst: check the carrier
                if abs(gap - period) > 0.02 * period:
                    self.errors.append(f"carrier period {gap / 1000:.2f} us at {r / 1e6:.3f} ms")
            else:
                bursts.append((start, fall_of[last_r]))
                start = r
            high = fall_of.get(r, r) - r
            if not 0.25 * period <= high <= 0.60 * period:
                self.errors.append(f"carrier duty {high / period:.2f} at {r / 1e6:.3f} ms")
            last_r = r
        bursts.append((start, fall_of[last_r]))

        def near(x, us):
            return abs(x / 1000 - us) <= 0.10 * us

        out, i = [], 0
        while i < len(bursts):
            mark = bursts[i][1] - bursts[i][0]
            space = bursts[i + 1][0] - bursts[i][1] if i + 1 < len(bursts) else None
            if not near(mark, 9000) or space is None:
                self.errors.append(f"expected a 9 ms leader, got a {mark / 1000:.0f} us mark")
                i += 1
                continue
            if near(space, 2250):                    # repeat code
                tail = bursts[i + 1][1] - bursts[i + 1][0]
                if not near(tail, 562.5):
                    self.errors.append(f"repeat code end mark {tail / 1000:.0f} us")
                out.append(("repeat",))
                i += 2
                continue
            if not near(space, 4500):
                self.errors.append(f"leader space {space / 1000:.0f} us")
            bits = []
            for k in range(32):
                b0, b1 = bursts[i + 1 + k], bursts[i + 2 + k]
                m, s = b0[1] - b0[0], b1[0] - b0[1]
                if not near(m, 562.5):
                    self.errors.append(f"bit {k} mark {m / 1000:.0f} us")
                if near(s, 562.5):
                    bits.append(0)
                elif near(s, 1687.5):
                    bits.append(1)
                else:
                    self.errors.append(f"bit {k} space {s / 1000:.0f} us")
                    bits.append(0)
            end = bursts[i + 33]
            if not near(end[1] - end[0], 562.5):
                self.errors.append(f"end mark {(end[1] - end[0]) / 1000:.0f} us")
            v = sum(b << k for k, b in enumerate(bits))
            a, na, c, nc = v & 0xFF, (v >> 8) & 0xFF, (v >> 16) & 0xFF, v >> 24
            if a ^ na != 0xFF or c ^ nc != 0xFF:
                self.errors.append(f"inverse check failed: {v:#010x}")
            out.append(("frame", a, c))
            self.frame_starts = getattr(self, "frame_starts", []) + [bursts[i][0]]
            i += 34
        self.leaders = [b[0] for b in bursts if near(b[1] - b[0], 9000)]
        return out


def _synthetic(freq=38000, leader=9000, data=0xC33CA55A, bitmark=562.5, duty=0.5):
    """Edges of an ideal NEC frame with the given faults (for the self-test)."""
    p, t, edges = 1e9 / freq, 1e6, []

    def mark(us):
        nonlocal t
        n = round(us * 1000 / p)
        for k in range(n):
            edges.extend([(t + k * p, 1), (t + k * p + duty * p, 0)])
        t += n * p

    mark(leader)
    t += 4500 * 1000
    for i in range(32):
        mark(bitmark)
        t += (1687.5 if (data >> i) & 1 else 562.5) * 1000
    mark(562.5)
    return edges


if __name__ == "__main__":
    # Self-test of the model on synthetic waveforms (the full simulation takes
    # about 20 minutes): `cd test && python -m lib.ir_nec`
    for kw, bad in [({}, False), ({"freq": 40000}, True), ({"leader": 8000}, True),
                    ({"data": 0xC33CA55B}, True), ({"duty": 0.15}, True), ({"bitmark": 700}, True)]:
        r = NecReceiver.__new__(NecReceiver)
        r.errors, r.edges = [], _synthetic(**kw)
        out = r.decode()
        assert bool(r.errors) == bad, (kw, out, r.errors[:2])
        print(kw or "ideal frame", "->", out, r.errors[:1])
    print("model self-test passed")
