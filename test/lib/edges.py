# Copyright (c) 2026 Tyler Hong
# SPDX-License-Identifier: Apache-2.0
"""Event-driven edge recorder for one output bit: wakes only when uo_out or
uio_out changes, so slow protocols cost little Python time. Records
(time_ns, level) for every change of the chosen bit."""

import cocotb

from cocotb.utils import get_sim_time

from .pins import bit


class EdgeRecorder:
    def __init__(self, dut, bus="uo_out", index=0):
        self.dut, self.sig, self.i = dut, getattr(dut, bus), index
        self.edges = []
        self.level = None

    def _read(self):
        try:
            return bit(self.sig, self.i)
        except ValueError:
            return None

    def start(self):
        self.level = self._read()
        self._task = cocotb.start_soon(self._run())

    def stop(self):
        self._task.cancel()

    async def _run(self):
        while True:
            await self.sig.value_change
            v = self._read()
            if v is not None and v != self.level:
                self.edges.append((get_sim_time("ns"), v))
                self.level = v

    def pulses(self, level):
        """(start_ns, width_ns) of every completed pulse at `level`."""
        out = []
        for (t0, v0), (t1, _) in zip(self.edges, self.edges[1:]):
            if v0 == level:
                out.append((t0, t1 - t0))
        return out
