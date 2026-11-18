# Copyright (c) 2026 Tyler Hong
# SPDX-License-Identifier: Apache-2.0
"""Independent clause 22 PHY management interface, written from IEEE 802.3
clause 22.2.4.5 (management frame: preamble of 32 ones, ST 01, OP 10 read /
01 write, PHYAD, REGAD, TA, 16 data bits, MSB first; MDIO sampled on the
rising edge of MDC; on a read the PHY releases MDIO for the first
turnaround bit and drives 0 in the second, then the data, each driven after
a rising edge), not from the RTL.

MDIO is a tri-state line with a pull-up: the chip drives it when its
output enable is set, the PHY when it is reading out, otherwise it reads 1.
Both driving at once is an error. Event driven (uio edges)."""

import cocotb


from .pins import bit


class MdioPhy:
    def __init__(self, dut, mdio, mdc, phy_addr, regs):
        self.dut, self.mdio, self.mdc, self.addr = dut, mdio, mdc, phy_addr
        self.regs = dict(regs)
        self.errors, self.frames = [], []
        self.phy_oe, self.phy_val = False, 1
        self.bits = []
        self.state = "idle"
        self.ones = 0

    def start(self):
        self._task = cocotb.start_soon(self._run())

    def stop(self):
        self._task.cancel()

    def _chip(self):
        oe, out = int(self.dut.uio_oe.value), int(self.dut.uio_out.value)
        return (oe >> self.mdio) & 1, (out >> self.mdio) & 1, (out >> self.mdc) & 1, (oe >> self.mdc) & 1

    def _resolve(self):
        c_oe, c_val, _, _ = self._chip()
        if c_oe and self.phy_oe:
            self.errors.append("chip and PHY drive MDIO together")
        v = c_val if c_oe else (self.phy_val if self.phy_oe else 1)
        cur = int(self.dut.uio_in.value)
        self.dut.uio_in.value = (cur & ~(1 << self.mdio)) | (v << self.mdio)
        return v

    async def _run(self):
        last_mdc = 0
        while True:
            await First2(self.dut)
            line = self._resolve()
            _, _, mdc, mdc_oe = self._chip()
            if mdc and not last_mdc and mdc_oe:
                self._rise(line)
                self._resolve()
            last_mdc = mdc

    def _rise(self, b):
        # the PHY's output for the next bit changes just after this rising edge
        if self.state == "idle":
            self.ones = self.ones + 1 if b else 0
            if self.ones >= 32:
                self.state = "pre"
            return
        if self.state == "pre":
            if b == 1:
                return  # more preamble
            self.state, self.bits = "hdr", [0]
            return
        if self.state == "hdr":
            self.bits.append(b)
            if len(self.bits) == 14:
                st, op = self.bits[0:2], self.bits[2:4]
                phy = int("".join(map(str, self.bits[4:9])), 2)
                reg = int("".join(map(str, self.bits[9:14])), 2)
                if st != [0, 1]:
                    self.errors.append(f"start {st}")
                self.cur = (op, phy, reg)
                if op == [1, 0] and phy == self.addr:
                    self.state, self.n = "rd_ta", 0  # TA1: released; drive 0 after the next edge
                elif op == [0, 1]:
                    self.state, self.bits = "wr", []
                else:
                    self.state, self.n = "skip", 18
            return
        if self.state == "rd_ta":  # this edge sampled TA1 (nobody drives)
            if b != 1:
                self.errors.append("TA1 not released")
            self.phy_oe, self.phy_val = True, 0
            v = self.regs.get(self.cur[2], 0)
            self.out = [(v >> (15 - k)) & 1 for k in range(16)]
            self.state = "rd"
            return
        if self.state == "rd":  # TA2 or a data bit was just sampled: present the next one
            if self.out:
                self.phy_val = self.out.pop(0)
            else:
                self.phy_oe = False
                op, phy, reg = self.cur
                self.frames.append(("read", phy, reg, self.regs.get(reg, 0)))
                self.state, self.ones = "idle", 0
            return
        if self.state == "wr":
            self.bits.append(b)
            if len(self.bits) == 18:
                if self.bits[0:2] != [1, 0]:
                    self.errors.append(f"write TA {self.bits[0:2]}")
                v = int("".join(map(str, self.bits[2:])), 2)
                op, phy, reg = self.cur
                if phy == self.addr:
                    self.regs[reg] = v
                self.frames.append(("write", phy, reg, v))
                self.state, self.ones = "idle", 0
            return
        if self.state == "skip":
            self.n -= 1
            if self.n == 0:
                self.state, self.ones = "idle", 0


async def First2(dut):
    from cocotb.triggers import First

    await First(dut.uio_out.value_change, dut.uio_oe.value_change)
