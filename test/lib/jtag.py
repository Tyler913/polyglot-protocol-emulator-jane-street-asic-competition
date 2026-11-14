# Copyright (c) 2026 Tyler Hong
# SPDX-License-Identifier: Apache-2.0
"""Independent IEEE 1149.1 (JTAG) chain model, written from the standard,
not from the RTL or the firmware.

- TCK, TMS and TDI come from the chip; TDO goes back to it. The TAP
  controllers sample TMS and TDI on the rising TCK edge and change TDO on the
  falling edge. The model checks that TMS and TDI do not change within
  setup_min / hold_min clock cycles of a rising TCK edge.
- Each device has the 16-state TAP controller, an instruction register of
  its own length (Capture-IR loads ...01, as the standard requires), and the
  data registers IDCODE (32 bits, selected after Test-Logic-Reset), BYPASS
  (1 bit, all-ones instruction) and one 8-bit USER register (instruction
  `user_ir`) whose value is updated on Update-DR.
- Devices are chained: chip TDI -> device 0 -> device 1 -> ... -> chip TDO.
"""

import cocotb
from cocotb.triggers import FallingEdge

# TAP states
TLR, RTI, SEL_DR, CAP_DR, SH_DR, EX1_DR, PAU_DR, EX2_DR, UPD_DR = range(9)
SEL_IR, CAP_IR, SH_IR, EX1_IR, PAU_IR, EX2_IR, UPD_IR = range(9, 16)

NEXT = {  # state: (next if TMS = 0, next if TMS = 1)
    TLR: (RTI, TLR), RTI: (RTI, SEL_DR),
    SEL_DR: (CAP_DR, SEL_IR), CAP_DR: (SH_DR, EX1_DR), SH_DR: (SH_DR, EX1_DR),
    EX1_DR: (PAU_DR, UPD_DR), PAU_DR: (PAU_DR, EX2_DR), EX2_DR: (SH_DR, UPD_DR),
    UPD_DR: (RTI, SEL_DR),
    SEL_IR: (CAP_IR, TLR), CAP_IR: (SH_IR, EX1_IR), SH_IR: (SH_IR, EX1_IR),
    EX1_IR: (PAU_IR, UPD_IR), PAU_IR: (PAU_IR, EX2_IR), EX2_IR: (SH_IR, UPD_IR),
    UPD_IR: (RTI, SEL_DR),
}


class TapDevice:
    def __init__(self, ir_len, idcode, idcode_ir, user_ir, user=0):
        self.ir_len, self.idcode = ir_len, idcode
        self.idcode_ir, self.user_ir = idcode_ir, user_ir
        self.user = user
        self.state = TLR
        self.ir = idcode_ir
        self.ir_sh = 0
        self.dr_sh = 0
        self.dr_len = 32
        self.tdo = 0  # value presented on TDO (changes on the falling edge)
        self.updates = []  # (instruction, value) at each Update-DR of a writable DR

    def _dr(self):
        if self.ir == self.idcode_ir:
            return self.idcode, 32
        if self.ir == self.user_ir:
            return self.user, 8
        return 0, 1  # BYPASS (and any other instruction here)

    def rise(self, tms, tdi):
        """Rising TCK: shift in the current state, then move."""
        s = self.state
        if s == SH_DR:
            self.dr_sh = (self.dr_sh >> 1) | (tdi << (self.dr_len - 1))
        elif s == SH_IR:
            self.ir_sh = (self.ir_sh >> 1) | (tdi << (self.ir_len - 1))
        elif s == CAP_DR:
            self.dr_sh, self.dr_len = self._dr()
        elif s == CAP_IR:
            self.ir_sh = 0b01
        nxt = NEXT[s][tms]
        if nxt == UPD_IR:
            self.ir = self.ir_sh
        if nxt == UPD_DR and self.ir == self.user_ir:
            self.user = self.dr_sh
            self.updates.append((self.ir, self.dr_sh))
        if nxt == TLR:
            self.ir = self.idcode_ir
        self.state = nxt

    def fall(self):
        """Falling TCK: TDO shows the bit that will leave on the next rising edge."""
        if self.state == SH_DR:
            self.tdo = self.dr_sh & 1
        elif self.state == SH_IR:
            self.tdo = self.ir_sh & 1
        return self.tdo


class JtagChain:
    def __init__(self, dut, tck, tms, tdi, tdo, devices, setup_min=2, hold_min=2):
        """tck/tms/tdi: chip output pin numbers in uo_out; tdo: ui_in bit."""
        self.dut, self.devices = dut, devices
        self.tck, self.tms, self.tdi, self.tdo = tck, tms, tdi, tdo
        self.setup_min, self.hold_min = setup_min, hold_min
        self.errors = []
        self.t = 0
        self.rises = 0
        self.pclk = 0
        self.t_rise = -1000
        self.t_chg = -1000
        self.pin = None
        from .pins import ui_pins
        self.ui = ui_pins(dut)
        # The TDO wires between devices change on falling edges only
        self.links = [0] * len(devices)

    def start(self):
        self._task = cocotb.start_soon(self._run())

    def stop(self):
        self._task.cancel()

    async def _run(self):
        while True:
            await FallingEdge(self.dut.clk)
            self.t += 1
            out = int(self.dut.uo_out.value)
            tck = (out >> self.tck) & 1
            tms = (out >> self.tms) & 1
            tdi = (out >> self.tdi) & 1
            if (tms, tdi) != self.pin:
                if self.pin is not None:
                    self.t_chg = self.t
                    if self.t - self.t_rise < self.hold_min:
                        self.errors.append(f"t={self.t}: TMS/TDI changed {self.t - self.t_rise} cycles after TCK rose (hold)")
                self.pin = (tms, tdi)
            if tck and not self.pclk:
                if self.t - self.t_chg < self.setup_min:
                    self.errors.append(f"t={self.t}: TMS/TDI changed {self.t - self.t_chg} cycles before TCK rose (setup)")
                self.t_rise = self.t
                self.rises += 1
                din = tdi
                for i, d in enumerate(self.devices):
                    d.rise(tms, din)
                    din = self.links[i]
            elif self.pclk and not tck:
                for i, d in enumerate(self.devices):
                    self.links[i] = d.fall()
                self.ui.set(self.tdo, self.links[-1])
            self.pclk = tck
