# Copyright (c) 2026 Tyler Hong
# SPDX-License-Identifier: Apache-2.0
"""Independent ARM Serial Wire Debug target model (SW-DP + one MEM-AP),
written from the ADIv5 rules, not from the RTL or the firmware:

- SWCLK is driven by the host; both sides only look at its rising edges. The
  host changes SWDIO while SWCLK is low and the target samples it on the
  rising edge. The target changes SWDIO right after a rising edge, and the
  host reads it before the next rising edge.
- SWDIO is bidirectional with a pull-up. One turnaround cycle (nobody drives)
  separates host and target phases. The model reports contention (both
  driving) and host data changes too close to a rising SWCLK edge.
- Selection: at least 50 cycles of SWDIO high (line reset), the 16-bit
  JTAG-to-SWD sequence 0xE79E (LSB first), then another line reset. After a
  line reset the DP is locked until the host reads DPIDR; other requests get
  no answer.
- Packet: 8-bit request (start 1, APnDP, RnW, A[3:2], even parity, stop 0,
  park 1), turnaround, 3-bit ACK (OK 001, WAIT 010, FAULT 100, LSB first),
  then 32 data bits + even parity (read: target drives, then a turnaround;
  write: a turnaround, then the host drives).
- DP registers: DPIDR (read 0x0), ABORT (write 0x0), CTRL/STAT (0x4; power-up
  request bits 28/30 are acknowledged in bits 29/31; STICKYERR bit 5),
  SELECT (write 0x8: APSEL [31:24], APBANKSEL [7:4]), RDBUFF (read 0xC).
- AP reads are posted: an AP read returns the result of the previous AP read;
  its own result is in RDBUFF. AP accesses before debug power-up get FAULT
  and set STICKYERR; FAULT answers every AP access until ABORT clears it.
- MEM-AP: CSW (0x00; size [2:0], AddrInc [5:4]), TAR (0x04), DRW (0x0C,
  word access at TAR, TAR += 4 with AddrInc = 1), IDR (0xFC).
"""

import cocotb
from cocotb.triggers import FallingEdge

JTAG_TO_SWD = 0xE79E
ACK_OK, ACK_WAIT, ACK_FAULT = 1, 2, 4


def parity(x):
    return bin(x).count("1") & 1


class SwdTarget:
    def __init__(self, dut, swclk, swdio, dpidr=0x2BA01477, apidr=0x24770011,
                 wait_drw=0, setup_min=2, hold_min=2):
        self.dut, self.swclk, self.swdio = dut, swclk, swdio
        self.dpidr, self.apidr = dpidr, apidr
        self.wait_drw = wait_drw  # answer the first n DRW accesses with WAIT
        self.setup_min, self.hold_min = setup_min, hold_min
        self.mem = {}
        self.errors = []
        self.log = []  # (kind, ap, addr, value, ack) per completed packet
        self.t = 0
        self.mode = "jtag"
        self.ones = 0
        self.seq = 0
        self.nseq = 0
        self.locked = True
        self.ctrl_stat = 0
        self.select = 0
        self.rdbuff = 0
        self.ap_result = 0
        self.csw = 0x00000040  # DeviceEn
        self.tar = 0
        self.fault = False
        self.drive = None  # None = released, else the level driven
        self.state = "idle"
        self.pclk = 0
        self.t_rise = -1000
        self.t_chg = -1000
        self.pchip = None

    def start(self):
        self._task = cocotb.start_soon(self._run())

    def stop(self):
        self._task.cancel()

    # ---------------------------------------------------------------- bus
    async def _run(self):
        while True:
            await FallingEdge(self.dut.clk)
            self.t += 1
            out = int(self.dut.uio_out.value)
            oe = int(self.dut.uio_oe.value)
            clk = (out >> self.swclk) & 1 if (oe >> self.swclk) & 1 else 0
            chip_on = (oe >> self.swdio) & 1
            chip_v = (out >> self.swdio) & 1
            chip = chip_v if chip_on else None
            if chip != self.pchip:
                if chip is not None and self.pchip is not None:
                    self.t_chg = self.t
                    if self.t - self.t_rise < self.hold_min:
                        self.errors.append(f"t={self.t}: SWDIO changed {self.t - self.t_rise} cycles after SWCLK rose (hold)")
                self.pchip = chip
            if chip_on and self.drive is not None:
                self.errors.append(f"t={self.t}: contention on SWDIO in state {self.state}")
            level = chip_v if chip_on else (self.drive if self.drive is not None else 1)
            if clk and not self.pclk:
                if chip_on and self.t - self.t_chg < self.setup_min:
                    self.errors.append(f"t={self.t}: SWDIO changed {self.t - self.t_chg} cycles before SWCLK rose (setup)")
                self.t_rise = self.t
                self._rise(level)
            self.pclk = clk
            if not chip_on:
                level = self.drive if self.drive is not None else 1
            v = 0
            for p in range(8):
                if (oe >> p) & 1:
                    v |= ((out >> p) & 1) << p
            v = (v & ~(1 << self.swdio)) | (level << self.swdio)
            self.dut.uio_in.value = v

    # ------------------------------------------------------- bit protocol
    def _rise(self, b):
        """One rising SWCLK edge: sample b, then update what the target drives."""
        if b:
            self.ones += 1
        else:
            self.ones = 0
        if self.state != "tx" and self.ones >= 50:
            # Line reset (only while the host drives; the target never drives 50 ones)
            if self.mode == "seq_seen":
                self.mode = "swd"
            if self.mode == "swd":
                self.locked = True
            self.state = "idle"
            self.drive = None
            self.seq = 0
            self.nseq = 0
            return
        if self.mode in ("jtag", "seq_seen"):
            # Look for the 16-bit selection sequence (LSB first) after a line reset
            self.seq = (self.seq >> 1) | (b << 15)
            self.nseq += 1
            if self.mode == "jtag" and self.seq == JTAG_TO_SWD and self.nseq == 16:
                self.mode = "seq_seen"
            return
        st = self.state
        if st == "idle":
            if b:
                self.state, self.req, self.n = "req", 1, 1
        elif st == "req":
            self.req |= b << self.n
            self.n += 1
            if self.n == 8:
                self._request()
        elif st == "trn1":
            # Turnaround edge: start driving ACK[0]
            self.state, self.n = "ack", 0
            self.drive = self.ack & 1
        elif st == "ack":
            self.n += 1
            if self.n < 3:
                self.drive = (self.ack >> self.n) & 1
            elif self.rnw and self.ack == ACK_OK:
                self.state, self.n = "tx", 0
                self.drive = self.data & 1
            elif self.ack == ACK_OK:
                self.drive, self.state = None, "wtrn"
            else:
                self.drive, self.state = None, "trn2"
                self._done(None)
        elif st == "tx":
            self.n += 1
            if self.n < 32:
                self.drive = (self.data >> self.n) & 1
            elif self.n == 32:
                self.drive = parity(self.data)
            else:
                self.drive, self.state = None, "trn2"
                self._done(self.data)
        elif st == "trn2":
            self.state = "idle"  # turnaround back to the host
        elif st == "wtrn":
            self.state, self.n, self.wdata = "rx", 0, 0
        elif st == "rx":
            if self.n < 32:
                self.wdata |= b << self.n
                self.n += 1
            else:
                if b != parity(self.wdata):
                    self.errors.append(f"t={self.t}: write data parity error ({self.wdata:#010x})")
                    self.ctrl_stat |= 1 << 7  # WDATAERR
                else:
                    self._write(self.wdata)
                self.state = "idle"
                self._done(self.wdata)
        # "lockout": ignore everything until a line reset

    def _request(self):
        r = self.req
        start, ap, rnw = r & 1, (r >> 1) & 1, (r >> 2) & 1
        a = ((r >> 3) & 3) << 2
        par, stop, park = (r >> 5) & 1, (r >> 6) & 1, (r >> 7) & 1
        self.ap, self.rnw, self.addr = ap, rnw, a
        if not (start and not stop and park) or par != parity((r >> 1) & 0xF):
            # Protocol error (also seen when a line reset starts): no answer
            # until the next line reset
            self.log.append(("bad", r, None, None, None))
            self.state = "lockout"
            return
        if self.locked and not (not ap and rnw and a == 0):
            self.log.append(("ignored", ap, a, None, None))
            self.state = "lockout"  # locked: no answer until DPIDR is read after a line reset
            return
        self.ack, self.data = self._access()
        self.state = "trn1"

    def _done(self, value):
        self.log.append(("read" if self.rnw else "write", self.ap, self.addr, value, self.ack))

    # ------------------------------------------------------------ registers
    def _access(self):
        """Decide the ACK for the current request; for a read return the data."""
        if self.ap:
            if not (self.ctrl_stat >> 29) & 1 or self.fault:
                self.fault = True
                self.ctrl_stat |= 1 << 5  # STICKYERR
                return ACK_FAULT, 0
            reg = (self.select & 0xF0) | self.addr
            if reg == 0x0C and self.wait_drw:
                self.wait_drw -= 1
                return ACK_WAIT, 0
            if self.rnw:
                prev = self.ap_result
                self.ap_result = self._ap_read(reg)
                self.rdbuff = self.ap_result
                return ACK_OK, prev
            self.pending = reg
            return ACK_OK, 0
        if self.rnw:
            if self.addr == 0x0:
                self.locked = False
                return ACK_OK, self.dpidr
            if self.addr == 0x4:
                return ACK_OK, self.ctrl_stat
            if self.addr == 0xC:
                return ACK_OK, self.rdbuff
            return ACK_OK, 0
        return ACK_OK, 0

    def _write(self, v):
        if self.ap:
            reg = self.pending
            if reg == 0x00:
                self.csw = (v & 0x37) | 0x40
            elif reg == 0x04:
                self.tar = v
            elif reg == 0x0C:
                self.mem[self.tar] = v
                self._inc()
            return
        if self.addr == 0x0:  # ABORT
            if v & (1 << 2):
                self.ctrl_stat &= ~(1 << 5)
                self.fault = False
            if v & (1 << 3):
                self.ctrl_stat &= ~(1 << 7)
        elif self.addr == 0x4:
            req = v & 0x50000000
            self.ctrl_stat = (self.ctrl_stat & 0xA0) | req | (req << 1)
        elif self.addr == 0x8:
            self.select = v

    def _ap_read(self, reg):
        if reg == 0xFC:
            return self.apidr
        if reg == 0x00:
            return self.csw
        if reg == 0x04:
            return self.tar
        if reg == 0x0C:
            v = self.mem.get(self.tar, 0)
            self._inc()
            return v
        return 0

    def _inc(self):
        if (self.csw >> 4) & 3 == 1:
            self.tar = (self.tar + 4) & 0xFFFFFFFF
