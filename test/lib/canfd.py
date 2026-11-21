# Copyright (c) 2026 Tyler Hong
# SPDX-License-Identifier: Apache-2.0
"""Independent CAN FD model (ISO 11898-1:2015 frame format), written from the
standard's description as summarised in public sources (Bosch CAN FD
specification v1.0 and the ISO changes: stuff count and CRC initial value;
CiA CAN FD pages), not from our RTL.

Frame: SOF, 11-bit ID (or base ID, SRR, IDE, 18-bit extension), RRS, IDE, FDF
(recessive), res, BRS, ESI, DLC, 0..64 data bytes. Dynamic stuffing (after five
equal bits) from SOF to the end of the data field. CRC field: stuff count
(number of dynamic stuff bits mod 8, Gray coded, plus an even parity bit), then
CRC-17 (up to 16 data bytes) or CRC-21, with a fixed stuff bit (complement of
the bit before it) before the stuff count and after every fourth bit; a
dynamic stuff bit due after the last data bit is replaced by the first fixed
stuff bit. The CRC (initial value 1 followed by zeros) runs over SOF..data with
the dynamic stuff bits, plus the stuff count. With BRS recessive the data bit
rate applies from the sample point of BRS to the sample point of the CRC
delimiter. Classic frames (FDF dominant: r0) follow CAN 2.0 (lib/can.py).

CanFdNode: a controller on lib.can.CanBus with its own nominal and data bit
times and sample points: hard sync on SOF, resync on recessive-to-dominant
edges, destuffing and frame checks, ACK for good frames, and one frame to send
(classic or FD), with arbitration.
"""

from .can import EFF, crc15

DLC_LEN = [0, 1, 2, 3, 4, 5, 6, 7, 8, 12, 16, 20, 24, 32, 48, 64]


def crc_bits(bits, width, poly, init):
    c, top, mask = init, 1 << (width - 1), (1 << width) - 1
    for b in bits:
        fb = b ^ (1 if c & top else 0)
        c = (c << 1) & mask
        if fb:
            c ^= poly
    return c


def crc17(bits):
    return crc_bits(bits, 17, 0x1685B, 1 << 16)


def crc21(bits):
    return crc_bits(bits, 21, 0x102899, 1 << 20)


def gray_sc(n):
    """Stuff count field: 3-bit Gray code of n mod 8 and an even parity bit."""
    g = (n % 8) ^ ((n % 8) >> 1)
    bits = [(g >> 2) & 1, (g >> 1) & 1, g & 1]
    return bits + [sum(bits) & 1]


def len_to_dlc(n):
    return DLC_LEN.index(n)


def header_bits(fid, n, fd=True, brs=True, esi=0):
    dlc = len_to_dlc(n) if fd else n
    if fid & EFF:
        base, ext = (fid >> 18) & 0x7FF, fid & 0x3FFFF
        bits = [0] + [(base >> (10 - i)) & 1 for i in range(11)] + [1, 1]
        bits += [(ext >> (17 - i)) & 1 for i in range(18)]
        bits += [0, 1, 0, int(brs), esi] if fd else [0, 0, 0]       # RRS, FDF, res, BRS, ESI / RTR, r1, r0
    else:
        bits = [0] + [(fid >> (10 - i)) & 1 for i in range(11)]
        bits += [0, 0, 1, 0, int(brs), esi] if fd else [0, 0, 0]    # RRS, IDE, FDF, res, BRS, ESI / RTR, IDE, r0
    return bits + [(dlc >> (3 - i)) & 1 for i in range(4)]


def fd_wire_bits(fid, data, brs=True, esi=0, bad_crc=False):
    """Wire bits SOF..CRC delimiter of an FD frame, as (bit, kind) with kind
    'arb' (arbitration field), 'n' (nominal), 'brs', 'd' (data phase), 'dlm'."""
    seq = header_bits(fid, len(data), True, brs, esi)
    for x in data:
        seq += [(x >> (7 - i)) & 1 for i in range(8)]
    narb = 32 if fid & EFF else 13            # ID .. RRS
    nbrs = 35 if fid & EFF else 16            # index of BRS
    out, run, last, dsc = [], 0, None, 0
    for i, b in enumerate(seq):
        kind = "arb" if i <= narb else ("n" if i < nbrs else ("brs" if i == nbrs else "d"))
        out.append((b, kind))
        run = run + 1 if b == last else 1
        last = b
        if run == 5 and i < len(seq) - 1:      # after the last data bit: replaced by the fixed stuff bit
            out.append((1 - b, "n" if i < nbrs else "d"))
            dsc += 1
            last, run = 1 - b, 1
    sc = gray_sc(dsc)
    if len(data) <= 16:
        c, w = crc17([b for b, _ in out] + sc), 17
    else:
        c, w = crc21([b for b, _ in out] + sc), 21
    c ^= 1 if bad_crc else 0
    tail = sc + [(c >> (w - 1 - i)) & 1 for i in range(w)]
    prev = out[-1][0]
    for i, b in enumerate(tail):
        if i % 4 == 0:
            out.append((1 - prev, "d"))
            prev = 1 - prev
        out.append((b, "d"))
        prev = b
    out.append((1, "dlm"))
    if not brs:
        out = [(b, "arb" if k == "arb" else "n") for b, k in out]
    return out


def classic_wire_bits(fid, data, bad_crc=False):
    seq = header_bits(fid, len(data), fd=False)
    for x in data:
        seq += [(x >> (7 - i)) & 1 for i in range(8)]
    c = crc15(seq) ^ (1 if bad_crc else 0)
    seq += [(c >> (14 - i)) & 1 for i in range(15)]
    narb = 32 if fid & EFF else 12
    out, run, last = [], 0, None
    for i, b in enumerate(seq):
        out.append((b, "arb" if i <= narb else "n"))
        run = run + 1 if b == last else 1
        last = b
        if run == 5:
            out.append((1 - b, "n"))
            last, run = 1 - b, 1
    return out + [(1, "n")]                   # CRC delimiter


class CanFdNode:
    def __init__(self, bus, name, tn, td, spn=0.8, spd=0.75):
        self.bus, self.name = bus, name
        self.tn, self.td = tn, td
        self.spn, self.spd = round(spn * tn), round(spd * td)
        self.tx = 1
        self.frames, self.errors = [], []
        self.pending = None
        self.prev = 1
        self.idle_since = 0
        self.state = "idle"
        self.tx_bits = None

    # ---- sending -------------------------------------------------------
    def send(self, fid, data, fd=True, brs=True, join_sof=False, bad_crc=False):
        self.pending = dict(fid=fid, data=list(data), fd=fd, brs=brs, bad_crc=bad_crc)
        self.join_sof = join_sof

    def _wire(self):
        p = self.pending
        if p["fd"]:
            return fd_wire_bits(p["fid"], p["data"], p["brs"], 0, p["bad_crc"])
        return classic_wire_bits(p["fid"], p["data"], p["bad_crc"])

    # ---- bit timing ----------------------------------------------------
    def _hard_sync(self, t):
        self.state = "frame"
        self.bs, self.bl, self.sp = t, self.tn, self.spn
        self.next_bs = t + self.tn
        self.sampled = False
        self.rx = dict(bits=[], crcbits=[], run=0, last=None, dsc=0, mode="dyn", fsb=False,
                       tail=[], fd=None, brs=False, dlm=None, acked=None, post=0, crc_ok=None, sc_ok=None)
        if self.tx_bits is not None:
            self.txi = 1                           # SOF already on the bus

    def step(self, t, lvl):
        fall = self.prev == 1 and lvl == 0
        self.prev = lvl
        if self.state == "idle":
            if lvl == 0 and fall and t - self.idle_since >= 3 * self.tn:
                if self.pending and (self.join_sof or self.tx == 0):
                    self.tx_bits = self._wire()
                    self.tx = 0
                self._hard_sync(t)
            elif lvl == 1:
                if self.pending and not self.join_sof and t - self.idle_since >= 11 * self.tn:
                    self.tx = 0                    # our SOF; hard sync on the edge next cycle
            else:
                self.idle_since = t
            return
        if fall and t != self.bs:                  # resync on recessive-to-dominant edges
            if not self.sampled:
                self.bs, self.next_bs = t, t + self.bl
            else:
                self.next_bs = t
        if t == self.next_bs:
            self.bs, self.next_bs, self.sampled = t, t + self.bl, False
            self._drive_next()
        if not self.sampled and t - self.bs == self.sp:
            self.sampled = True
            self._sample(t, lvl)

    def _set_rate(self, t, data):
        self.bl, self.sp = (self.td, self.spd) if data else (self.tn, self.spn)
        self.next_bs = t + (self.bl - self.sp)

    def _drive_next(self):
        if getattr(self, "ack_drive", False):
            self.tx, self.ack_drive = 0, False
            return
        if self.tx_bits is not None and self.txi < len(self.tx_bits):
            self.tx = self.tx_bits[self.txi][0]
            self.txi += 1
        else:
            self.tx = 1

    # ---- receiving -----------------------------------------------------
    def _err(self, msg):
        if not self.errors or self.errors[-1] != msg:
            self.errors.append(msg)

    def _sample(self, t, lvl):
        r = self.rx
        sending = self.tx_bits is not None
        if sending and self.txi <= len(self.tx_bits) and self.txi > 0:
            kind = self.tx_bits[self.txi - 1][1]
            if kind == "arb" and self.tx == 1 and lvl == 0:
                self.tx_bits, self.tx = None, 1    # lost arbitration: receiver from now on
                sending = False
        if r["mode"] == "dyn":
            if r["run"] == 5:
                if lvl == r["last"]:
                    self._err("stuff error")
                r["run"], r["last"] = 1, lvl
                if r["fd"] is not False:
                    r["crcbits"].append(lvl)
                    r["dsc"] += 1
                return
            if r.get("cend"):                       # classic: CRC (and its stuff bit) done
                r["mode"] = "post"
                self._post_bit(t, lvl)
                return
            r["run"] = r["run"] + 1 if lvl == r["last"] else 1
            r["last"] = lvl
            r["crcbits"].append(lvl)
            self._data_bit(t, lvl)
        elif r["mode"] == "fix":
            if r["fsb"]:
                if lvl == r["last"]:
                    self._err("fixed stuff bit error")
                r["fsb"], r["last"] = False, lvl
                return
            r["last"] = lvl
            r["tail"].append(lvl)
            if len(r["tail"]) % 4 == 0 and len(r["tail"]) < r["tlen"]:
                r["fsb"] = True
            if len(r["tail"]) == r["tlen"]:
                r["mode"] = "post"
        else:
            self._post_bit(t, lvl)

    def _hdr(self):
        b = self.rx["bits"]
        return 1 if len(b) >= 14 and b[13] else 0

    def _data_bit(self, t, lvl):
        r = self.rx
        b = r["bits"]
        b.append(lvl)
        n, ide = len(b), self._hdr()
        fdf_i = 33 if ide else 14
        if n == fdf_i + 1:
            r["fd"] = bool(lvl)
        if r["fd"] and n == fdf_i + 3 and lvl:          # BRS recessive: data bit rate at this sample point
            r["brs"] = True
            self._set_rate(t, True)
        hdr = fdf_i + 8 if r["fd"] else (39 if ide else 19)
        if n == hdr:
            dlc = int("".join(map(str, b[hdr - 4:hdr])), 2)
            r["nbytes"] = DLC_LEN[dlc] if r["fd"] else min(dlc, 8)
            r["hdr"] = hdr
        if "nbytes" in r and n == r["hdr"] + 8 * r["nbytes"]:
            if r["fd"]:
                r["mode"], r["fsb"] = "fix", True
                r["tlen"] = 4 + (17 if r["nbytes"] <= 16 else 21)
        if not r["fd"] and "nbytes" in r and n == r["hdr"] + 8 * r["nbytes"] + 15:
            r["cend"] = True

    def _post_bit(self, t, lvl):
        r = self.rx
        r["post"] += 1
        k = r["post"]
        if k == 1:                                      # CRC delimiter
            if not lvl:
                self._err("CRC delimiter dominant")
            if r["fd"]:
                dyn = r["crcbits"]
                sc, cr = r["tail"][:4], r["tail"][4:]
                r["sc_ok"] = sc == gray_sc(r["dsc"])
                w = 17 if r["nbytes"] <= 16 else 21
                c = int("".join(map(str, cr)), 2)
                r["crc_ok"] = r["sc_ok"] and c == (crc17 if w == 17 else crc21)(dyn + sc)
                if r["brs"]:
                    self._set_rate(t, False)
            else:
                r["crc_ok"] = crc15(r["bits"]) == 0
            if r["crc_ok"] and self.tx_bits is None:
                self.ack_drive = True
        elif k == 2:
            r["acked"] = lvl == 0
        elif k == 3:
            if not lvl:
                self._err("ACK delimiter dominant")
        elif k == 3 + 7:
            b, hdr = r["bits"], r["hdr"]
            fid = int("".join(map(str, b[1:12])), 2)
            if self._hdr():
                fid = EFF | (fid << 18) | int("".join(map(str, b[14:32])), 2)
            data = [int("".join(map(str, b[hdr + 8 * i: hdr + 8 + 8 * i])), 2) for i in range(r["nbytes"])]
            by = self.name if self.tx_bits is not None else "other"
            self.frames.append(dict(id=fid, data=data, fd=bool(r["fd"]), brs=r["brs"], crc_ok=r["crc_ok"],
                                    sc_ok=r["sc_ok"], dsc=r["dsc"],
                                    acked=r["acked"], by=by))
            if by == self.name:
                self.pending = None
            self.tx_bits = None
            self.state = "idle"
            self.idle_since = t
