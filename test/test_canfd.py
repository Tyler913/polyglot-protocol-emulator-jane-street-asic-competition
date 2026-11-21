# Copyright (c) 2026 Tyler Hong
# SPDX-License-Identifier: Apache-2.0
"""CAN FD (ISO 11898-1:2015) on Wire Engine 0, judged by an independent CAN FD
node model (test/lib/canfd.py) that has its own sample points (80 % nominal,
75 % data), resyncs on edges and checks dynamic and fixed stuffing, the stuff
count, CRC-17/21 and the bit rate switch.

- The chip sends FD frames (0 to 64 bytes, base and extended IDs, with and
  without BRS) and one classic frame (sw/tests/canfd); the model must decode
  and ACK every one, and the chip's own RX must decode them back.
- The model sends frames to the chip (sw/tests/canfd_rx); the chip must
  report them and ACK exactly the good ones (one FD frame has a bad CRC).
Bit rates: 500 kbit/s / 2 Mbit/s and 1 Mbit/s / 5 Mbit/s at 50 MHz."""

import cocotb
from cocotb.triggers import ClockCycles

from lib.can import EFF, CanBus
from lib.canfd import CanFdNode, len_to_dlc
from lib.soc import F_CPU, load, read_shared, start, wait_irq

DATA = [(3 * i + 1) & 0xFF for i in range(64)]
SENT = [  # what sw/tests/canfd sends: id, data, fd, brs
    (0x123, DATA[:8], True, True),
    (0x1ABCDEF | EFF, DATA[:64], True, True),
    (0x7FF, [], True, True),
    (0x055, DATA[40:60], True, False),
    (0x0F0, DATA[:3], False, False),
    (0x400, [0] * 12, True, True),
    (0x0AA, DATA[5:53], True, True),
]


async def chip_sends(dut, nominal, data):
    host = await start(dut)
    tn, td = F_CPU / nominal, F_CPU / data
    bus = CanBus(dut, txd_uio=0, rxd_ui=2, bit_cycles=tn)
    node = bus.add(CanFdNode(bus, "node", round(tn), round(td)))
    bus.start()
    await load(host, "canfd", [nominal, data])
    await host.ctrl(0x01)
    await wait_irq(dut, 1, timeout=10_000)
    try:
        await wait_irq(dut, 0, timeout=400_000)
    except AssertionError:
        dut._log.info(f"model frames {[(hex(f['id']), len(f['data']), f['crc_ok'], f['acked']) for f in node.frames]}, errors {node.errors[:5]}")
        raise
    await ClockCycles(dut.clk, 100)
    bus.stop()
    res = await read_shared(host, 0, 9)
    dut._log.info(f"model frames {[(hex(f['id']), len(f['data']), f['fd'], f['brs'], f['crc_ok'], f['sc_ok'], f['dsc']) for f in node.frames]}")
    dut._log.info(f"chip results {[hex(x) for x in res]}")
    assert not node.errors, node.errors[:5]
    assert res[0] == 0x600D00FD, f"done {res[0]:#x}"
    got = [(f["id"], f["data"], f["fd"], f["brs"]) for f in node.frames]
    assert got == SENT, f"model decoded {got}"
    assert all(f["crc_ok"] and f["acked"] and f["by"] == "other" for f in node.frames), node.frames
    for k, (fid, d, fd, brs) in enumerate(SENT):
        r = res[2 + k] & 0xF1F00                    # FDF, BRS, delimiter, CRC ok, SC ok, DLC
        exp = (0x1D00 | (brs << 9) | (len_to_dlc(len(d)) << 16)) if fd else (0xC00 | (len(d) << 16))
        assert r == exp, f"chip's own RX on frame {k}: FDCTL {r:#x}, expected {exp:#x}"


@cocotb.test()
async def test_canfd_send_500k_2m(dut):
    await chip_sends(dut, 500_000, 2_000_000)


@cocotb.test()
async def test_canfd_send_1m_5m(dut):
    await chip_sends(dut, 1_000_000, 5_000_000)


async def chip_receives(dut, nominal, data):
    host = await start(dut)
    tn, td = F_CPU / nominal, F_CPU / data
    bus = CanBus(dut, txd_uio=0, rxd_ui=2, bit_cycles=tn)
    node = bus.add(CanFdNode(bus, "node", round(tn), round(td), 0.75, 0.7))
    bus.start()
    frames = [  # id, data, fd, brs, bad CRC
        (0x101, DATA[:8], True, True, False),
        (0x1234567 | EFF, DATA[:64], True, True, False),
        (0x7F0, [], True, True, False),
        (0x2A5, [0xFF] * 20, True, False, False),
        (0x321, [0xC4, 0x3B, 0x00], False, False, False),
        (0x0A0, DATA[10:22], True, True, True),          # bad CRC: no ACK
        (0x0A1, [0] * 16, True, True, False),             # CRC-17 with long runs
        (0x00F | EFF, DATA[1:25], True, True, False),     # CRC-21
    ]
    await load(host, "canfd_rx", [len(frames), nominal, data])
    await host.ctrl(0x01)
    await wait_irq(dut, 1, timeout=10_000)
    for fid, d, fd, brs, bad in frames:
        node.send(fid, d, fd=fd, brs=brs, bad_crc=bad)
        for _ in range(400):
            await ClockCycles(dut.clk, 500)
            if node.pending is None:
                break
        else:
            raise AssertionError(f"node never finished frame {fid:#x}")
    await ClockCycles(dut.clk, 2000)
    bus.stop()
    res = await read_shared(host, 0, 3 + 4 * len(frames))
    dut._log.info(f"model frames {[(hex(f['id']), f['acked']) for f in node.frames]}")
    dut._log.info(f"chip results {[hex(x) for x in res]}")
    assert not node.errors, node.errors[:5]
    acks = [f["acked"] for f in node.frames]
    assert acks == [not f[4] for f in frames], f"ACKs {acks}"
    assert res[1] == len(frames), f"chip received {res[1]} frames"
    for k, (fid, d, fd, brs, bad) in enumerate(frames):
        r = res[3 + 4 * k: 7 + 4 * k]
        assert r[0] == fid, f"frame {k}: id {r[0]:#x}"
        assert r[1] == len(d) | (fd << 8) | ((brs and fd) << 9) | ((not bad) << 10), f"frame {k}: {r[1]:#x}"
        ck = 0
        for x in d:
            ck = (((ck << 1) | (ck >> 31)) & 0xFFFFFFFF) ^ x
        assert r[2] == ck, f"frame {k}: data checksum"


@cocotb.test()
async def test_canfd_receive_500k_2m(dut):
    await chip_receives(dut, 500_000, 2_000_000)


@cocotb.test()
async def test_canfd_receive_1m_5m(dut):
    await chip_receives(dut, 1_000_000, 5_000_000)


@cocotb.test()
async def test_canfd_receive_back_to_back_dma(dut):
    """Frames back to back (3-bit intermission) at 1 + 5 Mbit/s, received
    through a DMA ring while C spends 100 us on other work after each frame:
    the 4-word RX FIFO alone would overflow (checked by the same test without
    the DMA, which loses frames)."""
    host = await start(dut)
    nominal, data = 1_000_000, 5_000_000
    tn, td = F_CPU / nominal, F_CPU / data
    bus = CanBus(dut, txd_uio=0, rxd_ui=2, bit_cycles=tn)
    node = bus.add(CanFdNode(bus, "node", round(tn), round(td), 0.75, 0.7))
    bus.start()
    frames = []
    for k in range(4):
        frames.append(dict(fid=0x100 + k, data=DATA[k:] + DATA[:k]))
        frames.append(dict(fid=0x200 + k, data=[]))
        frames.append(dict(fid=0x300 + k, data=[k] * 3, fd=False, brs=False))
    await load(host, "canfd_rx", [12, nominal, data, 1, 100])
    await host.ctrl(0x01)
    await wait_irq(dut, 1, timeout=10_000)
    node.send_all(frames)
    await wait_irq(dut, 0, timeout=400_000)
    bus.stop()
    res = await read_shared(host, 0, 3 + 4 * 12)
    assert not node.errors, node.errors[:5]
    assert [f["acked"] for f in node.frames] == [True] * 12, [f["acked"] for f in node.frames]
    assert res[0] == 0x600D00FE and res[1] == 12, f"done {res[0]:#x}, {res[1]} frames"
    for k, f in enumerate(frames):
        r = res[3 + 4 * k: 7 + 4 * k]
        d = f["data"]
        fd = f.get("fd", True)
        assert r[0] == f["fid"], f"frame {k}: id {r[0]:#x}"
        assert r[1] == len(d) | (fd << 8) | (fd << 9) | (1 << 10), f"frame {k}: {r[1]:#x}"
