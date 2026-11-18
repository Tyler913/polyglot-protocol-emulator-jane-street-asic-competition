# Copyright (c) 2026 Tyler Hong
# SPDX-License-Identifier: Apache-2.0
"""A 10BASE-T node in plain C (sw/examples/eth_ping) at 60 MHz: it answers an
ARP request and an ICMP echo request (ping) with replies a host stack would
accept. Frames to the chip come from the IEEE 802.3 model (EthRxDriver, with
edge jitter); replies are decoded by the independent transmit decoder
(EthTxMonitor: Manchester, preamble/SFD, FCS) and checked field by field
against RFC 826 (ARP), RFC 791 (IPv4 header checksum) and RFC 792 (ICMP)."""

import struct

import cocotb
from cocotb.triggers import Timer

from lib.eth import EthRxDriver, EthTxMonitor
from lib.soc import load, read_shared, start

CHIP_MAC = bytes.fromhex("020000000002")
CHIP_IP = bytes([10, 0, 0, 2])
HOST_MAC = bytes.fromhex("02aabbccddee")
HOST_IP = bytes([10, 0, 0, 1])


def inet_csum(b):
    if len(b) % 2:
        b += b"\0"
    s = sum(struct.unpack(f"!{len(b) // 2}H", b))
    while s >> 16:
        s = (s & 0xFFFF) + (s >> 16)
    return ~s & 0xFFFF


def pad(f):
    return f.ljust(60, b"\0")


def arp_request(target_ip):
    arp = struct.pack("!HHBBH6s4s6s4s", 1, 0x0800, 6, 4, 1, HOST_MAC, HOST_IP, b"\0" * 6, target_ip)
    return pad(b"\xff" * 6 + HOST_MAC + b"\x08\x06" + arp)


def ping_request(ident, seq, payload):
    icmp = struct.pack("!BBHHH", 8, 0, 0, ident, seq) + payload
    icmp = icmp[:2] + struct.pack("!H", inet_csum(icmp)) + icmp[4:]
    ip = struct.pack("!BBHHHBBH4s4s", 0x45, 0, 20 + len(icmp), 0x1234, 0, 64, 1, 0, HOST_IP, CHIP_IP)
    ip = ip[:10] + struct.pack("!H", inet_csum(ip)) + ip[12:]
    return pad(CHIP_MAC + HOST_MAC + b"\x08\x00" + ip + icmp)


@cocotb.test()
async def test_eth_arp_ping(dut):
    host = await start(dut, clk_ns=16.666)  # 60 MHz
    mon = EthTxMonitor(dut, tdp=6, tdn=7, half=3, tail_bits=2)
    mon.start()
    drv = EthRxDriver(dut, ui_bit=0, jitter_ps=6_000, seed=7)
    await load(host, "eth_ping")
    await host.ctrl(0x01)
    await Timer(50, unit="us")  # start-up (crt0 clears the 600-byte frame buffer)

    payload = bytes(range(32))
    for f in (arp_request(bytes([10, 0, 0, 9])),   # not for us: no answer
              arp_request(CHIP_IP),
              ping_request(0x4242, 1, payload),
              ping_request(0x4242, 2, payload[:20])):
        await drv.send(f)
        await Timer(150, unit="us")  # the reply (about 70 us) goes out before the next request
    mon.stop()

    assert not mon.errors, mon.errors[:5]
    stats = await read_shared(host, 0, 4)
    assert stats == [4, 0, 1, 2], f"received/bad/arp/echo = {stats}"
    assert len(mon.frames) == 3, f"{len(mon.frames)} replies"
    for data, ok in mon.frames:
        assert ok, "reply FCS wrong"

    # ARP reply (RFC 826)
    a = mon.frames[0][0][:-4]
    assert a[0:6] == HOST_MAC and a[6:12] == CHIP_MAC and a[12:14] == b"\x08\x06"
    htype, ptype, hlen, plen, op, sha, spa, tha, tpa = struct.unpack("!HHBBH6s4s6s4s", a[14:42])
    assert (htype, ptype, hlen, plen, op) == (1, 0x0800, 6, 4, 2)
    assert (sha, spa, tha, tpa) == (CHIP_MAC, CHIP_IP, HOST_MAC, HOST_IP)

    # ICMP echo replies (RFC 791, 792)
    for (data, _), seq, pl in zip(mon.frames[1:], (1, 2), (payload, payload[:20])):
        e = data[:-4]
        assert e[0:6] == HOST_MAC and e[6:12] == CHIP_MAC and e[12:14] == b"\x08\x00"
        ip = e[14:34]
        assert inet_csum(ip) == 0, "IPv4 header checksum"
        assert ip[12:16] == CHIP_IP and ip[16:20] == HOST_IP
        tot = struct.unpack("!H", ip[2:4])[0]
        icmp = e[34 : 14 + tot]
        assert inet_csum(icmp) == 0, "ICMP checksum"
        t, code, _, ident, s = struct.unpack("!BBHHH", icmp[:8])
        assert (t, code, ident, s) == (0, 0, 0x4242, seq)
        assert icmp[8:] == pl

