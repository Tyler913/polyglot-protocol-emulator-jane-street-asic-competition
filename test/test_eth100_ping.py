# Copyright (c) 2026 Tyler Hong
# SPDX-License-Identifier: Apache-2.0
"""A 100 Mbit/s Ethernet node in plain C (sw/examples/eth100_ping) on the
experimental RMII port (RTL built with make RMII=yes): it answers an ARP
request and ICMP echo requests (32 and 1000 bytes of payload) with replies a
host stack would accept. Frames to the chip and the replies go through the
independent RMII PHY model (lib/rmii.py: preamble, SFD, FCS and inter-frame
gap checked); the replies are checked field by field against RFC 826 (ARP),
RFC 791 (IPv4 header checksum) and RFC 792 (ICMP)."""

import struct

import cocotb
from cocotb.triggers import ClockCycles

from lib.rmii import RmiiPhy
from lib.soc import load, read_shared, start
from test_eth_ping import CHIP_IP, CHIP_MAC, HOST_IP, HOST_MAC, arp_request, inet_csum, ping_request


@cocotb.test()
async def test_eth100_arp_ping(dut):
    host = await start(dut)
    phy = RmiiPhy(dut)
    phy.start()
    await load(host, "eth100_ping")
    await host.ctrl(0x01)
    await ClockCycles(dut.clk, 3000)  # start-up (crt0 clears the frame buffer)

    small, big = bytes(range(32)), bytes((7 * i) & 0xFF for i in range(1000))
    requests = [arp_request(bytes([10, 0, 0, 9])),   # not for us: no answer
                arp_request(CHIP_IP),
                ping_request(0x4242, 1, small),
                ping_request(0x4242, 2, big)]
    for r in requests:
        phy.queue.append((r, False, 100))
        while phy.queue:
            await ClockCycles(dut.clk, 100)
        await ClockCycles(dut.clk, 40_000)  # the reply goes out before the next request
    phy.stop()

    assert not phy.errors, phy.errors[:5]
    stats = await read_shared(host, 0, 4)
    assert stats == [4, 0, 1, 2], f"received/bad/arp/echo = {stats}"
    assert len(phy.frames) == 3, f"{len(phy.frames)} replies"

    # ARP reply (RFC 826)
    a = phy.frames[0]
    assert a[0:6] == HOST_MAC and a[6:12] == CHIP_MAC and a[12:14] == b"\x08\x06"
    htype, ptype, hlen, plen, op, sha, spa, tha, tpa = struct.unpack("!HHBBH6s4s6s4s", a[14:42])
    assert (htype, ptype, hlen, plen, op) == (1, 0x0800, 6, 4, 2)
    assert (sha, spa, tha, tpa) == (CHIP_MAC, CHIP_IP, HOST_MAC, HOST_IP)

    # ICMP echo replies (RFC 791, 792)
    for e, seq, pl in zip(phy.frames[1:], (1, 2), (small, big)):
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
