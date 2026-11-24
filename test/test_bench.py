# Copyright (c) 2026 Tyler Hong
# SPDX-License-Identifier: Apache-2.0
"""CPU compute benchmark (sw/tests/bench): CRC-32 two ways, SHA-256, word
copy and libgcc multiply/divide on a 4 KB buffer. Every result is recomputed
here; the cycle counts are logged as one "BENCH {...}" line with derived
rates at 50 MHz. The program uses about 11 KB, so it needs more than the 8 KB SRAM of earlier layouts."""

import binascii
import hashlib
import json

import cocotb

from lib.soc import F_CPU, load, read_shared, start, wait_irq

N = 4096
SEED = 0x2468ACE1


def fill(seed):
    x, out = seed | 1, bytearray()
    for _ in range(N):
        x ^= (x << 13) & 0xFFFFFFFF
        x ^= x >> 17
        x ^= (x << 5) & 0xFFFFFFFF
        out.append(x & 0xFF)
    return bytes(out)


def rotl1_xor(words):
    s = 0
    for w in words:
        s = (((s << 1) | (s >> 31)) & 0xFFFFFFFF) ^ w
    return s


@cocotb.test()
async def test_bench(dut):
    host = await start(dut)
    await load(host, "bench", [SEED])
    await host.ctrl(0x01)
    await wait_irq(dut, 1, timeout=3_000_000)
    r = await read_shared(host, 0, 32)
    assert r[1] == 0x600DBE7C, f"done marker {r[1]:#x}"

    data = fill(SEED)
    crc = binascii.crc32(data) & 0xFFFFFFFF
    assert r[9] == crc and r[11] == crc, (hex(r[9]), hex(r[11]), hex(crc))
    digest = hashlib.sha256(data).digest()
    assert b"".join(w.to_bytes(4, "big") for w in r[13:21]) == digest, "SHA-256"
    assert r[22] == rotl1_xor(int.from_bytes(data[i:i + 4], "little") for i in range(0, N, 4)), "copy"
    a, b = 0x12345679, 1000003
    assert r[24] == sum(a * (b + i) for i in range(1000)) & 0xFFFFFFFF, "multiply"
    assert r[26] == sum(a // (b + i) for i in range(1000)) & 0xFFFFFFFF, "divide"
    assert r[28] == sum(i * 40503 for i in range(1000)) & 0xFFFFFFFF, "shift-add"

    def rate(cycles):                       # kB/s for N bytes at F_CPU
        return round(N / (cycles / F_CPU) / 1000)

    out = dict(crc32_bitwise=dict(cycles=r[8], per_byte=round(r[8] / N, 1), kB_s=rate(r[8])),
               crc32_table=dict(cycles=r[10], per_byte=round(r[10] / N, 1), kB_s=rate(r[10])),
               sha256=dict(cycles=r[12], per_byte=round(r[12] / N, 1), kB_s=rate(r[12])),
               copy=dict(cycles=r[21], per_word=round(r[21] / (N / 4), 2), MB_s=round(rate(r[21]) / 1000, 1)),
               mul32_libgcc=dict(per_op=round(r[23] / 1000, 1)),
               div32_libgcc=dict(per_op=round(r[25] / 1000, 1)),
               mul16_shift_add=dict(per_op=round(r[27] / 1000, 1)))
    dut._log.info("BENCH " + json.dumps(out))
