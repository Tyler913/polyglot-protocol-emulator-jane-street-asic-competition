/*
 * Copyright (c) 2026 Tyler Hong
 * SPDX-License-Identifier: Apache-2.0
 *
 * bench: how much computing the CPU does between protocol work. Each kernel
 * runs on a 4 KB buffer (filled from a host seed) and its cycle count goes to
 * the host, with a result the test recomputes in Python:
 *
 *   [8]  CRC-32 (IEEE), bitwise            [9]  its CRC
 *   [10] CRC-32 with a 1 KB table           [11] its CRC (table built first, not timed)
 *   [12] SHA-256 of the 4 KB                [13..20] the digest (big-endian words)
 *   [21] word copy 4 KB                     [22] checksum of the copy
 *   [23] 1000 x 32-bit multiply (libgcc)    [24] sum of the products
 *   [25] 1000 x 32-bit divide (libgcc)      [26] sum of the quotients
 *   [27] 1000 x 16-bit shift-add multiply   (no M extension: what C code can do)
 *
 * Host parameters: [0] seed. [1] = 0x600DBE7C when done.
 * Used by test/test_bench.py. Uses 4 KB buffer + 4 KB copy + 1 KB table: a
 * program like this needs more than the 8 KB SRAM of earlier layouts.
 */

#include "protocol.h"

#define N 4096

static uint8_t  buf[N];
static uint32_t dst[N / 4];
static uint32_t table[256];

static uint32_t crc32_bitwise(const uint8_t *p, uint32_t n)
{
    uint32_t c = 0xFFFFFFFFu;
    while (n--) {
        c ^= *p++;
        for (int k = 0; k < 8; k++) c = (c >> 1) ^ (0xEDB88320u & (0u - (c & 1u)));
    }
    return ~c;
}

static uint32_t crc32_table(const uint8_t *p, uint32_t n)
{
    uint32_t c = 0xFFFFFFFFu;
    while (n--) c = table[(c ^ *p++) & 0xFFu] ^ (c >> 8);
    return ~c;
}

/* ---- SHA-256 (FIPS 180-4) ---------------------------------------------- */

static const uint32_t K[64] = {
    0x428a2f98, 0x71374491, 0xb5c0fbcf, 0xe9b5dba5, 0x3956c25b, 0x59f111f1, 0x923f82a4, 0xab1c5ed5,
    0xd807aa98, 0x12835b01, 0x243185be, 0x550c7dc3, 0x72be5d74, 0x80deb1fe, 0x9bdc06a7, 0xc19bf174,
    0xe49b69c1, 0xefbe4786, 0x0fc19dc6, 0x240ca1cc, 0x2de92c6f, 0x4a7484aa, 0x5cb0a9dc, 0x76f988da,
    0x983e5152, 0xa831c66d, 0xb00327c8, 0xbf597fc7, 0xc6e00bf3, 0xd5a79147, 0x06ca6351, 0x14292967,
    0x27b70a85, 0x2e1b2138, 0x4d2c6dfc, 0x53380d13, 0x650a7354, 0x766a0abb, 0x81c2c92e, 0x92722c85,
    0xa2bfe8a1, 0xa81a664b, 0xc24b8b70, 0xc76c51a3, 0xd192e819, 0xd6990624, 0xf40e3585, 0x106aa070,
    0x19a4c116, 0x1e376c08, 0x2748774c, 0x34b0bcb5, 0x391c0cb3, 0x4ed8aa4a, 0x5b9cca4f, 0x682e6ff3,
    0x748f82ee, 0x78a5636f, 0x84c87814, 0x8cc70208, 0x90befffa, 0xa4506ceb, 0xbef9a3f7, 0xc67178f2};

#define ROR(x, n) (((x) >> (n)) | ((x) << (32 - (n))))

static void sha256_block(uint32_t h[8], const uint8_t *p)
{
    static uint32_t w[64];
    for (int i = 0; i < 16; i++)
        w[i] = ((uint32_t)p[4 * i] << 24) | ((uint32_t)p[4 * i + 1] << 16) | ((uint32_t)p[4 * i + 2] << 8) | p[4 * i + 3];
    for (int i = 16; i < 64; i++) {
        uint32_t s0 = ROR(w[i - 15], 7) ^ ROR(w[i - 15], 18) ^ (w[i - 15] >> 3);
        uint32_t s1 = ROR(w[i - 2], 17) ^ ROR(w[i - 2], 19) ^ (w[i - 2] >> 10);
        w[i] = w[i - 16] + s0 + w[i - 7] + s1;
    }
    uint32_t a = h[0], b = h[1], c = h[2], d = h[3], e = h[4], f = h[5], g = h[6], hh = h[7];
    for (int i = 0; i < 64; i++) {
        uint32_t t1 = hh + (ROR(e, 6) ^ ROR(e, 11) ^ ROR(e, 25)) + ((e & f) ^ (~e & g)) + K[i] + w[i];
        uint32_t t2 = (ROR(a, 2) ^ ROR(a, 13) ^ ROR(a, 22)) + ((a & b) ^ (a & c) ^ (b & c));
        hh = g; g = f; f = e; e = d + t1; d = c; c = b; b = a; a = t1 + t2;
    }
    h[0] += a; h[1] += b; h[2] += c; h[3] += d; h[4] += e; h[5] += f; h[6] += g; h[7] += hh;
}

static void sha256(const uint8_t *p, uint32_t n, uint32_t h[8])   /* n a multiple of 64 */
{
    static const uint32_t h0[8] = {0x6a09e667, 0xbb67ae85, 0x3c6ef372, 0xa54ff53a,
                                   0x510e527f, 0x9b05688c, 0x1f83d9ab, 0x5be0cd19};
    static uint8_t last[64];
    for (int i = 0; i < 8; i++) h[i] = h0[i];
    for (uint32_t i = 0; i < n; i += 64) sha256_block(h, p + i);
    for (int i = 0; i < 64; i++) last[i] = 0;
    last[0] = 0x80;
    uint32_t bits = n * 8u;
    last[60] = (uint8_t)(bits >> 24); last[61] = (uint8_t)(bits >> 16);
    last[62] = (uint8_t)(bits >> 8);  last[63] = (uint8_t)bits;
    sha256_block(h, last);
}

/* ---- main --------------------------------------------------------------- */

static uint32_t mul16(uint32_t a, uint32_t b)       /* shift-add, 16 x 16 bits */
{
    uint32_t r = 0;
    for (int k = 0; k < 16; k++) { if (b & 1u) r += a; a <<= 1; b >>= 1; }
    return r;
}

int main(void)
{
    uint32_t x = PG_SHARED[0] | 1u, t, h[8];
    volatile uint32_t *R = PG_SHARED;
    for (int i = 1; i < 32; i++) R[i] = 0;

    for (uint32_t i = 0; i < N; i++) {                 /* xorshift fill */
        x ^= x << 13; x ^= x >> 17; x ^= x << 5;
        buf[i] = (uint8_t)x;
    }
    for (uint32_t i = 0; i < 256; i++) {
        uint32_t c = i;
        for (int k = 0; k < 8; k++) c = (c >> 1) ^ (0xEDB88320u & (0u - (c & 1u)));
        table[i] = c;
    }

    t = now(); R[9]  = crc32_bitwise(buf, N); R[8]  = now() - t;
    t = now(); R[11] = crc32_table(buf, N);   R[10] = now() - t;
    t = now(); sha256(buf, N, h);             R[12] = now() - t;
    for (int i = 0; i < 8; i++) R[13 + i] = h[i];

    t = now();
    const uint32_t *s = (const uint32_t *)buf;
    for (uint32_t i = 0; i < N / 4; i++) dst[i] = s[i];
    R[21] = now() - t;
    uint32_t sum = 0;
    for (uint32_t i = 0; i < N / 4; i++) sum = (sum << 1 | sum >> 31) ^ dst[i];
    R[22] = sum;

    volatile uint32_t a = 0x12345679u, b = 1000003u;   /* volatile: real calls each time */
    t = now(); sum = 0;
    for (uint32_t i = 0; i < 1000; i++) sum += a * (b + i);
    R[23] = now() - t; R[24] = sum;
    t = now(); sum = 0;
    for (uint32_t i = 0; i < 1000; i++) sum += a / (b + i);
    R[25] = now() - t; R[26] = sum;
    t = now(); sum = 0;
    for (uint32_t i = 0; i < 1000; i++) sum += mul16(i, 40503u);
    R[27] = now() - t; R[28] = sum;

    R[1] = 0x600DBE7Cu;
    host_irq(1);
    pg_park();
}
