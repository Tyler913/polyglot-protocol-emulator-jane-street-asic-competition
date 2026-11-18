/*
 * Copyright (c) 2026 Tyler Hong
 * SPDX-License-Identifier: Apache-2.0
 *
 * sd_spi: an SD card in SPI mode (SD Physical Layer Simplified Spec, part 7):
 * power-up clocks, CMD0, CMD8, ACMD41 until ready, CMD58 (OCR), then CMD17
 * reads a 512-byte block. Each command is 48 bits: 01, index, argument,
 * CRC7, end bit. The engine's CRC unit (7 bits, x^7 + x^3 + 1) computes the
 * CRC7 over the first 40 bits and sends it right after them (TXATTR
 * CRC_INIT / CRC_OUT); the C code only pushes the index and the argument.
 * The data block's CRC16 is checked in C.
 *
 * Pins: SCK uo[2] (P10), MOSI uo[3] (P11), CS uo[4] (P12), MISO ui[1] (P17).
 * 400 kHz during initialization, then 12.5 MHz (F/4, MISO sampled 3 cycles late).
 * Results (PG_SHARED): [0] init ok, [1] OCR, [2] block CRC ok, [3..6] first
 * 16 bytes of block 7, [7] 0x600D05D0.
 */

#include "protocol.h"

#define SD  W0
#define SCK PIN_UO(2)
#define MOSI PIN_UO(3)
#define CS  PIN_UO(4)
#define MISO PIN_UI(1)

static uint8_t blk[512] __attribute__((aligned(4)));

static uint8_t xfer(uint8_t b) { return spi_xfer(SD, b); }

static uint8_t sd_cmd(uint32_t idx, uint32_t arg)
{
    xfer(0xFF);                                  /* one byte of gap before the command */
    SD->TXATTR = WIRE_A_CRC_INIT;
    wire_putn(SD, 8, 0x40u | idx);
    SD->TXATTR = WIRE_A_CRC_OUT;                 /* CRC7 follows the argument */
    wire_putn(SD, 32, arg);
    SD->TXATTR = WIRE_A_NOCRC;
    wire_putn(SD, 1, 1u);                        /* end bit */
    SD->TXATTR = 0;
    for (int i = 0; i < 4; i++) (void)SD->RX;    /* 8 + 32 + 7 + 1 bits read back */
    uint8_t r = 0xFF;
    for (int i = 0; i < 9 && (r & 0x80u); i++) r = xfer(0xFF);   /* NCR: 1..8 bytes */
    return r;
}

static uint32_t get32(void)
{
    uint32_t v = 0;
    for (int i = 0; i < 4; i++) v = (v << 8) | xfer(0xFF);
    return v;
}

static uint16_t crc16(const uint8_t *p, int n)   /* CRC-16-CCITT, init 0 */
{
    uint16_t c = 0;
    while (n--) {
        c ^= (uint16_t)(*p++ << 8);
        for (int i = 0; i < 8; i++) c = (c & 0x8000u) ? (uint16_t)((c << 1) ^ 0x1021u) : (uint16_t)(c << 1);
    }
    return c;
}

int main(void)
{
    pin_write(CS, 1);
    pin_output(CS);
    wire_config(SD, WIRE_SPI_CTRL(SCK, MOSI, MISO, 0, 400000));
    wire_crc(SD, 0x09u, 7, 0u, 0, 0);
    for (int i = 0; i < 10; i++) xfer(0xFF);     /* >= 74 clocks with CS and MOSI high */

    spi_select(CS);
    int ok = sd_cmd(0, 0) == 0x01u;              /* GO_IDLE_STATE */
    ok &= sd_cmd(8, 0x1AAu) == 0x01u;            /* SEND_IF_COND: 2.7-3.6 V, check pattern */
    ok &= get32() == 0x1AAu;
    uint8_t r = 0x01;
    for (int i = 0; i < 20 && r == 0x01u; i++) {
        sd_cmd(55, 0);                           /* APP_CMD */
        r = sd_cmd(41, 1u << 30);                /* SD_SEND_OP_COND, HCS */
    }
    ok &= r == 0x00u;
    ok &= sd_cmd(58, 0) == 0x00u;                /* READ_OCR */
    PG_SHARED[1] = get32();
    PG_SHARED[0] = (uint32_t)ok;

    /* full speed: 12.5 MHz, MISO sampled 3 cycles after the nominal point */
    SD->DIV  = 4;
    SD->CTRL = SD->CTRL | WIRE_RX_DELAY(3);
    PG_SHARED[2] = 0;
    if (sd_cmd(17, 7) == 0x00u) {                /* READ_SINGLE_BLOCK 7 (block address: SDHC) */
        uint8_t t = 0xFF;
        for (int i = 0; i < 1000 && t == 0xFFu; i++) t = xfer(0xFF);
        if (t == 0xFEu) {                        /* start block token */
            for (int i = 0; i < 512; i++) blk[i] = xfer(0xFF);
            uint16_t c = (uint16_t)(xfer(0xFF) << 8);
            c |= xfer(0xFF);
            PG_SHARED[2] = (uint32_t)(c == crc16(blk, 512));
        }
    }
    spi_deselect(SD, CS);
    xfer(0xFF);
    for (int i = 0; i < 4; i++) PG_SHARED[3 + i] = ((uint32_t *)blk)[i];
    PG_SHARED[7] = 0x600D05D0u;
    host_irq(1);
    pg_park();
}
