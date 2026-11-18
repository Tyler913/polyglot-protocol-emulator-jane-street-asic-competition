/*
 * Copyright (c) 2026 Tyler Hong
 * SPDX-License-Identifier: Apache-2.0
 *
 * mdio: an IEEE 802.3 clause 22 management station (MDC/MDIO), the bus
 * that configures Ethernet PHYs. A frame is 32 preamble ones, start 01,
 * opcode (10 read, 01 write), 5-bit PHY address, 5-bit register, a 2-bit
 * turnaround and 16 data bits, MSB first, sampled on the rising edge of
 * MDC. On a lite engine (MDC = the internal bit clock, SPI mode 0) a write
 * is two pushes; a read sends the first 14 bits, then turns its driver off
 * and clocks 18 more bits with RX on: the turnaround (released, then 0 from
 * the PHY) and the data. MDC may pause between the two (it is aperiodic).
 *
 * Pins: MDIO uio[0] (P0, with a pull-up), MDC uio[1] (P1); 2.5 MHz.
 * Reads the PHY identifier (registers 2, 3), writes BMCR (register 0),
 * reads it back, then reads from an address nobody answers.
 * Results (PG_SHARED): [0] ID1, [1] ID2, [2] BMCR read back, [3] the read
 * from the empty address (0xFFFF), [4] 0x600D3D10.
 */

#include "protocol.h"

#define MD   W2
#define MDIO PIN_UIO(0)
#define MDC  PIN_UIO(1)

static const uint32_t base = WIRE_EN | WIRE_TX | WIRE_MSB_FIRST | WIRE_IDLE_HIGH | WIRE_CLK_OUT | WIRE_LEN(32);

static void mdio_write(uint32_t phy, uint32_t reg, uint32_t v)
{
    wire_putn(MD, 32, 0xFFFFFFFFu);                                      /* preamble */
    wire_putn(MD, 32, (0x5u << 28) | (phy << 23) | (reg << 18) | (0x2u << 16) | (v & 0xFFFFu));
    wire_flush(MD);
}

/* Returns the 16 data bits, or 0x10000 | data if the PHY did not pull the turnaround low */
static uint32_t mdio_read(uint32_t phy, uint32_t reg)
{
    wire_putn(MD, 32, 0xFFFFFFFFu);
    wire_putn(MD, 14, (0x6u << 10) | (phy << 5) | reg);                  /* start 01, read 10 */
    wire_flush(MD);
    MD->CTRL = base | WIRE_OE_NEVER | WIRE_RX;                           /* release MDIO, listen */
    wire_putn(MD, 18, 0x3FFFFu);                                         /* clocks: turnaround + data */
    uint32_t x = wire_get(MD);
    MD->CTRL = base;                                                     /* drive (idle 1) again */
    return (x & 0xFFFFu) | ((x & 0x10000u) ? 0x10000u : 0u);
}

int main(void)
{
    MD->CTRL = 0;
    MD->DIV  = PG_BIT_TICKS(2500000);
    MD->PINS = WIRE_PINS(MDIO, MDIO, MDC);
    MD->CTRL = base;
    PG_SHARED[0] = mdio_read(1, 2);
    PG_SHARED[1] = mdio_read(1, 3);
    mdio_write(1, 0, 0x1200);                     /* restart auto-negotiation, enable it */
    PG_SHARED[2] = mdio_read(1, 0);
    PG_SHARED[3] = mdio_read(7, 2) & 0xFFFFu;
    PG_SHARED[4] = 0x600D3D10u;
    host_irq(1);
    pg_park();
}
