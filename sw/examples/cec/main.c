/*
 * Copyright (c) 2026 Tyler Hong
 * SPDX-License-Identifier: Apache-2.0
 *
 * cec: an HDMI-CEC initiator (HDMI 1.4b, CEC section) on one open-drain line.
 * Every bit starts with a falling edge: a start bit is 3.7 ms low out of
 * 4.5 ms, a data bit 0.6 ms (1) or 1.5 ms (0) low out of 2.4 ms. A block is
 * 8 data bits, EOM and an ACK bit; the initiator sends the ACK bit as a 1 and
 * a follower that accepts the block stretches it to a 0, which the initiator
 * reads at 1.05 ms. All edges come from the timed GPIO queue (open drain =
 * output enable switched at exact cycles); the ACK is sampled after
 * wait_until() at its nominal time.
 *
 * Pin: CEC uio[2] (P2), with a pull-up. Logical address 4 (playback 1).
 * Sends <Image View On> to the TV (0), <Set OSD Name> "PG" to the TV, and
 * <Standby> to address 9 (nobody: no ACK).
 * Results (PG_SHARED): [0..2] 1 if every block of message i was ACKed,
 * [3] 0x600D0CEC.
 */

#include "protocol.h"

#define CEC  PIN_UIO(2)
#define MS(x) PG_US_TO_TICKS((x) * 1000u)
#define US(x) PG_US_TO_TICKS(x)

static uint32_t t;                    /* start of the next bit */

static void cec_bit(uint32_t low_us, uint32_t total_us)
{
    pin_oe_at(CEC, 1, t);             /* pull low */
    pin_oe_at(CEC, 0, t + US(low_us));
    t += US(total_us);
}

/* Returns 1 if the block was acknowledged (line low at 1.05 ms of the ACK bit) */
static int cec_block(uint8_t b, int eom, int broadcast)
{
    for (int i = 7; i >= 0; i--) cec_bit(((b >> i) & 1u) ? 600u : 1500u, 2400u);
    cec_bit(eom ? 600u : 1500u, 2400u);
    uint32_t ack_t = t;
    cec_bit(600u, 2400u);             /* ACK bit: we send a 1 */
    wait_until(ack_t + US(1050));
    int low = !pin_read(CEC);
    return broadcast ? !low : low;    /* broadcast: a 0 means a follower rejected it */
}

static int cec_send(uint8_t dst, const uint8_t *d, int n)
{
    int ok = 1;
    timed_flush();
    t = now() + US(100);
    cec_bit(3700u, 4500u);            /* start bit */
    ok &= cec_block((uint8_t)((4u << 4) | dst), n == 0, dst == 15u);
    for (int i = 0; i < n && ok; i++) ok &= cec_block(d[i], i == n - 1, dst == 15u);
    timed_flush();
    delay_ticks(MS(7));               /* signal free time before the next frame */
    return ok;
}

int main(void)
{
    static const uint8_t ivo[1] = { 0x04 };              /* <Image View On> */
    static const uint8_t osd[3] = { 0x47, 'P', 'G' };    /* <Set OSD Name> */
    static const uint8_t stb[1] = { 0x36 };              /* <Standby> */
    pin_write(CEC, 0);                /* open drain: output value 0, the OE does the rest */
    pin_input(CEC);
    delay_ticks(MS(1));
    PG_SHARED[0] = (uint32_t)cec_send(0, ivo, 1);
    PG_SHARED[1] = (uint32_t)cec_send(0, osd, 3);
    PG_SHARED[2] = (uint32_t)cec_send(9, stb, 1);
    PG_SHARED[3] = 0x600D0CECu;
    host_irq(1);
    pg_park();
}
