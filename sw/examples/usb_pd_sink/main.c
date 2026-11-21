/*
 * Copyright (c) 2026 Tyler Hong
 * SPDX-License-Identifier: Apache-2.0
 *
 * usb_pd_sink: a USB Power Delivery sink (USB-C) on Wire Engine 1. It waits
 * for the source's Source_Capabilities, answers every message with GoodCRC,
 * requests the highest fixed voltage up to 9 V (at most 2 A), and waits for
 * Accept and PS_RDY. The BMC line code is done by the engine (biphase mark,
 * edge-timed RX), 4b5b and CRC-32 in C. Used by test/test_usb_pd.py.
 *
 * Pins: CC TX uio[2] (P2, to a BMC driver), CC RX ui[4] (P20, from a
 * comparator on the same CC wire).
 * Results (PG_SHARED): [0] 0x600D0BD0 when done (0xBAD0xxxx on a failure),
 * [1] number of source PDOs, [2] chosen object position, [3] its voltage in mV,
 * [4] operating current in mA, [5] message IDs of the three received messages.
 */

#include "protocol.h"

#define PD W1

static void good_crc(uint32_t rx_header)
{
    pd_send(PD, PD_HEADER(PD_CTRL_GOODCRC, 0, 0, PD_HDR_ID(rx_header), 0), 0, 0);
}

static int32_t expect(uint32_t type, uint32_t cnt_min, uint32_t *obj)
{
    for (;;) {
        int32_t h = pd_recv(PD, obj, 50000);
        if (h < 0) return -1;
        good_crc((uint32_t)h);                        /* within 195 us of the EOP */
        if (PD_HDR_TYPE((uint32_t)h) == type && PD_HDR_CNT((uint32_t)h) >= cnt_min) return h;
    }
}

int main(void)
{
    uint32_t obj[7], id = 0, best = 0, best_mv = 0, ids = 0;
    int32_t h;

    for (int i = 0; i < 6; i++) PG_SHARED[i] = 0;
    pd_init(PD, PIN_UIO(2), PIN_UI(4));
    host_irq(1);

    h = expect(PD_DATA_SRC_CAP, 1, obj);
    if (h < 0) goto fail;
    ids = PD_HDR_ID((uint32_t)h);
    uint32_t n = PD_HDR_CNT((uint32_t)h);
    for (uint32_t i = 0; i < n; i++) {               /* fixed supplies: bits 31:30 = 0 */
        uint32_t mv = ((obj[i] >> 10) & 0x3FFu) * 50u;
        if ((obj[i] >> 30) == 0 && mv <= 9000u && mv > best_mv) {
            best = i + 1u;
            best_mv = mv;
        }
    }
    PG_SHARED[1] = n;
    uint32_t ma = (obj[best - 1u] & 0x3FFu) * 10u;
    if (ma > 2000u) ma = 2000u;
    uint32_t rdo = (best << 28) | ((ma / 10u) << 10) | (ma / 10u);
    for (int tries = 0; ; tries++) {                 /* Request until it gets a GoodCRC */
        pd_send(PD, PD_HEADER(PD_DATA_REQUEST, 0, 0, id, 1), &rdo, 1);
        int32_t g = pd_recv(PD, obj, 1000);
        if (g >= 0 && PD_HDR_TYPE((uint32_t)g) == PD_CTRL_GOODCRC && PD_HDR_ID((uint32_t)g) == id) break;
        if (tries == 2) goto fail;
    }
    id++;
    h = expect(PD_CTRL_ACCEPT, 0, obj);
    if (h < 0) goto fail;
    ids |= PD_HDR_ID((uint32_t)h) << 4;
    h = expect(PD_CTRL_PS_RDY, 0, obj);
    if (h < 0) goto fail;
    ids |= PD_HDR_ID((uint32_t)h) << 8;

    PG_SHARED[2] = best;
    PG_SHARED[3] = best_mv;
    PG_SHARED[4] = ma;
    PG_SHARED[5] = ids;
    PG_SHARED[0] = 0x600D0BD0u;
    host_irq(0);
    pg_park();
fail:
    PG_SHARED[0] = 0xBAD00000u;
    host_irq(0);
    pg_park();
}
