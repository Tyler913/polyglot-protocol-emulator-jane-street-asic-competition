/*
 * Copyright (c) 2026 Tyler Hong
 * SPDX-License-Identifier: Apache-2.0
 *
 * usb_hid_fs: the same HID keyboard as examples/usb_hid, at USB full speed
 * (12 Mbit/s). Only the bit period and which line idles high change; the
 * engine's NRZI, stuffing, CRC16, EOP and timed handshake work as at low
 * speed. Built for a 48 MHz clock (4 cycles per bit); 60 MHz also works
 * (5 cycles per bit) with PG_F_CPU set to it.
 * Pins: D- uio[4] (P4), D+ uio[5] (P5), 1.5 kOhm pull-up on D+.
 */

#ifndef PG_F_CPU
#define PG_F_CPU 48000000u
#endif
#define USB_FULL_SPEED 1
#include "../usb_hid/main.c"
