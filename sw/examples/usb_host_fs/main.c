/*
 * Copyright (c) 2026 Tyler Hong
 * SPDX-License-Identifier: Apache-2.0
 *
 * usb_host_fs: the USB host of examples/usb_host at full speed (12 Mbit/s,
 * 48 MHz clock: 4 cycles per bit). Besides the bit period and the idle line
 * (the device's pull-up is on D+), full speed replaces the low-speed
 * keep-alive by a start-of-frame token every 1 ms, sent on its exact cycle.
 * Pins: D- uio[4] (P4), D+ uio[5] (P5), 15 kOhm pull-downs.
 */

#ifndef PG_F_CPU
#define PG_F_CPU 48000000u
#endif
#define USB_FULL_SPEED 1
#include "../usb_host/main.c"
