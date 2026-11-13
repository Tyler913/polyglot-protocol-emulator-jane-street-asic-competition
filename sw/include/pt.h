/*
 * Copyright (c) 2026 Tyler Hong
 * SPDX-License-Identifier: Apache-2.0
 *
 * pt.h: cooperative tasks in plain C, in the style of Adam Dunkels'
 * protothreads (switch-based local continuations; this is our own minimal
 * version). A task is a function that returns at every wait; the next call
 * resumes after that wait. Several tasks called round-robin from main() run
 * concurrently, and each needs only one int of state. Locals do not survive
 * a wait: keep anything that must persist in static or task-struct storage.
 * Do not use `switch` across a wait inside a task.
 *
 *   static int blink(pt_t *pt) {
 *       static uint32_t t;
 *       PT_BEGIN(pt);
 *       for (;;) {
 *           pin_toggle(0); t = now() + 1000;
 *           PT_WAIT(pt, time_reached(t));
 *       }
 *       PT_END(pt);
 *   }
 *
 * In a task, use only engine accesses that cannot block: wait for
 * wire_rx_count() > 0 before wire_get(), for !(STATUS & WIRE_TX_ACTIVE)
 * instead of wire_flush(), and keep TX FIFO writes within its free space.
 */

#ifndef POLYGLOT_PT_H
#define POLYGLOT_PT_H

#define PT__FALL __attribute__((fallthrough));

typedef int pt_t;                      /* 0 = start, -1 = finished, else resume line */

#define PT_INIT(pt)        (*(pt) = 0)
#define PT_DONE(pt)        (*(pt) == -1)
#define PT_BEGIN(pt)       switch (*(pt)) { case 0:
#define PT_WAIT(pt, cond)  do { *(pt) = __LINE__; PT__FALL case __LINE__: \
                               if (!(cond)) return 0; } while (0)
#define PT_YIELD(pt)       do { *(pt) = __LINE__; return 0; case __LINE__:; } while (0)
#define PT_END(pt)         PT__FALL default:; } *(pt) = -1; return 1

/* Run a sub-task (its own pt_t) to completion from inside a task. */
#define PT_SPAWN(pt, child, call) do { PT_INIT(child); PT_WAIT(pt, (call) != 0); } while (0)

#endif /* POLYGLOT_PT_H */
