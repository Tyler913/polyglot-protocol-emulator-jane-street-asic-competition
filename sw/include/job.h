/*
 * Copyright (c) 2026 Tyler Hong
 * SPDX-License-Identifier: Apache-2.0
 *
 * job.h: protocol jobs for the job-queue kernel (sw/tests/queue).
 *
 * Memory: kernel 0x000-0x0FF, four job slots 0x100-0xDFF, the kernel's stack
 * below 0x3700, host mailbox 0x3700-0x37FF (0xE00-0x36FF is free for more). Slot k uses Wire Engine k; slots differ in size
 * (engines 0 and 1 are full engines, 2 and 3 lite):
 *   slot 0  0x100  832 bytes   slot 2  0x740  512 bytes
 *   slot 1  0x440  768 bytes   slot 3  0x940 1216 bytes
 * Each job lists the slots it fits in (jobs/<name>/slots); it is built once per
 * slot (build/job_<name>_<k>.bin). Slot k's mailbox is PG_SHARED[16k .. 16k + 15]:
 *   [0]      state: 0 idle, 1 run (host), 2 running, 3 done (kernel)
 *   [1..15]  parameters and results, defined by each job
 * The host loads a job image into an idle or done slot while the other slots
 * keep running, then writes 1 to its state word. The kernel calls the job (a
 * pt.h task) round-robin with the others until it returns 1, then sets the
 * state to 3 and raises HOST_IRQ.
 */

#ifndef POLYGLOT_JOB_H
#define POLYGLOT_JOB_H

#include "protocol.h"
#include "pt.h"

#define JOB_SLOTS      4
#define JOB_SLOT_ADDR(k) ((k) == 0 ? 0x100u : (k) == 1 ? 0x440u : (k) == 2 ? 0x740u : 0x940u)
#define JOB_MAILBOX(k) (&PG_SHARED[16 * (k)])

enum { JOB_IDLE = 0, JOB_RUN = 1, JOB_RUNNING = 2, JOB_DONE = 3 };

typedef int (*job_fn)(pt_t *pt, volatile uint32_t *mb);

#ifdef JOB_SLOT
/* In a job: the engine of this slot, and the entry point (first in the image). */
#define JOB_W ((pg_wire_t *)(0x80000100u + 0x100u * JOB_SLOT))
/* job_entry (first in the image) points gp at this job's data, then jumps to the
 * job body. With gp set, the linker turns most static-variable accesses into one
 * gp-relative instruction instead of a lui/lw pair. The kernel does not use gp. */
#define JOB_ENTRY                                                              \
    __asm__(".section .text.job_entry,\"ax\",@progbits\n"                     \
            ".globl job_entry\n"                                              \
            "job_entry:\n"                                                    \
            ".option push\n.option norelax\n"                                 \
            "lui gp, %hi(__global_pointer$)\n"                                \
            "addi gp, gp, %lo(__global_pointer$)\n"                           \
            ".option pop\n"                                                   \
            "j job_main\n"                                                    \
            ".previous\n");                                                   \
    int job_main(pt_t *pt, volatile uint32_t *mb)
#endif

static inline int tx_idle(pg_wire_t *w) { return !(w->STATUS & WIRE_TX_ACTIVE); }

#endif /* POLYGLOT_JOB_H */
