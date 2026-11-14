/*
 * Copyright (c) 2026 Tyler Hong
 * SPDX-License-Identifier: Apache-2.0
 *
 * queue: a resident kernel that runs up to four protocol jobs at once and
 * lets the host swap jobs while the others keep running (see job.h for the
 * memory map and the mailbox). Jobs live in sw/jobs/. Used by test/test_queue.py.
 */

#include "job.h"

int main(void)
{
    static pt_t pt[JOB_SLOTS];

    for (int i = 0; i < 64; i++) PG_SHARED[i] = 0;
    host_irq(0);
    for (;;) {
        int done = 0;
        for (int k = 0; k < JOB_SLOTS; k++) {
            volatile uint32_t *mb = JOB_MAILBOX(k);
            uint32_t s = mb[0];
            if (s == JOB_RUN) {
                PT_INIT(&pt[k]);
                mb[0] = s = JOB_RUNNING;
            }
            if (s == JOB_RUNNING &&
                ((job_fn)JOB_SLOT_ADDR(k))(&pt[k], mb))
                mb[0] = s = JOB_DONE;
            done |= s == JOB_DONE;
        }
        host_irq(done);
    }
}
