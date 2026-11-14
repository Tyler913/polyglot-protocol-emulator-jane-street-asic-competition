/*
 * Copyright (c) 2026 Tyler Hong
 * SPDX-License-Identifier: Apache-2.0
 *
 * The job-queue kernel (sw/tests/queue): the normal layout, but everything
 * must end below the first job slot (job.h: JOB_SLOT_BASE).
 */
INCLUDE rt/link.ld
ASSERT(__bss_end <= 0x100, "queue kernel too large: it must end below the first job slot")
