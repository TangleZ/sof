/* SPDX-License-Identifier: BSD-3-Clause
 *
 * Copyright 2019 NXP
 *
 * Author: Daniel Baluta <daniel.baluta@nxp.com>
 */

#ifdef __SOF_PLATFORM_H__

#ifndef __PLATFORM_PLATFORM_H__
#define __PLATFORM_PLATFORM_H__

#if !defined(__ASSEMBLER__) && !defined(LINKER)

#include <arch/lib/wait.h>
#include <sof/drivers/interrupt.h>
#include <sof/lib/clk.h>
#include <sof/drivers/mu.h>
#include <sof/lib/mailbox.h>
#include <stddef.h>
#include <stdint.h>

struct ll_schedule_domain;
struct timer;

#define PLATFORM_DEFAULT_CLOCK CLK_CPU(0)
#define LPSRAM_SIZE 16384

/* IPC Interrupt */
#define PLATFORM_IPC_INTERRUPT		IRQ_NUM_MU3
#define PLATFORM_IPC_INTERRUPT_NAME	NULL

/* Host page size */
#define HOST_PAGE_SIZE		4096
#define PLATFORM_PAGE_TABLE_SIZE	256

/* pipeline IRQ */
#define PLATFORM_SCHEDULE_IRQ		IRQ_NUM_SOFTWARE0
#define PLATFORM_SCHEDULE_IRQ_NAME	NULL

/* Platform stream capabilities */
#define PLATFORM_MAX_CHANNELS	4
#define PLATFORM_MAX_STREAMS	5

/* local buffer size of DMA tracing */
#define DMA_TRACE_LOCAL_SIZE	HOST_PAGE_SIZE

/* trace bytes flushed during panic */
#define DMA_FLUSH_TRACE_SIZE	(MAILBOX_TRACE_SIZE >> 2)

/* the interval of DMA trace copying */
#define DMA_TRACE_PERIOD		500000

/*
 * the interval of reschedule DMA trace copying in special case like half
 * fullness of local DMA trace buffer
 */
#define DMA_TRACE_RESCHEDULE_TIME	100

/* DSP default delay in cycles */
#define PLATFORM_DEFAULT_DELAY	12

#define SRAM_REG_FW_STATUS     0x4

/* Platform defined panic code */
static inline void platform_panic(uint32_t p)
{
	/* Store the error code in the debug box so the
	 * application processor can pick it up. Takes up 4 bytes
	 * from the debug box.
	 */
	mailbox_sw_reg_write(SRAM_REG_FW_STATUS, p);

	/* Notify application processor */
	imx_mu_xcr_rmw(IMX_MU_VERSION, IMX_MU_GCR, IMX_MU_xCR_GIRn(IMX_MU_VERSION, 1), 0);
}

/**
 * \brief Platform specific CPU entering idle.
 * May be power-optimized using platform specific capabilities.
 * @param level Interrupt level.
 */
static inline void platform_wait_for_interrupt(int level)
{
	arch_wait_for_interrupt(level);
}

extern intptr_t _module_init_start;
extern intptr_t _module_init_end;
#endif

#endif /* __PLATFORM_PLATFORM_H__ */

#else

#error "This file shouldn't be included from outside of sof/platform.h"

#endif /* __SOF_PLATFORM_H__ */
