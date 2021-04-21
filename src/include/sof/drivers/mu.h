/* SPDX-License-Identifier: BSD-3-Clause
 *
 * Copyright(c) 2019 Intel Corporation. All rights reserved.
 * Copyright 2020 NXP
 *
 * Author: Tomasz Lauda <tomasz.lauda@linux.intel.com>
 * Author: Daniel Baluta <daniel.baluta@nxp.com>
 */

#ifndef __SOF_DRIVERS_MU_H__
#define __SOF_DRIVERS_MU_H__

#include <sof/bit.h>
#include <stdint.h>

/* Transmit Register */
#define IMX_MU_xTRn(x)		(0x00 + 4 * (x))
/* Receive Register */
#define IMX_MU_xRRn(x)		(0x10 + 4 * (x))
/* Status Register */
#define IMX_MU_xSR		0x20
#define IMX_MU_xSR_GIPn(x)	BIT(28 + (3 - (x)))
#define IMX_MU_xSR_RFn(x)	BIT(24 + (3 - (x)))
#define IMX_MU_xSR_TEn(x)	BIT(20 + (3 - (x)))
#define IMX_MU_xSR_BRDIP	BIT(9)

/* Control Register */
#define IMX_MU_xCR		0x24
/* General Purpose Interrupt Enable */
#define IMX_MU_xCR_GIEn(x)	BIT(28 + (3 - (x)))
/* Receive Interrupt Enable */
#define IMX_MU_xCR_RIEn(x)	BIT(24 + (3 - (x)))
/* Transmit Interrupt Enable */
#define IMX_MU_xCR_TIEn(x)	BIT(20 + (3 - (x)))
/* General Purpose Interrupt Request */
#define IMX_MU_xCR_GIRn(x)	BIT(16 + (3 - (x)))

#define IMX8ULP_MU_GIER       0x110
#define IMX8ULP_MU_GCR        0x114
/* General Status Register */
#define IMX8ULP_MU_GSR        0x118
#define IMX8ULP_MU_TCR        0x120
/* Transmit Status Register */
#define IMX8ULP_MU_TSR        0x124
#define IMX8ULP_MU_RCR        0x128
/* Receive Status Register */
#define IMX8ULP_MU_RSR        0x12C
#define IMX8ULP_MU_TR0        0x200
#define IMX8ULP_MU_RR0        0x280

#if defined CONFIG_IMX8ULP
#define IMX_MU_GSR           IMX8ULP_MU_GSR
#define IMX_MU_GSR_GIPn(x)	BIT(3-(x))
#else
#define IMX_MU_GSR           IMX_MU_xSR
#define IMX_MU_GSR_GIPn(x)	IMX_MU_xSR_GIPn(x)
#endif

static inline uint32_t imx_mu_read(uint32_t reg)
{
	return *((volatile uint32_t*)(MU_BASE + reg));
}

static inline void imx_mu_write(uint32_t val, uint32_t reg)
{
	*((volatile uint32_t*)(MU_BASE + reg)) = val;
}

static inline uint32_t imx_mu_xcr_rmw(uint32_t set, uint32_t clr)
{
#if defined CONFIG_IMX8ULP
	volatile uint32_t val;

	if (clr & 0xf0000000 || clr == 0) {
		val = imx_mu_read(IMX8ULP_MU_GIER);
		val &= ~clr;
		val |= set;
		imx_mu_write(val, IMX8ULP_MU_GIER);
	}
	if (clr & 0x0f000000 || clr == 0) {
		val = imx_mu_read(IMX8ULP_MU_RCR);
		val &= ~clr;
		val |= set;
		imx_mu_write(val, IMX8ULP_MU_RCR);
	}
	if (clr & 0x00f00000 || clr == 0) {
		val = imx_mu_read(IMX8ULP_MU_TCR);
		val &= ~clr;
		val |= set;
		imx_mu_write(val, IMX8ULP_MU_TCR);
	}
	if (clr & 0x000f0000 || clr == 0) {
		val = imx_mu_read(IMX8ULP_MU_GCR);
		val &= ~clr;
		val |= set;
		imx_mu_write(val, IMX8ULP_MU_GCR);
	}
	return val;
#else
	volatile uint32_t val;

	val = imx_mu_read(IMX_MU_xCR);
	val &= ~clr;
	val |= set;
	imx_mu_write(val, IMX_MU_xCR);

	return val;
#endif
}

static inline uint32_t imx_mu_xsr_rmw(uint32_t set, uint32_t clr)
{
#if defined CONFIG_IMX8ULP
	volatile uint32_t val;

	if (clr & 0xf0000000 || clr == 0) {
		val = imx_mu_read(IMX8ULP_MU_GSR);
		val &= ~clr;
		val |= set;
		imx_mu_write(val, IMX8ULP_MU_GSR);
	}
	if (clr & 0x0f000000 || clr == 0) {
		val = imx_mu_read(IMX8ULP_MU_RSR);
		val &= ~clr;
		val |= set;
		imx_mu_write(val, IMX8ULP_MU_RSR);
	}
	if (clr & 0x00f00000 || clr == 0) {
		val = imx_mu_read(IMX8ULP_MU_TSR);
		val &= ~clr;
		val |= set;
		imx_mu_write(val, IMX8ULP_MU_TSR);
	}
	if (clr & 0x000f0000 || clr == 0) {
		val = imx_mu_read(IMX8ULP_MU_GCR);
		val &= ~clr;
		val |= set;
		imx_mu_write(val, IMX8ULP_MU_GCR);
	}
	return val;
#else
	volatile uint32_t val;

	val = imx_mu_read(IMX_MU_xSR);
	val &= ~clr;
	val |= set;
	imx_mu_write(val, IMX_MU_xSR);

	return val;
#endif

}

#endif /* __SOF_DRIVERS_MU_H__ */
