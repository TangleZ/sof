// SPDX-License-Identifier: BSD-3-Clause
//
// Copyright 2019 NXP
//
// Author: Daniel Baluta <daniel.baluta@nxp.com>

#include <sof/common.h>
#include <sof/drivers/edma.h>
#include <sof/drivers/sai.h>
#include <sof/lib/dai.h>
#include <sof/lib/memory.h>
#include <sof/sof.h>
#include <sof/spinlock.h>
#include <ipc/dai.h>
#include <ipc/stream.h>

static SHARED_DATA struct dai sai[] = {
{
	.index = 6,
	.plat_data = {
		.base = SAI_6_BASE,
		.fifo[SOF_IPC_STREAM_PLAYBACK] = {
			.offset		= SAI_6_BASE + REG_SAI_TDR0,
			/* use depth to model the HW FIFO size:
			 * each channel includes a 64 x 32 bit FIFO
			 * that can be accessed using Transmit or
			 * Receive Data Registers
			 */
			.depth		= 64,  /* in 4 bytes words */
			.handshake	= EDMA_HANDSHAKE(EDMA0_SAI_CHAN_TX_IRQ,
							 EDMA0_SAI_CHAN_TX),
		},
		.fifo[SOF_IPC_STREAM_CAPTURE] = {
			.offset		= SAI_6_BASE + REG_SAI_RDR0,
			.depth		= 64,  /* in 4 bytes words */
			.handshake	= EDMA_HANDSHAKE(EDMA0_SAI_CHAN_RX_IRQ,
							 EDMA0_SAI_CHAN_RX),
		},
		.dmamux_rx_num = DMAMUX2_SAI6_RX_NUM,
		.dmamux_tx_num = DMAMUX2_SAI6_TX_NUM,
	},
	.drv = &sai_driver,
},
};

const struct dai_type_info dti[] = {
	{
		.type = SOF_DAI_IMX_SAI,
		.dai_array = sai,
		.num_dais = ARRAY_SIZE(sai)
	},
};

const struct dai_info lib_dai = {
	.dai_type_array = dti,
	.num_dai_types = ARRAY_SIZE(dti)
};

int dai_init(struct sof *sof)
{
	int i;

	 /* initialize spin locks early to enable ref counting */
	for (i = 0; i < ARRAY_SIZE(sai); i++)
		spinlock_init(&sai[i].lock);

	platform_shared_commit(sai, sizeof(*sai));

	sof->dai_info = &lib_dai;

	return 0;
}
