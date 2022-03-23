/*
 * Copyright (C) 2021-2022, STMicroelectronics - All Rights Reserved
 *
 * SPDX-License-Identifier: BSD-3-Clause
 */

#include <common/debug.h>
#include <drivers/delay_timer.h>
#include <lib/mmio.h>

#include <ddrphy_phyinit.h>
#include <platform_def.h>

/* Firmware major messages */
#define FW_MAJ_MSG_TRAINING_SUCCESS	0x0000007
#define FW_MAJ_MSG_START_STREAMING	0x0000008
#define FW_MAJ_MSG_TRAINING_FAILED	0x00000FF

#define PHYINIT_DELAY_1US		1U
#define PHYINIT_DELAY_10US		10U

static void wait_uctwriteprotshadow(bool state)
{
	uint16_t read_data;
	uint16_t value = state ? BIT(0) : 0U;

	do {
		read_data = mmio_read_16((uintptr_t)(DDRPHYC_BASE +
						     (4 * (TAPBONLY | CSR_UCTSHADOWREGS_ADDR))));
		udelay(PHYINIT_DELAY_1US);
	} while ((read_data & BIT(0)) != value);
}

static void ack_message_receipt(void)
{
	/* Acknowledge the receipt of the message */
	mmio_write_16((uintptr_t)(DDRPHYC_BASE + (4 * (TAPBONLY | CSR_DCTWRITEPROT_ADDR))), 0U);

	udelay(PHYINIT_DELAY_1US);

	wait_uctwriteprotshadow(true);

	/* Complete the 4-phase protocol */
	mmio_write_16((uintptr_t)(DDRPHYC_BASE + (4 * (TAPBONLY | CSR_DCTWRITEPROT_ADDR))), 1U);

	udelay(PHYINIT_DELAY_1US);
}

static int get_major_message(void)
{
	int message_number;

	wait_uctwriteprotshadow(false);

	message_number = mmio_read_16((uintptr_t)(DDRPHYC_BASE +
						  (4 * (TAPBONLY | CSR_UCTWRITEONLYSHADOW_ADDR))));

	ack_message_receipt();

	return message_number;
}

static int get_streaming_message(void)
{
	int stream_word_lower_part;
	int stream_word_upper_part;

	wait_uctwriteprotshadow(false);

	stream_word_lower_part = mmio_read_16((uintptr_t)(DDRPHYC_BASE +
							  (4 * (TAPBONLY |
								CSR_UCTWRITEONLYSHADOW_ADDR))));

	stream_word_upper_part = mmio_read_16((uintptr_t)(DDRPHYC_BASE +
							  (4 * (TAPBONLY |
								CSR_UCTDATWRITEONLYSHADOW_ADDR))));

	ack_message_receipt();

	return stream_word_lower_part | (stream_word_upper_part << 16);
}

/*
 * Implements the mechanism to wait for completion of training firmware execution.
 *
 * The purpose of user this function is to wait for firmware to finish training.
 * The user can either implement a counter to wait or implement the polling
 * mechanism described in the Training Firmware App Note section "Running the
 * Firmware".  The wait time is highly dependent on the training features
 * enabled via sequencectrl input to the message block.  See Training Firmware
 * App note for details.
 *
 * The default behavior of this function is to print comments relating to this
 * process.  A function call of the same name will be printed in the output text
 * file.
 *
 * The user can choose to leave this function as is, or implement mechanism to
 * trigger mailbox poling event in simulation.
 *
 * \return void
 */
void ddrphy_phyinit_usercustom_g_waitfwdone(void)
{
	int fw_major_message;

	VERBOSE("%s Start\n", __func__);

	do {
		fw_major_message = get_major_message();
		VERBOSE("fw_major_message = %x\n", (unsigned int)fw_major_message);

		if (fw_major_message == FW_MAJ_MSG_START_STREAMING) {
			int i;
			int read_data = get_streaming_message();
			int stream_len = read_data & 0xFFFF;

			for (i = 0; i < stream_len; i++) {
				read_data = get_streaming_message();
				VERBOSE("streaming message = %x\n", (unsigned int)read_data);
			}
		}
	} while ((fw_major_message != FW_MAJ_MSG_TRAINING_SUCCESS) &&
		 (fw_major_message != FW_MAJ_MSG_TRAINING_FAILED));

	udelay(PHYINIT_DELAY_10US);

	if (fw_major_message == FW_MAJ_MSG_TRAINING_FAILED) {
		ERROR("%s Training has failed.\n", __func__);
	}

	VERBOSE("%s End\n", __func__);
}
