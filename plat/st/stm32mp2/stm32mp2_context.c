/*
 * Copyright (c) 2023, ARM Limited and Contributors. All rights reserved.
 *
 * SPDX-License-Identifier: BSD-3-Clause
 */

#include <drivers/clk.h>

#include <platform_def.h>
#include <stm32mp2_context.h>

#define BACKUP_CTX_ADDR		STM32MP_BACKUP_RAM_BASE
#define BACKUP_CTX_CLK		CK_BUS_BKPSRAM
#define ENC_KEY_SIZE_IN_BYTES	RISAF_KEY_SIZE_IN_BYTES

struct backup_data_s {
	uint8_t enc_mkey[ENC_KEY_SIZE_IN_BYTES];
};

void stm32mp_pm_save_enc_mkey_in_context(uint8_t *data)
{
	struct backup_data_s *backup_data;

	backup_data = (struct backup_data_s *)BACKUP_CTX_ADDR;

	clk_enable(BACKUP_CTX_CLK);

	memcpy(backup_data->enc_mkey, data, ENC_KEY_SIZE_IN_BYTES);

	clk_disable(BACKUP_CTX_CLK);
}

void stm32mp_pm_get_enc_mkey_from_context(uint8_t *data)
{
	struct backup_data_s *backup_data;

	backup_data = (struct backup_data_s *)BACKUP_CTX_ADDR;

	clk_enable(BACKUP_CTX_CLK);

	memcpy(data, backup_data->enc_mkey, ENC_KEY_SIZE_IN_BYTES);

	clk_disable(BACKUP_CTX_CLK);
}
