/*
 * Copyright (c) 2021, STMicroelectronics - All Rights Reserved
 *
 * SPDX-License-Identifier: BSD-3-Clause
 */

#include <arch_helpers.h>
#include <common/debug.h>
#include <drivers/delay_timer.h>
#include <drivers/st/stm32mp_ddr.h>
#include <drivers/st/stm32mp2_ddr.h>
#include <drivers/st/stm32mp2_ddr_helpers.h>
#include <drivers/st/stm32mp2_ddr_regs.h>
#include <lib/mmio.h>

#include <platform_def.h>

static enum stm32mp2_ddr_sr_mode saved_ddr_sr_mode;

int ddr_sw_self_refresh_exit(void)
{
	int ret = -1;

	switch (saved_ddr_sr_mode) {
	case DDR_SSR_MODE:
		/* TODO create related service */
		ret = 0;
		break;
	case DDR_HSR_MODE:
		/* TODO create related service */
		ret = 0;
		break;
	case DDR_ASR_MODE:
		/* TODO create related service */
		ret = 0;
		break;
	default:
		break;
	}

	return ret;
}

uint32_t ddr_get_io_calibration_val(void)
{
	/* TODO create related service */

	return 0U;
}

int ddr_standby_sr_entry(void)
{
	int ret = -1;

	switch (saved_ddr_sr_mode) {
	case DDR_SSR_MODE:
		/* TODO create related service */
		ret = 0;
		break;
	case DDR_HSR_MODE:
		/* TODO create related service */
		ret = 0;
		break;
	case DDR_ASR_MODE:
		/* TODO create related service */
		ret = 0;
		break;
	default:
		break;
	}

	return ret;
}

enum stm32mp2_ddr_sr_mode ddr_read_sr_mode(void)
{
	uint32_t pwrctl = mmio_read_32(stm32mp_ddrctrl_base() + DDRCTRL_PWRCTL);
	enum stm32mp2_ddr_sr_mode mode = DDR_SR_MODE_INVALID;

	switch (pwrctl & (DDRCTRL_PWRCTL_EN_DFI_DRAM_CLK_DISABLE |
			  DDRCTRL_PWRCTL_SELFREF_EN)) {
	case 0U:
		mode = DDR_SSR_MODE;
		break;
	case DDRCTRL_PWRCTL_EN_DFI_DRAM_CLK_DISABLE:
		mode = DDR_HSR_MODE;
		break;
	case DDRCTRL_PWRCTL_EN_DFI_DRAM_CLK_DISABLE | DDRCTRL_PWRCTL_SELFREF_EN:
		mode = DDR_ASR_MODE;
		break;
	default:
		break;
	}

	return mode;
}

void ddr_set_sr_mode(enum stm32mp2_ddr_sr_mode mode)
{
	int ret = -1;

	if (mode == saved_ddr_sr_mode) {
		return;
	}

	switch (mode) {
	case DDR_SSR_MODE:
		/* TODO create related service */
		ret = 0;
		break;
	case DDR_HSR_MODE:
		/* TODO create related service */
		ret = 0;
		break;
	case DDR_ASR_MODE:
		/* TODO create related service */
		ret = 0;
		break;
	default:
		break;
	}

	if (ret != 0) {
		ERROR("Unknown Self Refresh mode\n");
		panic();
	}
}

void ddr_save_sr_mode(void)
{
	saved_ddr_sr_mode = ddr_read_sr_mode();
}

void ddr_restore_sr_mode(void)
{
	ddr_set_sr_mode(saved_ddr_sr_mode);
}
