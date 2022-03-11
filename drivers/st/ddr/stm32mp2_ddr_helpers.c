/*
 * Copyright (c) 2021-2022, STMicroelectronics - All Rights Reserved
 *
 * SPDX-License-Identifier: BSD-3-Clause
 */

#include <errno.h>

#include <arch_helpers.h>
#include <common/debug.h>
#include <drivers/delay_timer.h>
#include <drivers/st/stm32mp_ddr.h>
#include <drivers/st/stm32mp2_ddr.h>
#include <drivers/st/stm32mp2_ddr_helpers.h>
#include <drivers/st/stm32mp2_ddr_regs.h>
#include <lib/mmio.h>

#include <platform_def.h>

/* HW idle period (unit: Multiples of 32 DFI clock cycles) */
#define HW_IDLE_PERIOD			0x3

static enum stm32mp2_ddr_sr_mode saved_ddr_sr_mode;

#pragma weak stm32_ddrdbg_get_base
uintptr_t stm32_ddrdbg_get_base(void)
{
	return 0U;
}

static int sr_loop(bool is_entry)
{
	uint32_t read_data;
	uint64_t timeout = timeout_init_us(DDR_TIMEOUT_US_1S);
	bool repeat_loop = false;

	/* Wait for DDRCTRL to be out of or back to "normal/mission mode" */
	do {
		read_data = mmio_read_32(stm32mp_ddrctrl_base() + DDRCTRL_STAT) &
			    DDRCTRL_STAT_SELFREF_TYPE_MASK;

		if (timeout_elapsed(timeout)) {
			return -ETIMEDOUT;
		}

		if (is_entry) {
			repeat_loop = (read_data == 0x0);
		} else {
			repeat_loop = (read_data != 0x0);
		}
	} while (repeat_loop);

	return 0;
}

static int sr_entry_loop(void)
{
	return sr_loop(true);
}

static int sr_exit_loop(void)
{
	return sr_loop(false);
}

static int sr_ssr_set(void)
{
	return 0;
}

static int sr_ssr_entry(void)
{
	/* Enable APB access to internal CSR registers */
	mmio_write_32(stm32mp_ddrphyc_base() + DDRPHY_APBONLY0_MICROCONTMUXSEL, 0);

	/* SW self refresh entry prequested */
	mmio_write_32(stm32mp_ddrctrl_base() + DDRCTRL_PWRCTL, DDRCTRL_PWRCTL_SELFREF_SW);

	return sr_entry_loop();
}

static int sr_ssr_exit(void)
{
	/* Enable APB access to internal CSR registers */
	mmio_write_32(stm32mp_ddrphyc_base() + DDRPHY_APBONLY0_MICROCONTMUXSEL, 0);

	/* SW self refresh entry prequested */
	mmio_write_32(stm32mp_ddrctrl_base() + DDRCTRL_PWRCTL, 0);

	return sr_exit_loop();
}

static int sr_hsr_set(void)
{
	uintptr_t ddrctrl_base = stm32mp_ddrctrl_base();

	mmio_write_32(stm32mp_rcc_base() + RCC_DDRITFCFGR, RCC_DDRITFCFGR_DDRCKMOD_HSR);

	/*
	 * manage quasi-dynamic registers modification
	 * hwlpctl.hw_lp_en : Group 2
	 */
	if (stm32mp_ddr_sw_selfref_entry((struct stm32mp_ddrctl *)ddrctrl_base) != 0) {
		panic();
	}
	stm32mp_ddr_start_sw_done((struct stm32mp_ddrctl *)ddrctrl_base);

	mmio_write_32(ddrctrl_base + DDRCTRL_HWLPCTL,
		      DDRCTRL_HWLPCTL_HW_LP_EN | DDRCTRL_HWLPCTL_HW_LP_EXIT_IDLE_EN |
		      (HW_IDLE_PERIOD << DDRCTRL_HWLPCTL_HW_LP_IDLE_X32_SHIFT));

	stm32mp_ddr_wait_sw_done_ack((struct stm32mp_ddrctl *)ddrctrl_base);
	stm32mp_ddr_sw_selfref_exit((struct stm32mp_ddrctl *)ddrctrl_base);

	return 0;
}

static int sr_hsr_entry(void)
{
	mmio_write_32(stm32mp_rcc_base() + RCC_DDRCPCFGR, RCC_DDRCPCFGR_DDRCPLPEN);

	return sr_entry_loop(); /* read_data should be equal to 0x223 */
}

static int sr_hsr_exit(void)
{
	mmio_write_32(stm32mp_rcc_base() + RCC_DDRCPCFGR,
		      RCC_DDRCPCFGR_DDRCPLPEN | RCC_DDRCPCFGR_DDRCPEN);

	/* TODO: check if sr_exit_loop() is needed here */

	return 0;
}

static int sr_asr_set(void)
{
	mmio_write_32(stm32_ddrdbg_get_base() + DDRDBG_LP_DISABLE, 0);

	return 0;
}

static int sr_asr_entry(void)
{
	/*
	 * Automatically enter into self refresh when there is no ddr traffic
	 * for the delay programmed into SYSCONF_DDRC_AUTO_SR_DELAY register.
	 * Default value is 0x20 (unit: Multiples of 32 DFI clock cycles).
	 */
	return sr_entry_loop();
}

static int sr_asr_exit(void)
{
	return sr_exit_loop();
}

int ddr_sw_self_refresh_exit(void)
{
	int ret = -1;

	switch (saved_ddr_sr_mode) {
	case DDR_SSR_MODE:
		ret = sr_ssr_exit();
		break;
	case DDR_HSR_MODE:
		ret = sr_hsr_exit();
		break;
	case DDR_ASR_MODE:
		ret = sr_asr_exit();
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
		ret = sr_ssr_entry();
		break;
	case DDR_HSR_MODE:
		ret = sr_hsr_entry();
		break;
	case DDR_ASR_MODE:
		ret = sr_asr_entry();
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
		ret = sr_ssr_set();
		break;
	case DDR_HSR_MODE:
		ret = sr_hsr_set();
		break;
	case DDR_ASR_MODE:
		ret = sr_asr_set();
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
