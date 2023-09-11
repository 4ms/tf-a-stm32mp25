/*
 * Copyright (C) 2023, STMicroelectronics - All Rights Reserved
 *
 * SPDX-License-Identifier: BSD-3-Clause
 */

#include <assert.h>
#include <errno.h>
#include <stdint.h>

#include <drivers/delay_timer.h>
#include <drivers/st/stm32mp_ddr.h>
#include <drivers/st/stm32mp_pmic2.h>

#include <platform_def.h>

static int pmic_ddr_power_init(enum ddr_type ddr_type)
{
#if STM32MP_DDR4_TYPE
	int status;
	struct rdev *vdd_ddr, *vref_ddr, *vtt_ddr, *vpp_ddr;

	/*
	 * DDR power on sequence is:
	 * enable VPP_DDR
	 * wait 2ms
	 * enable VREF_DDR, VTT_DDR, VPP_DDR
	 */

	/*
	 * FIXME: to be replaced by regulator_get_by_supply_name(... "vpp")
	 * vpp-supply = <&vpp_ddr>; in the device-tree
	 */
	/* vpp_ddr */
	vpp_ddr = regulator_get_by_name("ldo5");
	if (vpp_ddr == NULL) {
		return -ENOENT;
	}
	status = regulator_set_min_voltage(vpp_ddr);
	if (status != 0) {
		return status;
	}

	/* vdd_ddr */
	vdd_ddr = regulator_get_by_name("buck6");
	if (vdd_ddr == NULL) {
		return -ENOENT;
	}
	status = regulator_set_min_voltage(vdd_ddr);
	if (status != 0) {
		return status;
	}

	/* vref_ddr */
	vref_ddr = regulator_get_by_name("refddr");
	if (vref_ddr == NULL) {
		return -ENOENT;
	}

	/* vtt_ddr */
	vtt_ddr = regulator_get_by_name("ldo3");
	if (vtt_ddr == NULL) {
		return -ENOENT;
	}
	status = regulator_set_flag(vtt_ddr, REGUL_SINK_SOURCE);
	if (status != 0) {
		return status;
	}

	status = regulator_enable(vpp_ddr);
	if (status != 0) {
		return status;
	}

	/* could be set via enable_ramp_delay on vpp_ddr */
	udelay(2000);

	status = regulator_enable(vdd_ddr);
	if (status != 0) {
		return status;
	}

	status = regulator_enable(vref_ddr);
	if (status != 0) {
		return status;
	}

	status = regulator_enable(vtt_ddr);
	if (status != 0) {
		return status;
	}
#elif STM32MP_LPDDR4_TYPE
	int status;
	struct rdev *vdd1_ddr, *vdd2_ddr;

	/*
	 * DDR power on sequence is:
	 * enable VDD1_DDR
	 * wait 2ms
	 * enable VDD2_DDR
	 */

	/*
	 * FIXME: to be replaced by regulator_get_by_supply_name(... "vdd1_ddr")
	 * vdd1-supply = <&vdd1_ddr>; in the device-tree
	 */
	/* vdd1_ddr */
	vdd1_ddr = regulator_get_by_name("ldo3");
	if (vdd1_ddr == NULL) {
		return -ENOENT;
	}
	status = regulator_set_min_voltage(vdd1_ddr);
	if (status != 0) {
		return status;
	}

	/* vdd2_ddr */
	vdd2_ddr = regulator_get_by_name("buck6");
	if (vdd2_ddr == NULL) {
		return -ENOENT;
	}
	status = regulator_set_min_voltage(vdd2_ddr);
	if (status != 0) {
		return status;
	}

	status = regulator_enable(vdd1_ddr);
	if (status != 0) {
		return status;
	}

	/* could be set via enable_ramp_delay on vdd1_ddr */
	udelay(2000);

	status = regulator_enable(vdd2_ddr);
	if (status != 0) {
		return status;
	}
#else
	ERROR("DDR type no supported with PMIC\n");
	panic();
#endif

	return 0;
}

int stm32mp_board_ddr_power_init(enum ddr_type ddr_type)
{
	if (dt_pmic_status() > 0) {
		return pmic_ddr_power_init(ddr_type);
	}

	return 0;
}
