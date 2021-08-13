/*
 * Copyright (c) 2023, STMicroelectronics - All Rights Reserved
 *
 * SPDX-License-Identifier: BSD-3-Clause
 */

#include <assert.h>
#include <errno.h>

#include <arch_helpers.h>
#include <common/debug.h>
#include <drivers/arm/gic_common.h>
#include <drivers/arm/gicv2.h>
#include <drivers/st/stm32mp_reset.h>
#include <lib/mmio.h>
#include <lib/psci/psci.h>
#include <plat/common/platform.h>

#include <platform_def.h>

#define CA35SS_SYSCFG_VBAR_CR	0x2084U

static uintptr_t stm32_sec_entrypoint;
static u_register_t cntfrq_core0;
static uintptr_t saved_entrypoint;

static void stm32_cpu_standby(plat_local_state_t cpu_state)
{
}

/*******************************************************************************
 * STM32MP2 handler called when a power domain is about to be turned on. The
 * mpidr determines the CPU to be turned on.
 * Called by core 0 to activate core 1.
 ******************************************************************************/
static int stm32_pwr_domain_on(u_register_t mpidr)
{
	unsigned long current_cpu_mpidr = read_mpidr();

	if (stm32mp_is_single_core()) {
		return PSCI_E_INTERN_FAIL;
	}

	if (mpidr == current_cpu_mpidr) {
		return PSCI_E_INVALID_PARAMS;
	}

	cntfrq_core0 = read_cntfrq_el0();
	flush_dcache_range((uintptr_t)&cntfrq_core0, sizeof(u_register_t));

	/* Set CA35SS_SYSCFG VBAR address to secure entrypoint */
	mmio_write_32(A35SSC_BASE + CA35SS_SYSCFG_VBAR_CR, stm32_sec_entrypoint);

	/* Reset CPU1 */
	mmio_write_32(RCC_BASE + RCC_C1P1RSTCSETR, RCC_C1P1RSTCSETR_C1P1PORRST);

	return PSCI_E_SUCCESS;
}

static void stm32_pwr_domain_off(const psci_power_state_t *target_state)
{
	/* Nothing to do */
}

static void stm32_pwr_domain_suspend(const psci_power_state_t *target_state)
{
	/* Nothing to do, power domain is not disabled */
}

/*******************************************************************************
 * STM32MP2 handler called when a power domain has just been powered on after
 * being turned off earlier. The target_state encodes the low power state that
 * each level has woken up from.
 * Called by core 1 just after wake up.
 ******************************************************************************/
static void stm32_pwr_domain_on_finish(const psci_power_state_t *target_state)
{
	stm32mp_gic_pcpu_init();

	write_cntfrq_el0(cntfrq_core0);
}

/*******************************************************************************
 * STM32MP2 handler called when a power domain has just been powered on after
 * having been suspended earlier. The target_state encodes the low power state
 * that each level has woken up from.
 ******************************************************************************/
static void stm32_pwr_domain_suspend_finish(const psci_power_state_t
					    *target_state)
{
	/* Nothing to do, power domain is not disabled */
}

static void __dead2 stm32_pwr_domain_pwr_down_wfi(const psci_power_state_t
						  *target_state)
{
	ERROR("stm32mp2 Power Down WFI: operation not handled.\n");
	panic();
}

static void __dead2 stm32_system_off(void)
{
	ERROR("stm32mp2 System Off: operation not handled.\n");
	panic();
}

static void __dead2 stm32_system_reset(void)
{
	stm32mp_system_reset();
}

static int stm32_validate_power_state(unsigned int power_state,
				      psci_power_state_t *req_state)
{
	return PSCI_E_INVALID_PARAMS;
}

static int stm32_validate_ns_entrypoint(uintptr_t entrypoint)
{
	/* The non-secure entry point must be in DDR */
	if (entrypoint < STM32MP_DDR_BASE) {
		return PSCI_E_INVALID_ADDRESS;
	}

	saved_entrypoint = entrypoint;

	return PSCI_E_SUCCESS;
}

static int stm32_node_hw_state(u_register_t target_cpu,
			       unsigned int power_level)
{
	return PSCI_E_INVALID_PARAMS;
}

static void stm32_get_sys_suspend_power_state(psci_power_state_t *req_state)
{
}

/*******************************************************************************
 * Export the platform handlers. The ARM Standard platform layer will take care
 * of registering the handlers with PSCI.
 ******************************************************************************/
static const plat_psci_ops_t stm32_psci_ops = {
	.cpu_standby = stm32_cpu_standby,
	.pwr_domain_on = stm32_pwr_domain_on,
	.pwr_domain_off = stm32_pwr_domain_off,
	.pwr_domain_suspend = stm32_pwr_domain_suspend,
	.pwr_domain_on_finish = stm32_pwr_domain_on_finish,
	.pwr_domain_suspend_finish = stm32_pwr_domain_suspend_finish,
	.pwr_domain_pwr_down_wfi = stm32_pwr_domain_pwr_down_wfi,
	.system_off = stm32_system_off,
	.system_reset = stm32_system_reset,
	.validate_power_state = stm32_validate_power_state,
	.validate_ns_entrypoint = stm32_validate_ns_entrypoint,
	.get_node_hw_state = stm32_node_hw_state,
	.get_sys_suspend_power_state = stm32_get_sys_suspend_power_state,
};

/*******************************************************************************
 * Export the platform specific power ops.
 ******************************************************************************/
int plat_setup_psci_ops(uintptr_t sec_entrypoint,
			const plat_psci_ops_t **psci_ops)
{
	stm32_sec_entrypoint = sec_entrypoint;
	*psci_ops = &stm32_psci_ops;

	return 0;
}
