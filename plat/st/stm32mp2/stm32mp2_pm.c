/*
 * Copyright (c) 2023, STMicroelectronics - All Rights Reserved
 *
 * SPDX-License-Identifier: BSD-3-Clause
 */

#include <assert.h>
#include <errno.h>

#include <arch_helpers.h>
#include <bl31/interrupt_mgmt.h>
#include <common/debug.h>
#include <common/fdt_wrappers.h>
#include <drivers/arm/gic_common.h>
#include <drivers/arm/gicv2.h>
#include <drivers/st/bsec3_reg.h>
#include <drivers/st/stm32mp_clkfunc.h>
#include <drivers/st/stm32mp_pmic2.h>
#include <drivers/st/stm32mp_reset.h>
#include <drivers/st/stm32mp2_ddr_helpers.h>
#include <lib/mmio.h>
#include <lib/psci/psci.h>
#include <plat/common/platform.h>

#include <platform_def.h>
#include <stm32mp2_context.h>

#define CA35SS_SYSCFG_VBAR_CR	0x2084U

#define RAMCFG_RETRAMCR		0x180U
#define SRAMHWERDIS		BIT(12)

/* GIC interrupt number */
#define RCC_WAKEUP_IRQn		254

/* value with 64 MHz HSI period */
#define PWRLPDLYCR_VAL(delay, lsmcu)	((64000000U / (delay)) * (1U + (lsmcu)))

static volatile uint32_t stm32mp_core0_go;

/* support PSCI v1.0 Extended State-ID with the recommended encoding */
#define LVL_CORE		U(0)
#define LVL_D1			U(1)
#define LVL_D1_LPLV		U(2)
#define LVL_D2			U(3)
#define LVL_D2_LPLV		U(4)

#define stm32_make_pwrstate(lvl4, lvl3, lvl2, lvl1, lvl0, type) \
	(((STM32MP_LOCAL_STATE_ ## lvl4) << (PLAT_LOCAL_PSTATE_WIDTH * 4)) | \
	 ((STM32MP_LOCAL_STATE_ ## lvl3) << (PLAT_LOCAL_PSTATE_WIDTH * 3)) | \
	 ((STM32MP_LOCAL_STATE_ ## lvl2) << (PLAT_LOCAL_PSTATE_WIDTH * 2)) | \
	 ((STM32MP_LOCAL_STATE_ ## lvl1) << (PLAT_LOCAL_PSTATE_WIDTH * 1)) | \
	 ((STM32MP_LOCAL_STATE_ ## lvl0) << (PLAT_LOCAL_PSTATE_WIDTH * 0)) | \
	 ((type) << PSTATE_TYPE_SHIFT))

#define stm32_get_stateid_lvl(pwr_domain_state, lvl) \
	(pwr_domain_state[(lvl)] << (PLAT_LOCAL_PSTATE_WIDTH * (lvl)))

#define stm32_get_stateid(pwr_domain_state) \
	(stm32_get_stateid_lvl(pwr_domain_state, LVL_CORE) | \
	 stm32_get_stateid_lvl(pwr_domain_state, LVL_D1) | \
	 stm32_get_stateid_lvl(pwr_domain_state, LVL_D1_LPLV) | \
	 stm32_get_stateid_lvl(pwr_domain_state, LVL_D2) | \
	 stm32_get_stateid_lvl(pwr_domain_state, LVL_D2_LPLV) | \
	 ((pwr_domain_state[LVL_D1_LPLV] == STM32MP_LOCAL_STATE_OFF ? PSTATE_TYPE_POWERDOWN : 0) \
		<< PSTATE_TYPE_SHIFT))

/* State-id - 0x00000001 */
#define PWRSTATE_RUN \
	stm32_make_pwrstate(RUN, RUN, RUN, RUN, RET, PSTATE_TYPE_STANDBY)

/* State-id - 0x00000011 Stop1 */
#define PWRSTATE_STOP1 \
	stm32_make_pwrstate(RUN, RUN, RUN, RET, RET, PSTATE_TYPE_STANDBY)

/* State-id - 0x00000021 LP-Stop1*/
#define PWRSTATE_LP_STOP1 \
	stm32_make_pwrstate(RUN, RUN, RUN, LP, RET, PSTATE_TYPE_STANDBY)

/* State-id - 0x00000211 LPLV-Stop1*/
#define PWRSTATE_LPLV_STOP1 \
	stm32_make_pwrstate(RUN, RUN, LP, RET, RET, PSTATE_TYPE_STANDBY)

/* State-id - 0x40001333 Stop2 */
#define PWRSTATE_STOP2 \
	stm32_make_pwrstate(RUN, RET, OFF, OFF, OFF, PSTATE_TYPE_POWERDOWN)

/* State-id - 0x40002333 LP-Stop2*/
#define PWRSTATE_LP_STOP2 \
	stm32_make_pwrstate(RUN, LP, OFF, OFF, OFF, PSTATE_TYPE_POWERDOWN)

/* State-id - 0x40023333 LPLV-Stop2*/
#define PWRSTATE_LPLV_STOP2 \
	stm32_make_pwrstate(LP, OFF, OFF, OFF, OFF, PSTATE_TYPE_POWERDOWN)

/* State-id - 0x40033333 Standby */
#define PWRSTATE_STANDBY \
	stm32_make_pwrstate(OFF, OFF, OFF, OFF, OFF, PSTATE_TYPE_POWERDOWN)

/*
 *  The table storing the valid idle power states. Ensure that the
 *  array entries are populated in ascending order of state-id to
 *  enable us to use binary search during power state validation.
 *  The table must be terminated by a NULL entry.
 */
const unsigned int stm32mp_pm_idle_states[] = {
	PWRSTATE_RUN,
	PWRSTATE_STOP1,
	PWRSTATE_LP_STOP1,
	PWRSTATE_LPLV_STOP1,
	PWRSTATE_STOP2,
	PWRSTATE_LP_STOP2,
	PWRSTATE_LPLV_STOP2,
	0U, /* sentinel */
};

#define PM_IDLE_STATES_SIZE ARRAY_SIZE(stm32mp_pm_idle_states)

/* The supported low power mode on the board, including STANDBY */
unsigned int stm32mp_supported_pwr_states[PM_IDLE_STATES_SIZE + 1U];

/*******************************************************************************
 * STM32MP2 handler called when a CPU is about to enter standby.
 ******************************************************************************/
static void stm32_cpu_standby(plat_local_state_t cpu_state)
{
	u_register_t scr = read_scr_el3();

	assert(cpu_state == STM32MP_LOCAL_STATE_RET);

	/* Enable the Non-secure interrupt to wake the CPU. */
	write_scr_el3(scr | SCR_IRQ_BIT | SCR_FIQ_BIT);
	isb();
	/*
	 * Enter standby state.
	 * dsb is good practice before using wfi to enter low power states.
	 */
	dsb();
	wfi();
}

/*******************************************************************************
 * STM32MP2 handler called when a power domain is about to be turned on. The
 * mpidr determines the CPU to be turned on.
 * Called by core 0 to activate core 1.
 ******************************************************************************/
static int stm32_pwr_domain_on(u_register_t mpidr)
{
	unsigned int core_id = MPIDR_AFFLVL0_VAL(mpidr);

	if (stm32mp_is_single_core()) {
		return PSCI_E_NOT_SUPPORTED;
	}

	if (core_id == STM32MP_PRIMARY_CPU) {
		/* Cortex-A35 core0 can't be turned OFF, emulate it with a WFE loop */
		VERBOSE("BL31: Releasing core0 from wait loop...\n");
		stm32mp_core0_go = 1U;
		flush_dcache_range((uintptr_t)&stm32mp_core0_go, sizeof(stm32mp_core0_go));
		dsb();
		isb();
		sev();
	} else {
		/* Reset the secondary core */
		mmio_write_32(RCC_BASE + RCC_C1P1RSTCSETR, RCC_C1P1RSTCSETR_C1P1PORRST);
	}

	return PSCI_E_SUCCESS;
}

static void stm32_pwr_domain_off(const psci_power_state_t *target_state)
{
	/* Prevent interrupts from spuriously waking up this cpu */
	stm32mp_gic_cpuif_disable();
}

static void stm32mp2_enable_rcc_wakeup_irq(uintptr_t rcc_base)
{
	/* TODO : disable the other interruption in TF-A ? */

	/* Enable RCC Wake-up interrupt, */
	mmio_setbits_32(rcc_base + RCC_C1CIESETR, RCC_C1CIESETR_WKUPIE);

	plat_ic_set_interrupt_priority(RCC_WAKEUP_IRQn, GIC_HIGHEST_SEC_PRIORITY);
	plat_ic_enable_interrupt(RCC_WAKEUP_IRQn);
}

static void stm32mp2_disable_rcc_wakeup_irq(uintptr_t rcc_base)
{
	/* TODO : restore PMR ?*/

	plat_ic_disable_interrupt(RCC_WAKEUP_IRQn);

	/* Clear wakeup flag */
	mmio_setbits_32(rcc_base + RCC_C1CIFCLRR, RCC_C1CIFCLRR_WKUPF);

	/* Disable RCC Wake-up interrupt */
	mmio_clrbits_32(rcc_base + RCC_C1CIESETR, RCC_C1CIESETR_WKUPIE);
}

#if CONFIG_STM32MP25X_REVA
/*
 * M33 firmware: so.s = set DEESLEEP bit in SCR and WFI
 -------------------------
.cpu cortex-m33
.thumb
.globl _start
_start:
.word 0x0E080100
.word reset
.word loop
.word loop
.thumb_func
loop: b loop
.thumb_func
reset:
    ldr R1, =0xE000ED10
    mov R0, #0x4
    str R0, [R1]
loop_wfi:
    wfi
    b loop_wfi
 -------------------------
 * compiled with command
 *   arm-none-eabi-as so.s -o so.o
 *   arm-none-eabi-ld -Ttext=0x0A080000 so.o -o so.elf
 *   arm-none-eabi-objdump -D so.elf > so.list
 *   arm-none-eabi-objcopy so.elf -O binary so.bin
 *
 */
static const uint16_t retram_firmware[] = {
	0x0100, 0x0e08, /* stack */
	0x0013,	0x0a08, /* reset */
	0x0011,	0x0a08, /* loop */
	0x0011,	0x0a08, /* loop */
	0xe7fe,		/* a080010: b.n a080010 <loop> */
	0x4902,		/* a080012: ldr r1, [pc, #8] */
	0x2004,		/* a080014: movs r0, #4 to set DEESLEEP bit in SCR */
	0x6008,		/* a080016: str	r0, [r1, #0]*/
	0xbf30,		/* a080018: wfi */
	0xe7fd,		/* 0a08001a: b.n a080018 <loop_wfi> */
	0xed10, 0xe000	/* 0a08001c: System Control Register (SRC) address */
};
#endif

static int stm32_pwr_domain_suspend(const psci_power_state_t *target_state)
{
	uintptr_t pwr_base = stm32mp_pwr_base();
	uintptr_t rcc_base = stm32mp_rcc_base();
	u_register_t mpidr;
	unsigned int core_id;
	bool standby = false;
	uint32_t stateid = stm32_get_stateid(target_state->pwr_domain_state);

	/* Get the mpidr for this cpu */
	mpidr = read_mpidr_el1();
	core_id = MPIDR_AFFLVL0_VAL(mpidr);

	/* request STOP for current core */
	mmio_write_32(rcc_base + RCC_C1SREQSETR,
		      core_id == 0U ? RCC_C1SREQSETR_STPREQ_P0 : RCC_C1SREQSETR_STPREQ_P1);

	/* if retention only at D1 level return as nothing is to be done */
	if (stateid == PWRSTATE_RUN) {
		return PSCI_E_SUCCESS;
	}

	/* force Hold Boot and reset of CPU2 = Cortex M33 */
	mmio_clrbits_32(rcc_base + RCC_CPUBOOTCR, RCC_CPUBOOTCR_BOOT_CPU2);
	mmio_setbits_32(rcc_base + RCC_C2RSTCSETR, RCC_C2RSTCSETR_C2RST);

#if CONFIG_STM32MP25X_REVA
	/* Code for infinite loop in RETRAM for Cortex M33 */
	if (stm32mp_map_retram() != 0) {
		panic();
	}
	/* write in RETRAM the M33 firmware */
	memcpy((void *)RETRAM_BASE, &retram_firmware, sizeof(retram_firmware));

	if (stm32mp_unmap_retram() != 0) {
		panic();
	}
	/* Unblock the HW automaton by M33 running and resetting again */
	mmio_setbits_32(rcc_base + RCC_CPUBOOTCR, RCC_CPUBOOTCR_BOOT_CPU2);
	mmio_setbits_32(rcc_base + RCC_C2RSTCSETR, RCC_C2RSTCSETR_C2RST);
	mmio_clrbits_32(rcc_base + RCC_CPUBOOTCR, RCC_CPUBOOTCR_BOOT_CPU2);
	mmio_setbits_32(rcc_base + RCC_C2RSTCSETR, RCC_C2RSTCSETR_C2RST);
#endif

	/* Switch to Software Self-Refresh mode */
	if (stateid == PWRSTATE_STANDBY) {
		standby = true;
	}

	dcsw_op_all(DCCISW);
	ddr_save_sr_mode();
	ddr_set_sr_mode(DDR_SSR_MODE);
	if (ddr_sr_entry(standby) != 0) {
		panic();
	}

	/* Disable DDRSHR to avoid STANDBY/STOP exit issue */
	mmio_clrbits_32(rcc_base + RCC_DDRITFCFGR, RCC_DDRITFCFGR_DDRSHR);

	/* Perform the PWR configuration for the requested mode */
	switch (stateid) {
	case PWRSTATE_STOP1:
		VERBOSE("enter STOP1\n");
		/* Enable the Non-secure interrupt to wake up the CPU. */
		write_scr_el3(read_scr_el3() | SCR_IRQ_BIT | SCR_FIQ_BIT); /* TBC */

		mmio_write_32(pwr_base + PWR_CPU1CR, 0U);
		mmio_write_32(pwr_base + PWR_CPU2CR, 0U);
		stm32mp2_enable_rcc_wakeup_irq(rcc_base);
		break;

	case PWRSTATE_LP_STOP1:
		VERBOSE("enter LP_STOP1\n");
		mmio_write_32(pwr_base + PWR_CPU1CR, PWR_CPU1CR_LPDS_D1);
		mmio_write_32(pwr_base + PWR_CPU2CR, PWR_CPU2CR_LPDS_D2);
		stm32mp2_enable_rcc_wakeup_irq(rcc_base);
		break;

	case PWRSTATE_LPLV_STOP1:
		VERBOSE("enter LPLV_STOP1\n");
		mmio_write_32(pwr_base + PWR_CPU1CR, PWR_CPU1CR_LPDS_D1 | PWR_CPU1CR_LVDS_D1);
		mmio_write_32(pwr_base + PWR_CPU2CR, PWR_CPU2CR_LPDS_D2 | PWR_CPU2CR_LVDS_D2);
		stm32mp2_enable_rcc_wakeup_irq(rcc_base);
		break;

	case PWRSTATE_STOP2:
		VERBOSE("enter STOP2\n");
		mmio_write_32(pwr_base + PWR_CPU1CR, PWR_CPU1CR_PDDS_D1);
		mmio_write_32(pwr_base + PWR_CPU2CR, 0U);

		stm32mp_gic_cpuif_disable();
		stm32mp2_pll1_disable();
		stm32mp2_enable_rcc_wakeup_irq(rcc_base);
		break;

	case PWRSTATE_LP_STOP2:
		VERBOSE("enter LP_STOP2\n");
		mmio_write_32(pwr_base + PWR_CPU1CR, PWR_CPU1CR_PDDS_D1 | PWR_CPU1CR_LPDS_D1);
		mmio_write_32(pwr_base + PWR_CPU2CR, PWR_CPU2CR_LPDS_D2);

		stm32mp_gic_cpuif_disable();
		stm32mp2_pll1_disable();
		stm32mp2_enable_rcc_wakeup_irq(rcc_base);
		break;

	case PWRSTATE_LPLV_STOP2:
		VERBOSE("enter LPLV_STOP2\n");
		mmio_write_32(pwr_base + PWR_CPU1CR,
			      PWR_CPU1CR_PDDS_D1 | PWR_CPU1CR_LPDS_D1 | PWR_CPU1CR_LVDS_D1);
		mmio_write_32(pwr_base + PWR_CPU2CR, PWR_CPU2CR_LPDS_D2 | PWR_CPU2CR_LVDS_D2);

		stm32mp_gic_cpuif_disable();
		stm32mp2_pll1_disable();
		stm32mp2_enable_rcc_wakeup_irq(rcc_base);
		break;

	case PWRSTATE_STANDBY:
		VERBOSE("enter STANDBY\n");
		mmio_write_32(pwr_base + PWR_CPU1CR, PWR_CPU1CR_PDDS_D1 | PWR_CPU1CR_PDDS_D2);
		mmio_write_32(pwr_base + PWR_CPU2CR, PWR_CPU2CR_PDDS_D2);

		stm32mp_gic_cpuif_disable();
		stm32mp2_pll1_disable();
		break;

	default:
		panic();
		break;
	}

	/* clear previous status */
	mmio_setbits_32(pwr_base + PWR_CPU1CR, PWR_CPU1CR_CSSF);
	mmio_setbits_32(pwr_base + PWR_CPU2CR, PWR_CPU2CR_CSSF); /* TBC */
	mmio_setbits_32(pwr_base + PWR_CPU3CR, PWR_CPU3CR_CSSF);

	return PSCI_E_SUCCESS;
}

/*******************************************************************************
 * STM32MP2 handler called when a power domain has just been powered on after
 * being turned off earlier. The target_state encodes the low power state that
 * each level has woken up from.
 * Called by core 1 just after wake up.
 ******************************************************************************/
static void stm32_pwr_domain_on_finish(const psci_power_state_t *target_state)
{
	uintptr_t rcc_base = stm32mp_rcc_base();
	u_register_t mpidr = read_mpidr();
	unsigned int core_id = MPIDR_AFFLVL0_VAL(mpidr);

	if (core_id == STM32MP_PRIMARY_CPU) {
		stm32mp_core0_go = 0U;
		flush_dcache_range((uintptr_t)&stm32mp_core0_go, sizeof(stm32mp_core0_go));
		dsb();
		isb();
	} else {
		/* restore generic timer after reset */
		stm32mp_stgen_restore_rate();
	}

	stm32mp2_disable_rcc_wakeup_irq(rcc_base);

	stm32mp_gic_pcpu_init();
	stm32mp_gic_cpuif_enable();

	mmio_write_32(rcc_base + RCC_C1SREQCLRR,
		      core_id == 0U ? RCC_C1SREQCLRR_STPREQ_P0 : RCC_C1SREQCLRR_STPREQ_P1);

}

/*******************************************************************************
 * STM32MP2 handler called when a power domain has just been powered on after
 * having been suspended earlier. The target_state encodes the low power state
 * that each level has woken up from.
 ******************************************************************************/
static void stm32_pwr_domain_suspend_finish(const psci_power_state_t
					    *target_state)
{
	uintptr_t rcc_base = stm32mp_rcc_base();
	u_register_t mpidr;
	unsigned int core_id;

	stm32mp2_disable_rcc_wakeup_irq(rcc_base);

	/* Get the mpidr for this cpu */
	mpidr = read_mpidr_el1();
	core_id = MPIDR_AFFLVL0_VAL(mpidr);

	mmio_write_32(rcc_base + RCC_C1SREQCLRR,
		      core_id == 0U ? RCC_C1SREQCLRR_STPREQ_P0 : RCC_C1SREQCLRR_STPREQ_P1);

	/* Restore DDRSHR after STANDBY/STOP exit issue */
	mmio_setbits_32(rcc_base + RCC_DDRITFCFGR, RCC_DDRITFCFGR_DDRSHR);

	/* Perform the common system specific operations */
	if (target_state->pwr_domain_state[LVL_D1_LPLV] == STM32MP_LOCAL_STATE_OFF) {
		/* Restore the DDR self refresh mode */
		ddr_restore_sr_mode();
	} else {
		write_scr_el3(read_scr_el3() & ~(SCR_FIQ_BIT | SCR_IRQ_BIT));
		isb();

		/* Exit DDR self refresh mode after STOP mode */
		ddr_sr_exit();
		ddr_restore_sr_mode();

		/* Clear RCC CPU1 wake-up interrupt : TODO */

		stm32mp_gic_cpuif_enable();
	}
}

static void __dead2 stm32_pwr_domain_pwr_down_wfi(const psci_power_state_t
						  *target_state)
{
	u_register_t mpidr = read_mpidr();
	unsigned int core_id = MPIDR_AFFLVL0_VAL(mpidr);
	uintptr_t rcc_base = stm32mp_rcc_base();

	/* The first power down on core 0, core 1 is running */
	if ((mmio_read_32(rcc_base + RCC_C1SREQSETR) == 0U) && (core_id == STM32MP_PRIMARY_CPU)) {
		uintptr_t sec_entrypoint;

		/* Request STOP for current core */
		mmio_write_32(rcc_base + RCC_C1SREQSETR, RCC_C1SREQSETR_STPREQ_P0);

		/* Core 0 can't be turned OFF, emulate it with a WFE loop */
		VERBOSE("BL31: core0 entering wait loop...\n");
		while (stm32mp_core0_go == 0U) {
			wfe();
		}

		VERBOSE("BL31: core0 resumed.\n");
		dsbsy();
		sec_entrypoint = mmio_read_32(BSEC_BASE + BSEC_SCRATCHR0);
		/* Jump manually to entry point, with mmu disabled. */
		disable_mmu_el3();
		((void(*)(void))sec_entrypoint)();

		/* This shouldn't be reached */
		panic();
	}

	/* Request STOP for current core */
	mmio_write_32(rcc_base + RCC_C1SREQSETR,
		      core_id == 0U ? RCC_C1SREQSETR_STPREQ_P0 : RCC_C1SREQSETR_STPREQ_P1);

	if (((mmio_read_32(rcc_base + RCC_C1SREQSETR) &
	      RCC_C1SREQSETR_STPREQ_MASK) == RCC_C1SREQSETR_STPREQ_MASK)) {
		/* Save the context when all the core are requested to stop */
		stm32_pm_context_save(target_state);
	}

	/*
	 * Synchronize on memory accesses and instruction flow before
	 * auto-reset from the WFI instruction.
	 */
	dsb();
	isb();
	wfi();

	/* This shouldn't be reached */
	panic();
}

static void __dead2 stm32_system_off(void)
{
	uintptr_t pwr_base = stm32mp_pwr_base();
	uintptr_t rcc_base = stm32mp_rcc_base();

	/* Prevent interrupts from spuriously waking up this cpu */
	stm32mp_gic_cpuif_disable();

	/* Program the power controller to enable wakeup interrupts. */
	// stm32_pwr_set_wakeup(mpidr);
	// TODO ? RCC_C1CIESETR.

	/* force Hold Boot and reset of CPU2 = Cortex M33 */
	mmio_clrbits_32(rcc_base + RCC_CPUBOOTCR, RCC_CPUBOOTCR_BOOT_CPU2);
	mmio_setbits_32(rcc_base + RCC_C2RSTCSETR, RCC_C2RSTCSETR_C2RST);

	/* request standby, normally with DDR off */
	mmio_setbits_32(pwr_base + PWR_CPU1CR, PWR_CPU1CR_PDDS_D1);
	mmio_setbits_32(pwr_base + PWR_CPU1CR, PWR_CPU1CR_PDDS_D2);
	dsb();
	isb();
	wfi();

	/* This shouldn't be reached */
	panic();
}

static void __dead2 stm32_system_reset(void)
{
	stm32mp_system_reset();
}

/**
 * stm32_validate_power_state() - This function ensures that the power state
 * parameter in request is valid.
 *
 * @power_state		Power state of core
 * @req_state		Requested state
 *
 * @return	Returns status, either success or reason
 */
static int stm32_validate_power_state(unsigned int power_state,
				      psci_power_state_t *req_state)
{
	unsigned int state_id;
	unsigned int i;

	assert(req_state != NULL);

	/*
	 *  Currently we are using a linear search for finding the matching
	 *  entry in the idle power state array. This can be made a binary
	 *  search if the number of entries justify the additional complexity.
	 */
	for (i = 0U; stm32mp_pm_idle_states[i] != 0U; i++) {
		if (power_state == stm32mp_pm_idle_states[i]) {
			break;
		}
	}

	/* Return error if entry not found in the idle state array */
	if (stm32mp_pm_idle_states[i] == 0U) {
		return PSCI_E_INVALID_PARAMS;
	}

	/* search if board restrict the number of supported modes */
	for (i = 0U; stm32mp_supported_pwr_states[i] != 0U; i++) {
		if (power_state == stm32mp_supported_pwr_states[i]) {
			break;
		}
	}
	if (stm32mp_supported_pwr_states[i] == 0U) {
		ERROR("PSCI power state not supported %x\n", power_state);
		return PSCI_E_INVALID_PARAMS;
	}

	i = 0U;
	state_id = psci_get_pstate_id(power_state);

	/* Parse the State ID and populate the state info parameter */
	while (state_id != 0U) {
		req_state->pwr_domain_state[i++] = state_id & PLAT_LOCAL_PSTATE_MASK;
		state_id >>= PLAT_LOCAL_PSTATE_WIDTH;
	}

	return PSCI_E_SUCCESS;
}

static int stm32_validate_ns_entrypoint(uintptr_t entrypoint)
{
	/* The non-secure entry point must be in DDR */
	if (entrypoint < STM32MP_DDR_BASE) {
		return PSCI_E_INVALID_ADDRESS;
	}

	return PSCI_E_SUCCESS;
}

/**
 * stm32_get_sys_suspend_power_state() -  Get power state for system suspend
 *
 * @req_state	Requested state
 */
static void stm32_get_sys_suspend_power_state(psci_power_state_t *req_state)
{
	unsigned int state_id;
	unsigned int i;

	/* search the max supported modes */
	for (i = ARRAY_SIZE(stm32mp_supported_pwr_states) - 1U; i > 0U; i--) {
		if ((stm32mp_supported_pwr_states[i] != 0U) &&
		    (psci_get_pstate_type(stm32mp_supported_pwr_states[i]) ==
		     PSTATE_TYPE_POWERDOWN)) {
			break;
		}
	}
	state_id = psci_get_pstate_id(stm32mp_supported_pwr_states[i]);

	/* Parse the State ID and populate the state info parameter */
	for (i = 0U; i <= PLAT_MAX_PWR_LVL; i++) {
		req_state->pwr_domain_state[i] = state_id & PLAT_LOCAL_PSTATE_MASK;
		state_id >>= PLAT_LOCAL_PSTATE_WIDTH;
	}
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
	.get_sys_suspend_power_state = stm32_get_sys_suspend_power_state,
};

/*******************************************************************************
 * This function retrieve the generic information from DT.
 * Returns node on success and a negative FDT error code on failure.
 ******************************************************************************/
static int stm32_parse_domain_idle_state(void)
{
	unsigned int domain_idle_states[PM_IDLE_STATES_SIZE];
	void *fdt = NULL;
	int node = 0;
	int subnode = 0;
	uint32_t power_state;
	unsigned int i = 0U;
	unsigned int j = 0U;
	unsigned int nb_states = 0U;
	int ret;

	if (fdt_get_address(&fdt) == 0) {
		return -ENODEV;
	}

	/* search supported domain-idle-state in device tree */
	node = fdt_path_offset(fdt, "/domain-idle-states");
	if (node < 0) {
		return node;
	}

	memset(domain_idle_states, 0, sizeof(domain_idle_states));
	fdt_for_each_subnode(subnode, fdt, node) {
		ret = fdt_read_uint32(fdt, subnode, "arm,psci-suspend-param", &power_state);
		if (ret != 0) {
			return ret;
		}

		/* cross check with supported pm idle state in driver */
		for (j = 0U; stm32mp_pm_idle_states[j] != 0U; j++) {
			if (stm32mp_pm_idle_states[j] == power_state) {
				break;
			}
		}
		/* Return error if entry not found and not standby */
		if ((stm32mp_pm_idle_states[j] == 0U) && (power_state != PWRSTATE_STANDBY)) {
			return -EINVAL;
		}

		domain_idle_states[i++] = power_state;

		/* check array size */
		if (i > ARRAY_SIZE(domain_idle_states)) {
			return -E2BIG;
		}
	}

	memset(stm32mp_supported_pwr_states, 0, sizeof(stm32mp_supported_pwr_states));

	/* The CPU idle state is always supported, not present in domain node */
	stm32mp_supported_pwr_states[nb_states++] = PWRSTATE_RUN;

	/* Add each supported power state with same order than stm32mp_pm_idle_states */
	for (j = 0U; stm32mp_pm_idle_states[j] != 0U; j++) {
		for (i = 0U; domain_idle_states[i] != 0U && i < PM_IDLE_STATES_SIZE; i++) {
			if (domain_idle_states[i] == stm32mp_pm_idle_states[j]) {
				stm32mp_supported_pwr_states[nb_states++] = domain_idle_states[i];
				break;
			}
		}
	}

	/* Add STANDBY at the end of the list if supported */
	for (i = 0U; domain_idle_states[i] != 0U && i < PM_IDLE_STATES_SIZE; i++) {
		if (domain_idle_states[i] == PWRSTATE_STANDBY) {
			stm32mp_supported_pwr_states[nb_states++] = PWRSTATE_STANDBY;
			break;
		}
	}

	VERBOSE("stm32mp_supported_pwr_states[]=\n");
	for (i = 0U; stm32mp_supported_pwr_states[i] != 0U; i++) {
		VERBOSE("%u = %x\n", i, stm32mp_supported_pwr_states[i]);
	}

	return 0;
}

/*******************************************************************************
 * Initialize STM32MP2 for PM support: RCC, PWR
 ******************************************************************************/
static void stm32_pm_init(void)
{
	uintptr_t pwr_base = stm32mp_pwr_base();
	uintptr_t rcc_base = stm32mp_rcc_base();
	uint32_t lsmcu;
	uint32_t lpcfg_d2 = 0U;

	/* RCC init: DDR is shared by default */
	mmio_setbits_32(rcc_base + RCC_DDRITFCFGR, RCC_DDRITFCFGR_DDRSHR);

	/* Legacy mode: only CPU1 is allowed to boot, core1 is OFF */
	mmio_setbits_32(rcc_base + RCC_LEGBOOTCR, RCC_LEGBOOTCR_LEGACY_BEN);

	mmio_write_32(rcc_base + RCC_C1SREQCLRR, RCC_C1SREQCLRR_STPREQ_P0);
	mmio_write_32(rcc_base + RCC_C1SREQSETR, RCC_C1SREQSETR_STPREQ_P1);

	/* PWR register init */
	mmio_write_32(pwr_base + PWR_CPU1CR, 0U);
	mmio_write_32(pwr_base + PWR_CPU2CR, 0U);

	/* Maintain BKPSRAM & RETRAM content in Standby */
	mmio_write_32(pwr_base + PWR_CR9, PWR_CR9_BKPRBSEN);
	mmio_write_32(pwr_base + PWR_CR10, PWR_CR10_RETRBSEN_STANDBY);

	/* Prevent RETRAM erase */
	mmio_write_32(RAMCFG_BASE + RAMCFG_RETRAMCR, SRAMHWERDIS);

	/*
	 * if PWR_D1CR_LPCFG_D1 is set to 0,
	 *      PWR_CPU_ON signal is low when CPU1 is in standby or stop2/stop3
	 * else only standby
	 *  should be set for PMIC
	 *
	 * and set PWR_D1CR_POPL_D1 = 3ms
	 */
	mmio_write_32(pwr_base + PWR_D1CR, (3 << PWR_D1CR_POPL_D1_SHIFT));

	/*
	 * Set PWR_D2CR_LPCFG_D2 for PMIC board
	 * And PWR_ON signals all modes on pmic
	 * - PWR_D2CR_POPL_D2 = 2ms
	 * - PWR_D2CR_PODH_D2 = 1ms
	 * - PWR_D2CR_LPLVDLY_D2 = 375us
	 */
	if (dt_pmic_status() > 0) {
		/* On PMIC board, control LPCFG */
		lpcfg_d2 = PWR_D2CR_LPCFG_D2;
	}
	mmio_write_32(pwr_base + PWR_D2CR, lpcfg_d2 |
					   (2 << PWR_D2CR_POPL_D2_SHIFT) |
					   (1 << PWR_D2CR_LPLVDLY_D2_SHIFT) |
					   (1 << PWR_D2CR_PODH_D2_SHIFT));

	/* system standby not set (D3) */
	mmio_write_32(pwr_base + PWR_D3CR, 0U);

	/* To confirmed: delay can be customizable or keep hardcoded at 1ms / 2ms */

	/* set DLY to 1ms */
	lsmcu = mmio_read_32(rcc_base + RCC_LSMCUDIVR) & RCC_LSMCUDIVR_LSMCUDIV;
	mmio_write_32(rcc_base + RCC_PWRLPDLYCR, PWRLPDLYCR_VAL(1000U, lsmcu));
}

/*******************************************************************************
 * Export the platform specific power ops.
 ******************************************************************************/
int plat_setup_psci_ops(uintptr_t sec_entrypoint,
			const plat_psci_ops_t **psci_ops)
{
	int ret = 0;

	ret = stm32_parse_domain_idle_state();
	if (ret != 0) {
		ERROR("invalid domain-idle-states in device tree %d\n", ret);
		panic();
	}

	/* Program secondary CPU entry points. */
	mmio_write_32(A35SSC_BASE + CA35SS_SYSCFG_VBAR_CR, sec_entrypoint);

	/* core 0 can't be turned OFF, emulate it with a WFE loop */
	stm32mp_core0_go = 0U;

	/* Save boot entry point for STOP2 exit */
	mmio_write_32(BSEC_BASE + BSEC_SCRATCHR0, sec_entrypoint);

	stm32_pm_init();

	*psci_ops = &stm32_psci_ops;

	return 0;
}
