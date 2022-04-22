/*
 * Copyright (c) 2023, STMicroelectronics - All Rights Reserved
 *
 * SPDX-License-Identifier: BSD-3-Clause
 */

#ifndef STM32MP2_PRIVATE_H
#define STM32MP2_PRIVATE_H

void configure_mmu(void);

void stm32mp2_arch_security_setup(void);
void stm32mp2_security_setup(void);

uint32_t stm32mp2_syscfg_get_chip_version(void);
uint32_t stm32mp2_syscfg_get_chip_dev_id(void);
size_t stm32mp2_syscfg_get_mm_size(uint8_t bank);

/* IO compensation identifiers */
enum syscfg_io_ids {
	SYSFG_VDDIO1_ID,
	SYSFG_VDDIO2_ID,
	SYSFG_VDDIO3_ID,
	SYSFG_VDDIO4_ID,
	SYSFG_VDD_IO_ID,
	SYSFG_NB_IO_ID
};

void stm32mp2_syscfg_enable_io_compensation(enum syscfg_io_ids id);

int stm32mp2_pwr_init_io_domains(void);

/* Get RISAF platform instance ID from peripheral IO memory base address */
int stm32_risaf_get_instance(uintptr_t base);

/* Get RISAF peripheral IO memory base address from platform instance ID */
uintptr_t stm32_risaf_get_base(int instance);

/* Get RISAF maximum number of regions from platform instance ID */
int stm32_risaf_get_max_region(int instance);

/* Get RISAF memory base address from platform instance ID */
uintptr_t stm32_risaf_get_memory_base(int instance);

/* Get RISAF memory size in bytes from platform instance ID */
size_t stm32_risaf_get_memory_size(int instance);

/* Get DDRDBG peripheral IO memory base address */
uintptr_t stm32_ddrdbg_get_base(void);

#endif /* STM32MP2_PRIVATE_H */
