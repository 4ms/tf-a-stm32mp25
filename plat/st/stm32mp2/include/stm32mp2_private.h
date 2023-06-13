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

#endif /* STM32MP2_PRIVATE_H */
