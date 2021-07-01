/*
 * Copyright (c) 2021, STMicroelectronics - All Rights Reserved
 *
 * SPDX-License-Identifier: GPL-2.0+ OR BSD-3-Clause
 */

#ifndef STM32MP2_DDR_REGS_H
#define STM32MP2_DDR_REGS_H

#include <drivers/st/stm32mp_ddrctrl_regs.h>
#include <lib/utils_def.h>

/* DDR Physical Interface Control (DDRPHYC) registers*/
struct stm32mp_ddrphy {
	uint32_t dummy;
} __packed;

#endif /* STM32MP2_DDR_REGS_H */
