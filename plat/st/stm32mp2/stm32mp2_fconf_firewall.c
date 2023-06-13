/*
 * Copyright (C) 2023, STMicroelectronics - All Rights Reserved
 *
 * SPDX-License-Identifier: BSD-3-Clause
 */

#include <platform_def.h>

#include <lib/fconf/fconf.h>
#include <lib/object_pool.h>
#include <tools_share/firmware_image_package.h>

void stm32mp2_arch_security_setup(void)
{
}

void stm32mp2_security_setup(void)
{
}

static int fconf_populate_stm32mp2_firewall(uintptr_t config)
{
	return 0;
}

FCONF_REGISTER_POPULATOR(FW_CONFIG, stm32mp2_firewall, fconf_populate_stm32mp2_firewall);
