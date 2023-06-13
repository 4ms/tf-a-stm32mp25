/*
 * Copyright (c) 2023, STMicroelectronics - All Rights Reserved
 *
 * SPDX-License-Identifier: BSD-3-Clause
 */

#include <platform_def.h>

#include <common/debug.h>
#include <lib/mmio.h>
#include <lib/utils_def.h>

#include <stm32mp2_private.h>

/*
 * SYSCFG register offsets (base relative)
 */
#define SYSCFG_DEVICEID			0x6400U

/*
 * SYSCFG_DEVICEID Register
 */
#define SYSCFG_DEVICEID_DEV_ID_MASK	GENMASK(11, 0)
#define SYSCFG_DEVICEID_REV_ID_MASK	GENMASK(31, 16)
#define SYSCFG_DEVICEID_REV_ID_SHIFT	16

/*
 * @brief  Get silicon revision from SYSCFG registers.
 * @retval chip version (REV_ID).
 */
uint32_t stm32mp2_syscfg_get_chip_version(void)
{
	return (mmio_read_32(SYSCFG_BASE + SYSCFG_DEVICEID) &
		SYSCFG_DEVICEID_REV_ID_MASK) >> SYSCFG_DEVICEID_REV_ID_SHIFT;
}

/*
 * @brief  Get device ID from SYSCFG registers.
 * @retval device ID (DEV_ID).
 */
uint32_t stm32mp2_syscfg_get_chip_dev_id(void)
{
	return mmio_read_32(SYSCFG_BASE + SYSCFG_DEVICEID) & SYSCFG_DEVICEID_DEV_ID_MASK;
}
