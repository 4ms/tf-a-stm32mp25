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
#define SYSCFG_OCTOSPIAMCR		0x2C00U
#define SYSCFG_DEVICEID			0x6400U

/*
 * SYSCFG_OCTOSPIAMCR Register
 */
#define SYSCFG_OCTOSPIAMCR_OAM_MASK	GENMASK_32(2, 0)

/*
 * SYSCFG_DEVICEID Register
 */
#define SYSCFG_DEVICEID_DEV_ID_MASK	GENMASK_32(11, 0)
#define SYSCFG_DEVICEID_REV_ID_MASK	GENMASK_32(31, 16)
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

size_t stm32mp2_syscfg_get_mm_size(uint8_t bank)
{
	uint32_t amcr = mmio_read_32(SYSCFG_BASE + SYSCFG_OCTOSPIAMCR);
	size_t addr_mapping1 = 0U;
	size_t addr_mapping2 = 0U;

	switch (amcr & SYSCFG_OCTOSPIAMCR_OAM_MASK) {
	case 0:
		addr_mapping1 = SZ_256M;
		break;
	case 1:
		addr_mapping1 = SZ_128M + SZ_64M;
		addr_mapping2 = SZ_64M;
		break;
	case 2:
		addr_mapping1 = SZ_128M;
		addr_mapping2 = SZ_128M;
		break;
	case 3:
		addr_mapping1 = SZ_64M;
		addr_mapping2 = SZ_128M + SZ_64M;
		break;
	default:
		addr_mapping2 = SZ_256M;
		break;
	}

	return (bank == 0U) ? addr_mapping1 : addr_mapping2;
}
