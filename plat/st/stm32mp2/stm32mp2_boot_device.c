/*
 * Copyright (c) 2021-2022, STMicroelectronics - All Rights Reserved
 *
 * SPDX-License-Identifier: BSD-3-Clause
 */

#include <assert.h>
#include <errno.h>

#include <drivers/nand.h>
#include <drivers/raw_nand.h>
#include <drivers/spi_nand.h>
#include <drivers/spi_nor.h>
#include <lib/utils.h>
#include <plat/common/platform.h>

#if STM32MP_RAW_NAND || STM32MP_SPI_NAND
static int get_data_from_otp(struct nand_device *nand_dev, bool is_slc)
{
	/* To do: implement otp parsing */

	return 0;
}
#endif /* STM32MP_RAW_NAND || STM32MP_SPI_NAND */

#if STM32MP_RAW_NAND
int plat_get_raw_nand_data(struct rawnand_device *device)
{
	device->nand_dev->ecc.mode = NAND_ECC_HW;
	device->nand_dev->ecc.size = SZ_512;

	return get_data_from_otp(device->nand_dev, true);
}
#endif

#if STM32MP_SPI_NAND
int plat_get_spi_nand_data(struct spinand_device *device)
{
	return get_data_from_otp(device->nand_dev, false);
}
#endif

#if STM32MP_SPI_NOR
int plat_get_nor_data(struct nor_device *device)
{
	device->size = SZ_64M;

	return 0;
}
#endif
