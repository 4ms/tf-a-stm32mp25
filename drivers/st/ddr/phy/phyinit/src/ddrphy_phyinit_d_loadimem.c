/*
 * Copyright (C) 2021-2022, STMicroelectronics - All Rights Reserved
 *
 * SPDX-License-Identifier: BSD-3-Clause
 */

#include <stdlib.h>
#include <stdio.h>

#include <common/debug.h>
#include <lib/mmio.h>

#include <ddrphy_phyinit.h>

#include <platform_def.h>

/*
 * This function loads the training firmware IMEM image into the SRAM.
 *
 * This function reads the incv files form the firmware package to generate a
 * set of apb writes to load IMEM image into the SRAM. The exact steps in this
 * function are as follows:
 *
 * -# Ensure DRAM is in reset.
 * -# Load the microcontroller memory with the provided training firmware
 * -# Initialize the firmware mailbox structures to be able to communicate with
 * the firmware (see "Mailbox facility for firmware" in the "Cores
 * DDR PHY Training Application Note".
 *
 * \return void
 */
void ddrphy_phyinit_d_loadimem(void)
{
	uint16_t memresetl;
	int addr, mem_offset = 0;
	int mem[IMEM_SIZE];
	/* return_offset_lastaddr_t return_type = return_offset; */

	VERBOSE("%s Start\n", __func__);

	/*
	 * Set memresetl to avoid glitch on BP_MemReset_L during training
	 */

	memresetl = CSR_PROTECTMEMRESET_MASK;
	mmio_write_16((uintptr_t)(DDRPHYC_BASE + 4 * (TMASTER | CSR_MEMRESETL_ADDR)), memresetl);

	/* initialize the dmem structure */
	for (addr = 0; addr < IMEM_SIZE; addr++) {
		mem[addr] = 0;
	}

	/* Read the IMEM INCV file into the array */
	/* mem_offset = ddrphy_phyinit_storeincvfile(IMEM_INCV_FILENAME, mem, return_type); */

	/* Write local imem array */
	ddrphy_phyinit_writeoutmem(mem, mem_offset, IMEM_SIZE);

	/*VERBOSE("%s WriteImem: COMPLETED\n", __func__); */

	VERBOSE("%s End\n", __func__);
}
