/*
 * Copyright (C) 2021-2022, STMicroelectronics - All Rights Reserved
 *
 * SPDX-License-Identifier: BSD-3-Clause
 */

#include <stdlib.h>
#include <stdio.h>

#include <common/debug.h>

#include <ddrphy_phyinit.h>

/*
 * This function loads the training firmware DMEM image and write the
 * Message Block parameters for the training firmware into the SRAM.
 *
 * This function performs the following tasks:
 *
 * -# Load the firmware DMEM segment to initialize the data structures from the
 * DMEM incv file provided in the training firmware package.
 * -# Write the Firmware Message Block with the required contents detailing the training parameters.
 *
 * \return void
 */
void ddrphy_phyinit_f_loaddmem(int pstate)
{
	pmu_smb_ddr_1d_t *msgblkptr;
	int addr;
	int mem_offset = 0;
	int sizeofmsgblk;
	int mem[DMEM_SIZE];
	/* return_offset_lastaddr_t return_type = return_lastaddr; */

	VERBOSE("%s Start (pstate=%d)\n", __func__, pstate);

	/* Set a pointer to the message block */
	msgblkptr = &mb_ddr_1d[pstate];

	/* Some basic checks on MessageBlock */
#if STM32MP_DDR3_TYPE || STM32MP_DDR4_TYPE
	if ((msgblkptr->enableddqs > 8 * (userinputbasic.numactivedbytedfi0)) ||
	    (msgblkptr->enableddqs <= 0)) {
		ERROR("%s enableddqs is Zero or greater than NumActiveDbytes for Dfi0\n",__func__);
	}
#elif STM32MP_LPDDR4_TYPE
	if (msgblkptr->enableddqscha % 16 != 0 || msgblkptr->enableddqschb % 16 != 0) {
		ERROR("%s Lp3/Lp4 - Number of Dq's Enabled per Channel much be multipe of 16\n",
		      __func__);
	}

	if ((msgblkptr->enableddqscha > 8 * (userinputbasic.numactivedbytedfi0)) ||
	    (msgblkptr->enableddqschb > 8 * (userinputbasic.numactivedbytedfi1)) ||
	    (msgblkptr->enableddqscha == 0 && msgblkptr->enableddqschb == 0)) {
		ERROR("%s EnabledDqsChA/B are not set correctly./1\n", __func__);
	}
#endif /* STM32MP_LPDDR4_TYPE */

	/* initialize the dmem structure */
	for (addr = 0; addr < DMEM_SIZE; addr++) {
		mem[addr] = 0;
	}

	/* mem_offset = ddrphy_phyinit_storeincvfile(DMEM_INCV_FILENAME, mem, return_type); */
	sizeofmsgblk = sizeof(mb_ddr_1d[pstate]);
	ddrphy_phyinit_storemsgblk(&(mb_ddr_1d[pstate]), sizeofmsgblk, mem);

	/* Write local dmem array */
	if (0 == (mem_offset % 1)) {
		/*Always write an even number of words so no 32bit quantity is uninitialized */
		mem_offset++;
	}

	ddrphy_phyinit_writeoutmem(mem, DMEM_ST_ADDR, (mem_offset - DMEM_ST_ADDR));

	VERBOSE("%s End\n", __func__);
}
