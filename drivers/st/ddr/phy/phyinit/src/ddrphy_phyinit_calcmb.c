/*
 * Copyright (C) 2021-2022, STMicroelectronics - All Rights Reserved
 *
 * SPDX-License-Identifier: BSD-3-Clause
 */

#include <stdlib.h>

#include <common/debug.h>

#include <ddrphy_phyinit.h>

/*
 * Reads PhyInit inputs structures and sets relevant message block
 * parameters.
 *
 * This function sets Message Block parameters based on user_input_basic and
 * user_input_advanced. user changes in these files takes precedence
 * over this function call.
 *
 * MessageBlock fields set ::
 *
 *  - dramtype
 *  - pstate
 *  - dramfreq
 *  - pllbypassen
 *  - dfifreqratio
 *  - phyodtimpedance
 *  - phydrvimpedance
 *  - bpznresval
 *  - enableddqscha (LPDDR4)
 *  - cspresentcha (LPDDR4)
 *  - enableddqsChb (LPDDR4)
 *  - cspresentchb (LPDDR4)
 *  - enableddqs (DDR3/DDR4)
 *  - phycfg (DDR3/DDR4)
 *  - x16present (DDR4)
 *
 * \return void
 */
void ddrphy_phyinit_calcmb(void)
{
	int myps = 0;
	int nad0 = userinputbasic.numactivedbytedfi0;
	int nad1 = 0;
	uint16_t mr4 __unused;

	VERBOSE("%s Start\n", __func__);

#if STM32MP_LPDDR4_TYPE
	nad1 = userinputbasic.numactivedbytedfi1;
#endif /* STM32MP_LPDDR4_TYPE */

	/* A few checks to make sure valid programming */
	if (nad0 <= 0 || nad1 < 0 || userinputbasic.numdbyte <= 0) {
		ERROR("%s numactivedbytedfi0, numactivedbytedfi0, NumByte out of range.\n",
		      __func__);
	}

	if ((nad0 + nad1) > userinputbasic.numdbyte) {
		ERROR("%s numactivedbytedfi0+numactivedbytedfi1 is larger than numdbyteDfi0\n",
		      __func__);
	}

	if (userinputbasic.dfi1exists == 0 && nad1 != 0) {
		ERROR("%s dfi1exists==0 but numdbyteDfi0 != 0\n", __func__);
	}

#if STM32MP_DDR4_TYPE
	/* OR all mr4 masked values, to help check in next loop */
	mr4 = 0;
	for (myps = 0; myps < userinputbasic.numpstates; myps++) {
		mr4 |= mb_ddr_1d[myps].mr4 & 0x1C0U;
	}
#endif /* STM32MP_DDR4_TYPE */

	/* 1D message block defaults */
	for (myps = 0; myps < userinputbasic.numpstates; myps++) {
		uint16_t disableddbyte __unused;
		int dbyte __unused;

#if STM32MP_DDR4_TYPE
		if (mr4 != 0x0) {
			ERROR("%s Setting DRAM CAL mode is not supported by the PHY.\n", __func__);
			ERROR("Memory controller may set CAL mode after PHY has entered mission\n");
			ERROR("mode. Please check value programmed in mb_ddr_1d[*].mr4\n");
			ERROR("and unset A8:6\n");
		}
#endif /* STM32MP_DDR4_TYPE */

#if STM32MP_DDR3_TYPE
		if (userinputbasic.dimmtype == NODIMM) {
			ddrphy_phyinit_softsetmb(myps,"dramtype",0x1);
		}
#elif STM32MP_DDR4_TYPE
		if (userinputbasic.dimmtype == NODIMM) {
			ddrphy_phyinit_softsetmb(myps,"dramtype",0x2);
		}
#endif /* STM32MP_DDR4_TYPE */

		ddrphy_phyinit_softsetmb(myps, "pstate", myps);
		ddrphy_phyinit_softsetmb(myps, "dramfreq", userinputbasic.frequency[myps] * 2);
		ddrphy_phyinit_softsetmb(myps, "pllbypassen", userinputbasic.pllbypass[myps]);

		if (userinputbasic.dfifreqratio[myps] == 1) {
			ddrphy_phyinit_softsetmb(myps, "dfifreqratio", 0x2);
		}

		ddrphy_phyinit_softsetmb(myps, "phyodtimpedance", 0);
		ddrphy_phyinit_softsetmb(myps, "phydrvimpedance", 0);
		ddrphy_phyinit_softsetmb(myps, "bpznresval", 0);

#if STM32MP_DDR3_TYPE || STM32MP_DDR4_TYPE
		ddrphy_phyinit_softsetmb(myps,"enableddqs",nad0 * 8);

		disableddbyte = 0x0U;

		for (dbyte = 0; dbyte < userinputbasic.numdbyte && dbyte < 8; dbyte++) {
			disableddbyte |= (ddrphy_phyinit_isdbytedisabled(dbyte) ?
									(0x1U << dbyte) : 0x0U);
		}
		ddrphy_phyinit_softsetmb(myps, "disableddbyte", disableddbyte);
#if STM32MP_DDR3_TYPE
		ddrphy_phyinit_softsetmb(myps, "phycfg", userinputadvanced.is2ttiming[myps]);
#else
		ddrphy_phyinit_softsetmb(myps, "phycfg", (mb_ddr_1d[myps].mr3 & 0x8U) ?
							0 : userinputadvanced.is2ttiming[myps]);
		ddrphy_phyinit_softsetmb(myps, "x16present",
					 (0x10 == userinputbasic.dramdatawidth) ?
								mb_ddr_1d[myps].cspresent : 0x0);
#endif /* STM32MP_DDR3_TYPE */
#elif STM32MP_LPDDR4_TYPE
		ddrphy_phyinit_softsetmb(myps, "enableddqscha", nad0 * 8);
		ddrphy_phyinit_softsetmb(myps, "cspresentcha", (2 == userinputbasic.numrank_dfi0) ?
								0x3 : userinputbasic.numrank_dfi0);
		ddrphy_phyinit_softsetmb(myps, "enableddqschb", nad1 * 8);
		ddrphy_phyinit_softsetmb(myps, "cspresentchb", (2 == userinputbasic.numrank_dfi1) ?
								0x3 : userinputbasic.numrank_dfi1);
#endif /* STM32MP_LPDDR4_TYPE */
	} /* myps */

	VERBOSE("%s End\n", __func__);
}
