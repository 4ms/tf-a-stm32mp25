/*
 * Copyright (C) 2021-2022, STMicroelectronics - All Rights Reserved
 *
 * SPDX-License-Identifier: BSD-3-Clause
 */

#include <arch_helpers.h>

#include <common/debug.h>

#include <ddrphy_phyinit.h>

/*
 * This function implements the flow of PhyInit software to initialize the PHY.
 *
 * The execution sequence follows the overview figure provided in the PhyInit App Note.
 *
 * \returns 0 on completion of the sequence, EXIT_FAILURE on error.
 */
int ddrphy_phyinit_sequence(bool skip_training)
{
	/* Check user input pstate number consistency vs. SW capabilities */
	if (userinputbasic.numpstates > NB_PS) {
		return -1;
	}

	/* registering function inputs */
	runtimeconfig.skip_train = skip_training;

	VERBOSE("%s Start\n", __func__);

	/* Initialize structures */
	ddrphy_phyinit_initstruct();

	/* Re-calculate Firmware Message Block input based on final user input */
	ddrphy_phyinit_calcmb();

	/* (A) Bring up VDD, VDDQ, and VAA */
	/* ddrphy_phyinit_usercustom_a_bringuppower(); */

	/* (B) Start Clocks and Reset the PHY */
	/* ddrphy_phyinit_usercustom_b_startclockresetphy(); */

	/* (C) Initialize PHY Configuration */
	ddrphy_phyinit_c_initphyconfig();

	/*
	 * Customize any register write desired; This can include any CSR not covered by PhyInit
	 * or user wish to override values calculated in step_C
	 */
	ddrphy_phyinit_usercustom_custompretrain();

	/* Stop retention register tracking for training firmware related registers */
	ddrphy_phyinit_reginterface(stoptrack, 0, 0);

#if STM32MP_DDR_SKIP_TRAINING
	if (skip_training) {
		/* Skip running training firmware entirely */
		ddrphy_phyinit_progcsrskiptrain(skip_training);
	}
#else
	{
		int pstate;

		/* Run all 1D power states, then 2D P0, to reduce total Imem/Dmem loads. */

		/* (D) Load the IMEM Memory for 1D training */
		ddrphy_phyinit_d_loadimem();

		for (pstate = 0; pstate < userinputbasic.numpstates; pstate++) {
			/* (E) Set the PHY input clocks to the desired frequency */
			/* ddrphy_phyinit_usercustom_e_setdficlk(pstate); */

			/*
			 * Note: this routine implies other items such as dfifreqratio, DfiCtlClk
			 * are also set properly.
			 * Because the clocks are controlled in the SOC, external to the software
			 * and PHY, this step intended to be replaced by the user with the necessary
			 * SOC operations to achieve the new input frequency to the PHY.
			 */

			/* (F) Write the Message Block parameters for the training firmware */
			ddrphy_phyinit_f_loaddmem(pstate);

			/* (G) Execute the Training Firmware */
			ddrphy_phyinit_g_execfw();

			/* (H) Read the Message Block results */
			ddrphy_phyinit_h_readmsgblock();
		}
	}
#endif /* STM32MP_DDR_SKIP_TRAINING */

	/* Start retention register tracking for training firmware related registers */
	ddrphy_phyinit_reginterface(starttrack, 0, 0);

	/* (I) Load PHY Init Engine Image */
	ddrphy_phyinit_i_loadpieimage(skip_training);

	/*
	 * Customize any CSR write desired to override values programmed by firmware or
	 * ddrphy_phyinit_i_loadpieimage()
	 */
	ddrphy_phyinit_usercustom_customposttrain();

	if (runtimeconfig.reten) {
		/* Save value of tracked registers for retention restore sequence. */
		/* ddrphy_phyinit_usercustom_saveretregs(); */
	}

	/* (J) Initialize the PHY to Mission Mode through DFI Initialization */
	/* ddrphy_phyinit_usercustom_j_entermissionmode(); */

	VERBOSE("%s End\n", __func__);

	return 0;
}
