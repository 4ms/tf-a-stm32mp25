/*
 * Copyright (C) 2021-2022, STMicroelectronics - All Rights Reserved
 *
 * SPDX-License-Identifier: BSD-3-Clause
 */

#include <stdio.h>
#include <stdlib.h>

#include <common/debug.h>
#include <lib/mmio.h>

#include <ddrphy_phyinit_usercustom.h>

#include <platform_def.h>

/*
 * This function is called before training firmware is executed. Any
 * register override in this function might affect the firmware training
 * results.
 *
 * This function is executed before firmware execution loop. Thus this function
 * should be used only for the following:
 *
 *  - Override PHY register values written by
 *  ddrphy_phyinit_c_initphyconfig.  An example use case is when this
 *  function does not perform the exact programing desired by the user.
 *  - Write custom PHY registers that need to take effect before training
 *  firmware execution.
 *
 * User shall use mmio_write_16 to write PHY registers in order for the register
 * to be tracked by PhyInit for retention restore.
 *
 * To override settings in the message block, users can assign values to the
 * fields in the message block data structure directly
 *
 * \ref examples/simple/ddrphy_phyinit_usercustom_custompretrain.c example of this function
 *
 * @return Void
 */
void ddrphy_phyinit_usercustom_custompretrain(void)
{
	VERBOSE("%s Start\n", __func__);

	VERBOSE("%s End\n", __func__);
}
