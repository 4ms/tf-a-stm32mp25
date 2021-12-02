/*
 * Copyright (c) 2021, STMicroelectronics - All Rights Reserved
 *
 * SPDX-License-Identifier: BSD-3-Clause
 */

#ifndef PLAT_TBBR_IMG_DEF_H
#define PLAT_TBBR_IMG_DEF_H

#include <export/common/tbbr/tbbr_img_def_exp.h>

#if STM32MP_DDR_FIP_IO_STORAGE
#define DDR_FW_ID		MAX_IMG_IDS_WITH_SPMDS
#define MAX_IMG_WITH_DDR_IDS	MAX_IMG_IDS_WITH_SPMDS + 1
#else
#define MAX_IMG_WITH_DDR_IDS	MAX_IMG_IDS_WITH_SPMDS
#endif

#define MAX_NUMBER_IDS		MAX_IMG_WITH_DDR_IDS

#endif /* PLAT_TBBR_IMG_DEF_H */

