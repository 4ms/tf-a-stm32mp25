/*
 * Copyright (c) 2021-2023, STMicroelectronics - All Rights Reserved
 *
 * SPDX-License-Identifier: BSD-3-Clause
 */

#ifndef PLAT_TBBR_IMG_DEF_H
#define PLAT_TBBR_IMG_DEF_H

#include <export/common/tbbr/tbbr_img_def_exp.h>

/* Undef the existing values */
#undef BKUP_FWU_METADATA_IMAGE_ID
#undef FWU_METADATA_IMAGE_ID
#undef FW_CONFIG_ID
#undef ENC_IMAGE_ID
#undef GPT_IMAGE_ID
#undef NT_FW_CONFIG_ID
#undef SOC_FW_CONFIG_ID
#undef TB_FW_CONFIG_ID

/* Define the STM32MP1 used ID */
#define BKUP_FWU_METADATA_IMAGE_ID	U(1)
#define FWU_METADATA_IMAGE_ID		U(2)
#define FW_CONFIG_ID			U(8)
#define ENC_IMAGE_ID			U(12)
#define GPT_IMAGE_ID			U(16)
#define NT_FW_CONFIG_ID			U(18)
#define SOC_FW_CONFIG_ID		U(19)
#define TB_FW_CONFIG_ID			U(20)
#define STM32MP_CONFIG_CERT_ID		U(24)

#if STM32MP_DDR_FIP_IO_STORAGE
#define DDR_FW_ID			U(27)
/* Increase the MAX_NUMBER_IDS to match the authentication pool required */
#define MAX_NUMBER_IDS			U(28)

#else
/* Increase the MAX_NUMBER_IDS to match the authentication pool required */
#define MAX_NUMBER_IDS			U(27)

#endif

#endif /* PLAT_TBBR_IMG_DEF_H */

