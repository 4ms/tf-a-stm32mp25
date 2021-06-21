/*
 * Copyright (C) 2021-2022, STMicroelectronics - All Rights Reserved
 *
 * SPDX-License-Identifier: BSD-3-Clause
 */

#ifndef DDRPHY_PHYINIT_H
#define DDRPHY_PHYINIT_H

#include <ddrphy_phyinit_usercustom.h>

#define DDRPHY_PHYINIT_RID 202006

/* Global Structures : instantiated in ddrphy_globals.c */
extern runtime_config_t runtimeconfig;

extern user_input_basic_t userinputbasic;
extern user_input_advanced_t userinputadvanced;

extern pmu_smb_ddr_1d_t mb_ddr_1d[NB_PS];
extern pmu_smb_ddr_1d_t shdw_ddr_1d[NB_PS];

/* Function definitions */
int ddrphy_phyinit_softsetmb(int ps, char *field, int value);
void ddrphy_phyinit_initstruct(void);
void *ddrphy_phyinit_get_user_input_basic_base(void);
void *ddrphy_phyinit_get_user_input_advanced_base(void);

#endif /* DDRPHY_PHYINIT_H */
