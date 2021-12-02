#
# Copyright (c) 2021, STMicroelectronics - All Rights Reserved
#
# SPDX-License-Identifier: BSD-3-Clause
#

# Name of the platform defined source file name,
# which contains platform defined UUID entries populated
# in the plat_def_toc_entries[].
PLAT_DEF_UUID_CONFIG_FILE_NAME	:= plat_def_uuid_config

PLAT_DEF_UUID_CONFIG_FILE_PATH := ../st/plat_fiptool

PLAT_DEF_UUID := yes

INCLUDE_PATHS +=  -I../../plat/st/stm32mp2/include/ \
		  -I./

ifeq (${PLAT_DEF_UUID},yes)
HOSTCCFLAGS += -DPLAT_DEF_FIP_UUID
PLAT_OBJECTS += ${PLAT_DEF_UUID_CONFIG_FILE_PATH}/${PLAT_DEF_UUID_CONFIG_FILE_NAME}.o
endif

OBJECTS += ${PLAT_OBJECTS}

