#
# Copyright (c) 2023, STMicroelectronics - All Rights Reserved
#
# SPDX-License-Identifier: BSD-3-Clause
#

include plat/st/common/common.mk

CRASH_REPORTING		:=	1
ENABLE_PIE		:=	1

# Default Device tree
DTB_FILE_NAME		?=	stm32mp257f-ev.dtb

STM32MP25		:=	1

# Will use SRAM2 as mbedtls heap
STM32MP_USE_EXTERNAL_HEAP :=	1

ifeq (${TRUSTED_BOARD_BOOT},1)
# PKA algo to include
PKA_USE_NIST_P256	:=	1
PKA_USE_BRAINPOOL_P256T1:=	1
endif

# STM32 image header version v2.2
STM32_HEADER_VERSION_MAJOR:=	2
STM32_HEADER_VERSION_MINOR:=	2

PKA_USE_NIST_P256	?=	0
PKA_USE_BRAINPOOL_P256T1 ?=	0

# Number of TF-A copies in the device
STM32_TF_A_COPIES		:=	2

# PLAT_PARTITION_MAX_ENTRIES must take care of STM32_TF-A_COPIES and other partitions
# such as metadata (2) and fsbl-m (2) to find all the FIP partitions (default is 2).
PLAT_PARTITION_MAX_ENTRIES	:=	$(shell echo $$(($(STM32_TF_A_COPIES) + 6)))

ifeq (${PSA_FWU_SUPPORT},1)
# Number of banks of updatable firmware
NR_OF_FW_BANKS			:=	2
NR_OF_IMAGES_IN_FW_BANK		:=	1

FWU_MAX_PART = $(shell echo $$(($(STM32_TF_A_COPIES) + 2 + $(NR_OF_FW_BANKS))))
ifeq ($(shell test $(FWU_MAX_PART) -gt $(PLAT_PARTITION_MAX_ENTRIES); echo $$?),0)
$(error "Required partition number is $(FWU_MAX_PART) where PLAT_PARTITION_MAX_ENTRIES is only \
$(PLAT_PARTITION_MAX_ENTRIES)")
endif
endif

STM32_HASH_VER		:=	4
STM32_RNG_VER		:=	4

# Download load address for serial boot devices
DWL_BUFFER_BASE 	?=	0x87000000

# DDR types
STM32MP_DDR3_TYPE	?=	0
STM32MP_DDR4_TYPE	?=	0
STM32MP_LPDDR4_TYPE	?=	0
ifeq (${STM32MP_DDR3_TYPE},1)
DDR_TYPE		:=	ddr3
endif
ifeq (${STM32MP_DDR4_TYPE},1)
DDR_TYPE		:=	ddr4
endif
ifeq (${STM32MP_LPDDR4_TYPE},1)
DDR_TYPE		:=	lpddr4
endif

# DDR features
STM32MP_DDR_DUAL_AXI_PORT	:=	1
STM32MP_DDR_FIP_IO_STORAGE	:=	1

# Device tree
BL2_DTSI		:=	stm32mp25-bl2.dtsi
FDT_SOURCES		:=	$(addprefix ${BUILD_PLAT}/fdts/, $(patsubst %.dtb,%-bl2.dts,$(DTB_FILE_NAME)))
BL31_DTSI		:=	stm32mp25-bl31.dtsi
FDT_SOURCES		+=	$(addprefix ${BUILD_PLAT}/fdts/, $(patsubst %.dtb,%-bl31.dts,$(DTB_FILE_NAME)))

# Macros and rules to build TF binary
STM32_TF_STM32		:=	$(addprefix ${BUILD_PLAT}/tf-a-, $(patsubst %.dtb,%.stm32,$(DTB_FILE_NAME)))
STM32_LD_FILE		:=	plat/st/stm32mp2/${ARCH}/stm32mp2.ld.S
STM32_BINARY_MAPPING	:=	plat/st/stm32mp2/${ARCH}/stm32mp2.S

STM32MP_FW_CONFIG_NAME	:=	$(patsubst %.dtb,%-fw-config.dtb,$(DTB_FILE_NAME))
STM32MP_FW_CONFIG	:=	${BUILD_PLAT}/fdts/$(STM32MP_FW_CONFIG_NAME)
STM32MP_SOC_FW_CONFIG	:=	$(addprefix ${BUILD_PLAT}/fdts/, $(patsubst %.dtb,%-bl31.dtb,$(DTB_FILE_NAME)))
ifeq (${STM32MP_DDR_FIP_IO_STORAGE},1)
STM32MP_DDR_FW_NAME	:=	${DDR_TYPE}_pmu_train.bin
STM32MP_DDR_FW		:=	drivers/st/ddr/phy/firmware/bin/${STM32MP_DDR_FW_NAME}
endif
FDT_SOURCES		+=	$(addprefix fdts/, $(patsubst %.dtb,%.dts,$(STM32MP_FW_CONFIG_NAME)))
# Add the FW_CONFIG to FIP and specify the same to certtool
$(eval $(call TOOL_ADD_PAYLOAD,${STM32MP_FW_CONFIG},--fw-config))
# Add the SOC_FW_CONFIG to FIP and specify the same to certtool
$(eval $(call TOOL_ADD_IMG,STM32MP_SOC_FW_CONFIG,--soc-fw-config,,$(ENCRYPT_BL31)))
ifeq (${STM32MP_DDR_FIP_IO_STORAGE},1)
# Add the FW_DDR to FIP and specify the same to certtool
$(eval $(call TOOL_ADD_IMG,STM32MP_DDR_FW,--ddr-fw))
endif
ifeq ($(GENERATE_COT),1)
STM32MP_CFG_CERT	:=	$(BUILD_PLAT)/stm32mp_cfg_cert.crt
# Add the STM32MP_CFG_CERT to FIP and specify the same to certtool
$(eval $(call TOOL_ADD_PAYLOAD,${STM32MP_CFG_CERT},--stm32mp-cfg-cert))
endif

# Enable flags for C files
$(eval $(call assert_booleans,\
	$(sort \
		PKA_USE_BRAINPOOL_P256T1 \
		PKA_USE_NIST_P256 \
		STM32MP_CRYPTO_ROM_LIB \
		STM32MP_DDR_DUAL_AXI_PORT \
		STM32MP_DDR_FIP_IO_STORAGE \
		STM32MP_DDR3_TYPE \
		STM32MP_DDR4_TYPE \
		STM32MP_LPDDR4_TYPE \
		STM32MP_USE_EXTERNAL_HEAP \
		STM32MP25 \
)))

$(eval $(call assert_numerics,\
	$(sort \
		PLAT_PARTITION_MAX_ENTRIES \
		STM32_HASH_VER \
		STM32_RNG_VER \
		STM32_TF_A_COPIES \
)))

$(eval $(call add_defines,\
	$(sort \
		DWL_BUFFER_BASE \
		PKA_USE_BRAINPOOL_P256T1 \
		PKA_USE_NIST_P256 \
		PLAT_DEF_FIP_UUID \
		PLAT_PARTITION_MAX_ENTRIES \
		PLAT_TBBR_IMG_DEF \
		STM32_HASH_VER \
		STM32_RNG_VER \
		STM32_TF_A_COPIES \
		STM32MP_CRYPTO_ROM_LIB \
		STM32MP_DDR_DUAL_AXI_PORT \
		STM32MP_DDR_FIP_IO_STORAGE \
		STM32MP_DDR3_TYPE \
		STM32MP_DDR4_TYPE \
		STM32MP_LPDDR4_TYPE \
		STM32MP_USE_EXTERNAL_HEAP \
		STM32MP25 \
)))

# Include paths and source files
PLAT_INCLUDES		+=	-Iplat/st/stm32mp2/include/
PLAT_INCLUDES		+=	-Idrivers/st/ddr/phy/phyinit/include/
PLAT_INCLUDES		+=	-Idrivers/st/ddr/phy/firmware/include/

PLAT_BL_COMMON_SOURCES	+=	plat/st/stm32mp2/stm32mp2_private.c

PLAT_BL_COMMON_SOURCES	+=	drivers/st/uart/${ARCH}/stm32_console.S

PLAT_BL_COMMON_SOURCES	+=	lib/cpus/${ARCH}/cortex_a35.S

PLAT_BL_COMMON_SOURCES	+=	drivers/st/bsec/bsec3.c					\
				drivers/st/iwdg/stm32_iwdg.c				\
				drivers/st/reset/stm32mp2_reset.c			\
				plat/st/stm32mp2/${ARCH}/stm32mp2_helper.S		\
				plat/st/stm32mp2/stm32mp2_syscfg.c			\
				plat/st/stm32mp2/stm32mp2_pwr.c

PLAT_BL_COMMON_SOURCES	+=	drivers/st/clk/clk-stm32-core.c				\
				drivers/st/clk/stm32mp2_clk.c				\
				drivers/st/crypto/stm32_rng.c

BL2_SOURCES		+=	plat/st/stm32mp2/plat_bl2_mem_params_desc.c		\
				plat/st/stm32mp2/stm32mp2_fconf_firewall.c

BL2_SOURCES		+=	drivers/st/crypto/stm32_hash.c				\
				plat/st/stm32mp2/bl2_plat_setup.c

BL2_SOURCES		+=	drivers/st/rif/stm32mp2_risaf.c

ifeq (${TRUSTED_BOARD_BOOT},1)
BL2_SOURCES		+=	drivers/st/crypto/stm32_pka.c
BL2_SOURCES		+=	drivers/st/crypto/stm32_saes.c
endif

ifneq ($(filter 1,${STM32MP_EMMC} ${STM32MP_SDMMC}),)
BL2_SOURCES		+=	drivers/st/mmc/stm32_sdmmc2.c
endif

ifeq (${STM32MP_RAW_NAND},1)
BL2_SOURCES		+=	drivers/st/fmc/stm32_fmc2_nand.c
endif

ifneq ($(filter 1,${STM32MP_SPI_NAND} ${STM32MP_SPI_NOR}),)
BL2_SOURCES		+=	drivers/st/spi/stm32_ospi.c
endif

ifneq ($(filter 1,${STM32MP_RAW_NAND} ${STM32MP_SPI_NAND} ${STM32MP_SPI_NOR}),)
BL2_SOURCES		+=	plat/st/stm32mp2/stm32mp2_boot_device.c
endif

ifeq (${STM32MP_UART_PROGRAMMER},1)
BL2_SOURCES		+=	drivers/st/uart/stm32_uart.c
endif

ifeq (${STM32MP_USB_PROGRAMMER},1)
#The DFU stack uses only one end point, reduce the USB stack footprint
$(eval $(call add_define_val,CONFIG_USBD_EP_NB,1U))
$(eval $(call add_define,USB_CORE_AVOID_PACKET_SPLIT_MPS))
BL2_SOURCES		+=	drivers/st/usb_dwc3/usb_dwc3.c				\
				plat/st/stm32mp2/stm32mp2_usb_dfu.c
endif

BL2_SOURCES		+=	drivers/st/ddr/stm32mp2_ddr.c				\
				drivers/st/ddr/stm32mp2_ddr_helpers.c			\
				drivers/st/ddr/stm32mp2_ram.c

BL2_SOURCES		+=	drivers/st/ddr/phy/phyinit/src/ddrphy_phyinit_c_initphyconfig.c				\
				drivers/st/ddr/phy/phyinit/src/ddrphy_phyinit_calcmb.c					\
				drivers/st/ddr/phy/phyinit/src/ddrphy_phyinit_globals.c					\
				drivers/st/ddr/phy/phyinit/src/ddrphy_phyinit_i_loadpieimage.c				\
				drivers/st/ddr/phy/phyinit/src/ddrphy_phyinit_initstruct.c				\
				drivers/st/ddr/phy/phyinit/src/ddrphy_phyinit_isdbytedisabled.c				\
				drivers/st/ddr/phy/phyinit/src/ddrphy_phyinit_loadpieprodcode.c				\
				drivers/st/ddr/phy/phyinit/src/ddrphy_phyinit_mapdrvstren.c				\
				drivers/st/ddr/phy/phyinit/src/ddrphy_phyinit_progcsrskiptrain.c			\
				drivers/st/ddr/phy/phyinit/src/ddrphy_phyinit_reginterface.c				\
				drivers/st/ddr/phy/phyinit/src/ddrphy_phyinit_restore_sequence.c			\
				drivers/st/ddr/phy/phyinit/src/ddrphy_phyinit_sequence.c				\
				drivers/st/ddr/phy/phyinit/src/ddrphy_phyinit_softsetmb.c				\
				drivers/st/ddr/phy/phyinit/usercustom/ddrphy_phyinit_usercustom_customposttrain.c	\
				drivers/st/ddr/phy/phyinit/usercustom/ddrphy_phyinit_usercustom_custompretrain.c	\
				drivers/st/ddr/phy/phyinit/usercustom/ddrphy_phyinit_usercustom_saveretregs.c

BL2_SOURCES		+=	drivers/st/ddr/phy/phyinit/src/ddrphy_phyinit_d_loadimem.c				\
				drivers/st/ddr/phy/phyinit/src/ddrphy_phyinit_f_loaddmem.c				\
				drivers/st/ddr/phy/phyinit/src/ddrphy_phyinit_g_execfw.c				\
				drivers/st/ddr/phy/phyinit/src/ddrphy_phyinit_h_readmsgblock.c				\
				drivers/st/ddr/phy/phyinit/src/ddrphy_phyinit_storemsgblk.c				\
				drivers/st/ddr/phy/phyinit/src/ddrphy_phyinit_writeoutmem.c				\
				drivers/st/ddr/phy/phyinit/usercustom/ddrphy_phyinit_usercustom_h_readmsgblock.c 	\
				drivers/st/ddr/phy/phyinit/usercustom/ddrphy_phyinit_usercustom_g_waitfwdone.c

BL2_SOURCES		+=	plat/st/stm32mp2/plat_image_load.c

# BL31 sources
BL31_SOURCES		+=	${FDT_WRAPPERS_SOURCES}

BL31_SOURCES		+=	plat/st/stm32mp2/bl31_plat_setup.c			\
				plat/st/stm32mp2/stm32mp2_pm.c				\
				plat/st/stm32mp2/stm32mp2_topology.c
# Generic GIC v2
include drivers/arm/gic/v2/gicv2.mk

BL31_SOURCES		+=	${GICV2_SOURCES}					\
				plat/common/plat_gicv2.c				\
				plat/st/common/stm32mp_gic.c

# Generic PSCI
BL31_SOURCES		+=	plat/common/plat_psci_common.c

BL31_SOURCES		+=	plat/st/common/stm32mp_svc_setup.c			\
				plat/st/stm32mp2/services/stgen_svc.c			\
				plat/st/stm32mp2/services/stm32mp2_svc_setup.c

# Arm Archtecture services
BL31_SOURCES		+=	services/arm_arch_svc/arm_arch_svc_setup.c

# Compilation rules
.PHONY: check_ddr_type
.SUFFIXES:

bl2: check_ddr_type

check_ddr_type:
	$(eval DDR_TYPE = $(shell echo $$(($(STM32MP_DDR3_TYPE) + \
					   $(STM32MP_DDR4_TYPE) + \
					   $(STM32MP_LPDDR4_TYPE)))))
	@if [ ${DDR_TYPE} != 1 ]; then \
		echo "One and only one DDR type must be defined"; \
		false; \
	fi

# generate separate DDR FIP image
ifeq (${STM32MP_DDR_FIP_IO_STORAGE},1)
ifneq ($(filter 1,${STM32MP_UART_PROGRAMMER} ${STM32MP_USB_PROGRAMMER}),)

DDR_FIP_NAME		?=	fip-ddr.bin
DDR_FIP_ARGS		+=	--ddr-fw ${STM32MP_DDR_FW}
DDR_FIP_DEPS		+=	${STM32MP_DDR_FW}

${BUILD_PLAT}/${DDR_FIP_NAME}: ${DDR_FIP_DEPS} fiptool
	${Q}${FIPTOOL} create ${DDR_FIP_ARGS} $@
	${Q}${FIPTOOL} info $@
	@${ECHO_BLANK_LINE}
	@echo "Built $@ successfully"
	@${ECHO_BLANK_LINE}

fip-ddr: ${BUILD_PLAT}/${DDR_FIP_NAME}

fip: fip-ddr

endif
endif

# Create DTB file for BL31
${BUILD_PLAT}/fdts/%-bl31.dts: fdts/%.dts fdts/${BL31_DTSI} | ${BUILD_PLAT} fdt_dirs
	@echo '#include "$(patsubst fdts/%,%,$<)"' > $@
	@echo '#include "${BL31_DTSI}"' >> $@

${BUILD_PLAT}/fdts/%-bl31.dtb: ${BUILD_PLAT}/fdts/%-bl31.dts

include plat/st/common/common_rules.mk
