tools/fiptool/fiptool --verbose create \
	--ddr-fw drivers/st/ddr/phy/firmware/bin/stm32mp2/ddr4_pmu_train.bin \
	--soc-fw-config build/stm32mp2/release/fdts/stm32mp257f-ev1-bl31.dtb \
	--soc-fw build/stm32mp2/release/bl31.bin \
	--fw-config build/stm32mp2/release/fdts/stm32mp257f-ev1-fw-config.dtb \
	--tos-fw ../optee-stm32mp25/out/arm-plat-stm32mp2/core/tee-header_v2.bin \
	--tos-fw-extra1 ../optee-stm32mp25/out/arm-plat-stm32mp2/core/tee-pager_v2.bin \
	--nt-fw ../build-baremetal-uboot/u-boot-nodtb.bin \
	--hw-config ../build-baremetal-uboot/u-boot.dtb \
	build/stm32mp2/release/fip.bin
