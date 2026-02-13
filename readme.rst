This has been forked from ST's TFA.

Modifications:
-------------
- allow compilation on a macOS host
- add BAREMETAL_IMAGE_LOADER build flag:
  - Builds BL2 and its DTB requirements
  - BL2 performs some hardware init (PMIC, DDRRAM, ...) and then
    loads a baremetal binary app from the FIP file, and excecutes it
    in EL3 Secure mode.
  - BL31 (TF-A Secure Monitor), BL32 (OP-TEE), BL33 (U-Boot) are not built or used


Building:
--------

To build for the stm32mp257f-ev1 board, first make sure the aarch64-none-elf-gcc toolchain
is on your path. Version 14.3 and 15.2 are known to work, and it's likely earlier versions will
as well.

```
export PATH=$PATH:/path/to/arm-gnu-toolchain-15.2.rel1-darwin-arm64-aarch64-none-elf/bin
```

Checkout the repo:
```
git checkout -b v2.10-stm32mp2-baremetal https://github.com/4ms/tf-a-stm32mp25.git
cd tf-a-stm32mp25
```
Note that the default branch is v2.10-stm32mp2-baremetal, make sure you use that.


Build:

```
make PLAT=stm32mp2 \
    CROSS_COMPILE=aarch64-none-elf- \
    DTB_FILE_NAME=stm32mp257f-ev1.dtb \
    STM32MP_SDMMC=1 \
    STM32MP_DDR4_TYPE=1 \
    BAREMETAL_IMAGE_LOADER=1 \
    LOG_LEVEL=40 \
    dtbs fsbl
```

This command can also be run with:

```
./build.sh
```

This will build `tf-a-stm32mp257f-ev1.stm32`, which is the FSBL (first stage bootloader). This file
gets copied to partitions 1 and 2 on the SD card with dd (replace /dev/diskX1 with the first
partition of your SD card, and likewise for /dev/diskX2)

```
sudo dd if=build/stm32mp2/release/tf-a-stm32mp257f-ev1.stm32 of=/dev/diskX1
sudo dd if=build/stm32mp2/release/tf-a-stm32mp257f-ev1.stm32 of=/dev/diskX2
```

The FSBL will initialize DDR RAM and launch your app. It reads the DDR training firmware and your
app from a FIP file contained on partition 5 of the SD card.

The DDR initialization firmware comes pre-built with TF-A. So all you need to do is make a FIP file
with that binary and your baremetal app binary. TF-A provides a FIP tool to do that.

To build fiptool:

```
cd tf-a-stm32mp25
make PLAT=stm32mp2 fiptool
```

On some macOS systems, you will need to do this:

```
cd tf-a-stm32mp25
make PLAT=stm32mp2 OPENSSL_DIR=/opt/homebrew/opt/openssl@1.1 HOSTCCFLAGS="-I/opt/homebrew/opt/openssl@1.1/include" fiptool
```

To support the above command, the file `make_helpers/defaults.mk` has been modified to allow
overriding the default location of OPENSSL_DIR.


Once the fiptool is built, run it to create your FIP file (change the path to your baremetal
project):

```
tools/fiptool/fiptool --verbose create \
	--ddr-fw drivers/st/ddr/phy/firmware/bin/stm32mp2/ddr4_pmu_train.bin \
	--bm-fw ../stm32mp2-baremetal/minimal_boot/build/main.bin \
	build/stm32mp2/release/fip.bin
```


Now flash the fip file to partition 5:

```
sudo dd if=build/stm32mp2/release/fip.bin of=/dev/diskX5
```


Shortcuts:
----------
- You can build the fip tool on macOS with `./buildfiptool.sh`
- You can make the fip file with `./makefip.sh`. Be sure to change the path to your baremetal app
- You can dd both the FSBL and the FIP with `./flashsd.sh`

A complete, fresh build and flashing looks like this:
```
make clean && ./build.sh && ./buildfiptool.sh && ./makefip.sh && ./flashsd.sh
```

Followed by an unmount for the disk


Trusted Firmware-A
==================

Trusted Firmware-A (TF-A) is a reference implementation of secure world software
for `Arm A-Profile architectures`_ (Armv8-A and Armv7-A), including an Exception
Level 3 (EL3) `Secure Monitor`_. It provides a suitable starting point for
productization of secure world boot and runtime firmware, in either the AArch32
or AArch64 execution states.

TF-A implements Arm interface standards, including:

-  `Power State Coordination Interface (PSCI)`_
-  `Trusted Board Boot Requirements CLIENT (TBBR-CLIENT)`_
-  `SMC Calling Convention`_
-  `System Control and Management Interface (SCMI)`_
-  `Software Delegated Exception Interface (SDEI)`_

The code is designed to be portable and reusable across hardware platforms and
software models that are based on the Armv8-A and Armv7-A architectures.

In collaboration with interested parties, we will continue to enhance TF-A
with reference implementations of Arm standards to benefit developers working
with Armv7-A and Armv8-A TrustZone technology.

Users are encouraged to do their own security validation, including penetration
testing, on any secure world code derived from TF-A.

More Info and Documentation
---------------------------

To find out more about Trusted Firmware-A, please `view the full documentation`_
that is available through `trustedfirmware.org`_.

--------------

*Copyright (c) 2013-2019, Arm Limited and Contributors. All rights reserved.*

.. _Armv7-A and Armv8-A: https://developer.arm.com/products/architecture/a-profile
.. _Secure Monitor: http://www.arm.com/products/processors/technologies/trustzone/tee-smc.php
.. _Power State Coordination Interface (PSCI): PSCI_
.. _PSCI: http://infocenter.arm.com/help/topic/com.arm.doc.den0022d/Power_State_Coordination_Interface_PDD_v1_1_DEN0022D.pdf
.. _Trusted Board Boot Requirements CLIENT (TBBR-CLIENT): https://developer.arm.com/docs/den0006/latest/trusted-board-boot-requirements-client-tbbr-client-armv8-a
.. _SMC Calling Convention: http://infocenter.arm.com/help/topic/com.arm.doc.den0028b/ARM_DEN0028B_SMC_Calling_Convention.pdf
.. _System Control and Management Interface (SCMI): SCMI_
.. _SCMI: http://infocenter.arm.com/help/topic/com.arm.doc.den0056a/DEN0056A_System_Control_and_Management_Interface.pdf
.. _Software Delegated Exception Interface (SDEI): SDEI_
.. _SDEI: http://infocenter.arm.com/help/topic/com.arm.doc.den0054a/ARM_DEN0054A_Software_Delegated_Exception_Interface.pdf
.. _Arm A-Profile architectures: https://developer.arm.com/architectures/cpu-architecture/a-profile
.. _view the full documentation: https://www.trustedfirmware.org/docs/tf-a
.. _trustedfirmware.org: http://www.trustedfirmware.org











