This has been forked from ST's TFA and modified to allow compilation on a macOS host.

To build for the stm32mp257f-ev1 board:

```
git checkout https://github.com/4ms/tf-a-stm32mp25.git
cd tf-a-stm32mp25
export CROSS_COMPILE=/path/to/arm-gnu-toolchain-12.3.rel1-darwin-arm64-aarch64-none-elf/bin/aarch64-none-elf-
make PLAT=stm32mp2 DTB_FILE_NAME=stm32mp257f-ev1.dtb STM32MP_SDMMC=1 SPD=opteed STM32MP_DDR4_TYPE=1
```

To build fiptool:

```
cd tf-a-stm32mp25
make fiptool
```

On some macOS systems, you may need to do this:

```
cd tf-a-stm32mp25
OPENSSL_DIR=/opt/homebrew/opt/openssl@1.1 HOSTCCFLAGS="-I/opt/homebrew/opt/openssl@1.1/include" make fiptool
```

The file `make_helpers/defaults.mk` has been modified to allow overriding the default location
of OPENSSL_DIR.

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











