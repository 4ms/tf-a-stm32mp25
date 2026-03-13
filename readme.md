This has been forked from ST's TFA and modified to allow compilation on a macOS host.

Requirements: aarch64-none-elf-gcc toolchain is on your path (v12 and up seem to work):
```bash
PATH=$PATH:/path/to/arm-gnu-toolchain-12.3.rel1-darwin-arm64-aarch64-none-elf/bin
```

To build for the stm32mp257f-ev1 board:

```bash
git checkout https://github.com/4ms/tf-a-stm32mp25.git
cd tf-a-stm32mp25

make PLAT=stm32mp2 DTB_FILE_NAME=stm32mp257f-ev1.dtb STM32MP_SDMMC=1 SPD=opteed STM32MP_DDR4_TYPE=1 CROSS_COMPILE=aarch64-none-elf-
```

To build fiptool:

```bash
cd tf-a-stm32mp25
make PLAT=stm32mp2 fiptool
```

On some macOS systems, you may need to do this:

```bash
cd tf-a-stm32mp25
make PLAT=stm32mp2 OPENSSL_DIR=/opt/homebrew/opt/openssl@1.1 HOSTCCFLAGS="-I/opt/homebrew/opt/openssl@1.1/include" fiptool
```

The file `make_helpers/defaults.mk` has been modified to allow overriding the default location
of OPENSSL_DIR.

To install the FSBL onto an SD card:

```bash
sudo dd if=build/stm32mp2/release/tf-a-stm32mp257f-ev1.stm32 of=/dev/xxx1
sudo dd if=build/stm32mp2/release/tf-a-stm32mp257f-ev1.stm32 of=/dev/xxx2
```

Where /dev/xxx1 and /dev/xxx2 are the devices for the 1st and 2nd partitions on the SD card.

----------
The original README is here: [readme-orig.rst](readme-orig.rst)

