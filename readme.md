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
make PLAT=stm32mp2 fiptool
```

On some macOS systems, you may need to do this:

```
cd tf-a-stm32mp25
make PLAT=stm32mp2 OPENSSL_DIR=/opt/homebrew/opt/openssl@1.1 HOSTCCFLAGS="-I/opt/homebrew/opt/openssl@1.1/include" fiptool
```

The file `make_helpers/defaults.mk` has been modified to allow overriding the default location
of OPENSSL_DIR.

----------
The original README is here: [readme-orig.rst](readme-orig.rst)

