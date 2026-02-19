This has been forked from ST's TFA.
The original readme is [here](readme-tfa.rst)

Modifications:
-------------
- Allow compilation on a macOS host
- Add BAREMETAL_IMAGE_LOADER build flag:
  - Builds BL2 with an embedded device tree DTB
  - BL2 performs some hardware init (PMIC, DDRRAM, ...) and then
    loads a baremetal binary app from the FIP file, which it excecutes
    in EL3 Secure mode.
  - Does not build or run BL31 (TF-A Secure Monitor), BL32 (OP-TEE), or BL33 (U-Boot).


Building:
--------

To build for the stm32mp257f-ev1 board, first make sure the aarch64-none-elf-gcc toolchain
is on your path. Version 14.3 and 15.2 are known to work, and it's likely earlier versions will
as well. Make sure it is on your path:

```bash
export PATH=$PATH:/path/to/arm-gnu-toolchain-15.2.rel1-darwin-arm64-aarch64-none-elf/bin
```

Checkout the repo:

```bash
git checkout -b v2.10-stm32mp2-baremetal https://github.com/4ms/tf-a-stm32mp25.git
cd tf-a-stm32mp25
```

Note that the default branch is v2.10-stm32mp2-baremetal, make sure you use that.

### Build the FSBL

Build:

```bash
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

```bash
./build.sh
```

This will build `tf-a-stm32mp257f-ev1.stm32`, which is the FSBL (first stage bootloader).

### Installing the FSBL on the SD card

The FSBL file needs to be copied to partitions 1 and 2 on the SD card using dd.
In the following commands, replace /dev/diskX1 and /dev/diskX2
with the device path of your SD card's first and second partitions.
On macOS, this should be like "/dev/disk4s1" and "/dev/disk4s2".
On Linux it's typically something like "/dev/sda1" and "/dev/sda2".

```bash
sudo dd if=build/stm32mp2/release/tf-a-stm32mp257f-ev1.stm32 of=/dev/diskX1
sudo dd if=build/stm32mp2/release/tf-a-stm32mp257f-ev1.stm32 of=/dev/diskX2
```

### Build the FIP file

The FSBL requires external an external binary file in order to initialize the DDR RAM.
It also needs your application binary so it can load it.

These binary files are joined together into a FIP file.
The FIP file lives on partition 5 of the SD card.

TF-A includes a tool called fiptool which can be used to create a FIP file.

To build fiptool:

```bash
cd tf-a-stm32mp25
make PLAT=stm32mp2 BAREMETAL_IMAGE_LOADER=1 fiptool
```

On some macOS systems, you will need to do this:

```bash
cd tf-a-stm32mp25
make PLAT=stm32mp2 BAREMETAL_IMAGE_LOADER=1 \
	OPENSSL_DIR=/opt/homebrew/opt/openssl@1.1 \
	HOSTCCFLAGS="-I/opt/homebrew/opt/openssl@1.1/include" \
	fiptool
```

To support the above command, the file `make_helpers/defaults.mk` has been modified to allow
overriding the default location of OPENSSL_DIR.

Once the fiptool is built, run it to create your FIP file (change the path to your baremetal
project):

```bash
tools/fiptool/fiptool --verbose create \
	--ddr-fw drivers/st/ddr/phy/firmware/bin/stm32mp2/ddr4_pmu_train.bin \
	--bm-fw ../stm32mp2-baremetal/minimal_boot/build/main.bin \
	build/stm32mp2/release/fip.bin
```

Note that the ddr-fw binary comes pre-built in this repo, so you don't need to build it.

## Installing on the SD card

Now flash the fip file to partition 5 (adjust the device path):

```bash
sudo dd if=build/stm32mp2/release/fip.bin of=/dev/diskX5
```


## Modifying the console output

By default, the console output is on USART2, which is connected to the ST-LINK via the USB jack on
the EV1 board. I found it more convenient to use USART6 via two pins on the 40-pin expansion header.
(pin 6 = GND, pin 8 = USART6.TX, pin 10 = USART6.RX). This allowed me to use the SWD header rather than the
ST-LINK interface (see below).

## Using SWD/JTAG instead of ST-LINK

The USB jack on the EV1 attaches to an STM32F7 chip which controls the STM32MP257 via SWD. It also
forwards USART2 between the STM32MP257 and USB. The easiest way to get started is to connect this
USB jack to your computer and then use openocd and gdb, which should detect it immediately. You
also can open a console to the USB serial device that appears when connected.

However, this approach is limited in that you cannot use your own debugger (JLINK or TRACE32, for
example). There is a MIPI10 header that has a standard SWD pinout, but you have to change some
jumpers in order to use it.

In order to attach your own debugger, you have to disable the F7 by installing a jumper on JP3
(which is right next to the  F7 chip).

Then, move the jumper on JP4 to the top position and plug a 5V barrel power supply into the jack
(CN20). Check the EV1 user manual for specifics, but the schematic mentions 5V 3A, and I was able to
power the board with a 12V 1.5A supply.

You cannot use the USB jack CN21 (marked "USB_PWR ST_LINK") for anything when the jumpers
are set in these positions.


Shortcuts:
----------
- You can build the fip tool on macOS with `./buildfiptool.sh`
- You can make the fip file with `./makefip.sh`. Be sure to change the path to your baremetal app
- You can dd both the FSBL and the FIP with `./flashsd.sh`

A complete, fresh build and flashing looks like this:
```
export SDCARDDEV=/dev/XXXXX
make clean && ./build.sh && ./buildfiptool.sh && ./makefip.sh && ./flashsd.sh
```

...where SDCARDDEV is set to the device of the SD card.


Notes on DTS
------------

The device tree binary (DTB) is embedded into the BL2 FSBL binary at a fixed location. This
is done in the link script:
    - stm32mp2.S tells the linker to put the contents of DTB_BIN_PATH (build/../stm32mp257f-ev1-bl2.dtb)
      into the section named .dtb_image
    - stm32mp2.ld.S puts .dtb_image in the .data section, at STM32MP_BL2_DTB_BASE (0x0E011000).
      The max size is 0x6000, so it can't exceed 0x0E017000 (which is where the code segment starts)

The DTB that's compiled is a the DTS given in the command-line build option DTB_FILE_NAME
(stm32mp257f-ev1.dts in our case), with the contents of stm32mp25-bl2.dtsi appended automatically.
Both these files are in fdts/.

