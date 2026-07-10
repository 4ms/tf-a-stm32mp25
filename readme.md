This has been forked from ST's TFA.
The original readme is [here](readme-tfa.rst)


Modifications:
-------------
- Allow compilation on a macOS host
- Add BAREMETAL_IMAGE_LOADER build flag:
  - Builds BL2 with an embedded device tree DTB
  - BL2 performs some hardware init (PMIC, DDRRAM, ...) and then
    loads a baremetal app from the SD card, which it executes
    in EL3 Secure mode.
  - The app is a .uimg file (U-Boot legacy image: a 64-byte header + raw
    binary) on the GPT partition named "app". BL2 validates the header's
    magic and CRC32s, loads the payload to the header's load address, and
    jumps to the header's entry point.
  - Does not build or run BL31 (TF-A Secure Monitor), BL32 (OP-TEE), or BL33 (U-Boot).

The BAREMETAL_IMAGE_LOADER build flag is optional -- you can compile without it and in theory still load
your application the normal way with U-Boot (in EL1 non-secure mode).

When the BAREMETAL_IMAGE_LOADER flag is set, the following things are setup:
- USART is configured (and used for console output)
- PMIC via I2C7 is setup to power the core and DDR RAM
- SDMMC1 and SDMMC2 (eMMC) are configured
- DDR RAM is initialized
- RISAF4 is setup to allow CA35 and the debugger to access the first 4GB
- BSEC is told to allow full debugger access to all cores

There are more things initialized (BSEC, TAMP), which I haven't fully investigated yet.


# Quick Start (Using the release files)

1. Partition an SD card: (Change `diskX` to your device name. Warning! this will erase all data on the device!)

```bash
./partition_sdcard.sh /dev/diskX
```

2. Download the latest releases of the `tf-a-stm32mp257f-ev1.stm32` and `fip.bin` files from the
[Releases](https://github.com/4ms/tf-a-stm32mp25/releases)

3. Write the two files to the SD card:

```bash
sudo dd if=build/stm32mp2/release/tf-a-stm32mp257f-ev1.stm32 of=/dev/diskX1
sudo dd if=build/stm32mp2/release/tf-a-stm32mp257f-ev1.stm32 of=/dev/diskX2
sudo dd if=build/stm32mp2/release/fip.bin of=/dev/diskX5
```

4. Insert the card into the EV1's SD card slot, set the BOOT switches to SD boot (down down down up -- see the EV1 docs),
and power it up. Attach a USB-C cable to your computer and open a console to the tty device that
shows up when you attach the cable. Press the reset button and you should see something like this:

```
NOTICE:  CPU: STM32MP257FAI Rev.?
NOTICE:  Model: STMicroelectronics STM32MP257F-EV1 Evaluation Board
NOTICE:  Board: MB1936 Var1.0 Rev.C-01
INFO:    Reset reason: Power-on reset (por_rstn) (0x2035)
INFO:    PMIC2 version = 0x11
INFO:    PMIC2 product ID = 0x20
INFO:    FCONF: Reading TB_FW firmware configuration file from: 0xe011000
INFO:    FCONF: Reading firmware configuration information for: stm32mp_fuse
INFO:    FCONF: Reading firmware configuration information for: stm32mp_io
INFO:    Using SDMMC
INFO:      Instance 1
INFO:    Boot used partition fsbl1
NOTICE:  BL2: v2.10-stm32mp2-r2.0(release):baremetal-v0.1(ba0a1568)
NOTICE:  BL2: Built : 23:44:40, Jul  9 2026
INFO:    BL2: Loading image id 26
INFO:    Loading image id=26 at address 0xe041000
INFO:    Image id=26 loaded: 0xe041000 - 0xe048524
INFO:    BL2: Doing platform setup
INFO:    RAM: DDR4 2x16Gbits 2x16bits 1200MHz
INFO:    Memory size = 0x100000000 (4096 MB)
INFO:    BL2: Skip loading image id 27
ERROR:   Could NOT find the 'app' partition!
ERROR:   Failed to load the baremetal app (-2)
```

Next step is to load an app! Go the [stm32mp2-baremetal](https://github.com/4ms/stm32mp2-baremetal)
repo to build and load an example project.


# Building from source


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
./build.sh
```

This script is just a shortcut for this command:

```bash
make PLAT=stm32mp2 CROSS_COMPILE=aarch64-none-elf- DTB_FILE_NAME=stm32mp257f-ev1.dtb STM32MP_SDMMC=1 STM32MP_DDR4_TYPE=1 \
    BAREMETAL_IMAGE_LOADER=1 LOG_LEVEL=40 BOARD=EV1 USE_UART=2 dtbs fsbl
```

The BOARD=... and USE_UART=... options can be changed to select a different hardware target and
console UART (see Configuring below).

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

The FSBL requires an external binary file in order to initialize the DDR RAM.

This binary goes into a FIP file, which lives on partition 5 of the SD card.
The FIP only needs to be written once: your application is NOT in the FIP --
it lives on its own "app" partition (see the stm32mp2-baremetal README).

TF-A includes a tool called fiptool which can be used to create a FIP file.

To build fiptool:

```bash
cd tf-a-stm32mp25

./buildfiptool.sh

# This is a shortcut for this on Linux:
make PLAT=stm32mp2 BAREMETAL_IMAGE_LOADER=1 fiptool

# And is a short cut for this on macOS:
make PLAT=stm32mp2 BAREMETAL_IMAGE_LOADER=1 \
	OPENSSL_DIR=/opt/homebrew/opt/openssl@1.1 \
	HOSTCCFLAGS="-I/opt/homebrew/opt/openssl@1.1/include" \
	fiptool
```

To support the above command, the file `make_helpers/defaults.mk` has been modified to allow
overriding the default location of OPENSSL_DIR.

Once the fiptool is built, run it to create the FIP file:

```bash
./makefip.sh

# This is a shortcut for this:
tools/fiptool/fiptool --verbose create \
	--ddr-fw drivers/st/ddr/phy/firmware/bin/stm32mp2/ddr4_pmu_train.bin \
	build/stm32mp2/release/fip.bin
```

Note that the ddr-fw binary comes pre-built in this repo, so you don't need to build it.

## Installing on the SD card

Now flash the fip file to partition 5 (adjust the device path):

```bash
sudo dd if=build/stm32mp2/release/fip.bin of=/dev/diskX5
```


## Configuring the build (UART and BOARD)

By default, the console output is on USART2, which is connected to the ST-LINK via the USB jack on
the EV1 board. I found it more convenient to use USART6 via two pins on the 40-pin expansion header.
(pin 6 = GND, pin 8 = USART6.TX, pin 10 = USART6.RX). This allowed me to use the SWD header rather than the
ST-LINK interface (see below). On a custom board I made, I wanted to use USART1 (PB8 and PB10).

Any of these options can be chosen by specifying `USE_UART=n` in the `make` command, where n is 1,
2, or 6.


```bash
# Example: Use UART6 on the GPIO expander header of the EV1:
make PLAT=stm32mp2 ... USE_UART=6 dtbs fsbl
```

If you look at the ./build.sh script, you'll see there's also an argument `BOARD=EV1`.
The `BOARD=...` option was something I set up to support a custom PCB with a different DDR4 layout
(which is selected by `BOARD=devboard`). I don't expect anyone else to need to use this, but if you
make your own custom board, you can copy how I did the `devboard` to install your own DDR4
configuration.


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

To get console output via the USART, you need to tell TF-A to use a different USART. I found
USART6 to be convenient -- see the previous section for instructions.


Shortcuts:
----------
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

