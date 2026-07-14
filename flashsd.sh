#!/usr/bin/env bash
# This script is just a fancy wrapper for:
# sudo dd if=build/stm32mp2/release/tf-a-stm32mp257f-ev1.stm32 of=${SDCARDDEV}1
# sudo dd if=build/stm32mp2/release/tf-a-stm32mp257f-ev1.stm32 of=${SDCARDDEV}2
# sudo dd if=build/stm32mp2/release/fip.bin of=${SDCARDDEV}5

set -u

BUILD_DIR="build/stm32mp2/release"
STM32_IMAGE="$BUILD_DIR/tf-a-stm32mp257f-ev1.stm32"
FIP_IMAGE="$BUILD_DIR/fip.bin"

usage() {
	cat <<EOF
Usage:
  $0 <sd-device>              e.g. $0 /dev/disk4s
  SDCARDDEV=<sd-device> $0    e.g. SDCARDDEV=/dev/disk4s $0

Flashes:
  $STM32_IMAGE -> <sd-device>1 and <sd-device>2
  $FIP_IMAGE   -> <sd-device>5
EOF
}

if [[ "${1:-}" == "-h" || "${1:-}" == "--help" ]]; then
	usage
	exit 0
fi

SDCARDDEV="${1:-${SDCARDDEV:-}}"

if [[ -z "$SDCARDDEV" ]]; then
	echo "Error: no SD card device given." >&2
	usage
	exit 1
fi

# Partition naming varies by device: if it ends in a digit, figure out the
# connector between the device and its partition numbers (macOS "disk4" -> "disk4s1",
# Linux "mmcblk0" -> "mmcblk0p1"). If it already ends in a non-digit (e.g. /dev/disk4s,
# /dev/sda), it's already a valid partition prefix -- use it as-is.
if [[ "$SDCARDDEV" =~ ^(.*[^0-9])[0-9]+$ ]]; then
	prefix="${BASH_REMATCH[1]}"
	case "$prefix" in
		/dev/disk)   SDCARDDEV="${SDCARDDEV}s" ;;
		/dev/mmcblk) SDCARDDEV="${SDCARDDEV}p" ;;
		*)
			echo "Error: don't know how '$SDCARDDEV' names its partitions." >&2
			echo "Pass the device with its partition-table suffix included, e.g. /dev/disk4s or /dev/mmcblk0p." >&2
			exit 1
			;;
	esac
fi

if [[ ! -e "${SDCARDDEV}1" ]]; then
	echo "Error: ${SDCARDDEV}1 not found. Is the SD card inserted and is '$SDCARDDEV' the right device?" >&2
	exit 1
fi

# Whole-disk device (no partition suffix), used for unmounting/ejecting.
case "$SDCARDDEV" in
	/dev/disk[0-9]*s)   WHOLE_DISK="${SDCARDDEV%s}" ;;
	/dev/mmcblk[0-9]*p) WHOLE_DISK="${SDCARDDEV%p}" ;;
	*)                  WHOLE_DISK="$SDCARDDEV" ;;
esac

case "$(uname -s)" in
	Darwin)
		command -v diskutil >/dev/null 2>&1 && diskutil unmountDisk "$WHOLE_DISK" >/dev/null 2>&1
		;;
	Linux)
		for part in "${SDCARDDEV}"[0-9]*; do
			[[ -e "$part" ]] && sudo umount "$part" >/dev/null 2>&1
		done
		;;
esac

status=0

if [[ -f "$STM32_IMAGE" ]]; then
	sudo dd if="$STM32_IMAGE" of="${SDCARDDEV}1" && \
	sudo dd if="$STM32_IMAGE" of="${SDCARDDEV}2"
	(( $? != 0 )) && status=1
else
	echo "Warning: $STM32_IMAGE not found, skipping." >&2
fi

if [[ -f "$FIP_IMAGE" ]]; then
	sudo dd if="$FIP_IMAGE" of="${SDCARDDEV}5"
	(( $? != 0 )) && status=1
else
	echo "Warning: $FIP_IMAGE not found, skipping." >&2
fi

if [[ ! -f "$STM32_IMAGE" && ! -f "$FIP_IMAGE" ]]; then
	echo "Error: nothing to flash, neither image was found. Did you run build.sh?" >&2
	exit 1
fi

case "$(uname -s)" in
	Darwin)
		command -v diskutil >/dev/null 2>&1 && diskutil eject "$WHOLE_DISK" >/dev/null 2>&1
		;;
	Linux)
		sync
		command -v eject >/dev/null 2>&1 && eject "$WHOLE_DISK" >/dev/null 2>&1
		;;
	*)
		sync
		;;
esac

exit $status
