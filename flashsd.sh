[[ -n "$SDCARDDEV" ]] && sudo dd if=build/stm32mp2/release/tf-a-stm32mp257f-ev1.stm32 of=${SDCARDDEV}1 && sudo dd if=build/stm32mp2/release/fip.bin of=${SDCARDDEV}5 || echo "SDCARDDEV is not set"
