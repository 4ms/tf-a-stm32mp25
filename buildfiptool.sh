#!/usr/bin/env bash
# Build the fiptool host tool.
# On macOS, point the build at Homebrew's OpenSSL; on Linux the system
# OpenSSL headers (package libssl-dev) are found automatically.

if [ "$(uname -s)" = "Darwin" ]; then
	make fiptool PLAT=stm32mp2 BAREMETAL_IMAGE_LOADER=1 OPENSSL_DIR=/opt/homebrew/opt/openssl@1.1 HOSTCCFLAGS="-I/opt/homebrew/opt/openssl@1.1/include"
else
	make fiptool PLAT=stm32mp2 BAREMETAL_IMAGE_LOADER=1
fi
