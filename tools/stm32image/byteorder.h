#pragma once

#if defined(__BEOS__) || defined(__NetBSD__) || defined(__FreeBSD__) ||        \
    defined(__sun__) || defined(__APPLE__)
#include <inttypes.h>
#elif defined(__linux__) || defined(__WIN32__) || defined(__MINGW32__) ||      \
    defined(__OpenBSD__)
#include <stdint.h>
#endif

#include <errno.h>
#include <stdint.h>
#include <stdlib.h>

#define uswap_16(x) ((((x) & 0xff00) >> 8) | (((x) & 0x00ff) << 8))
#define uswap_32(x)                                                            \
  ((((x) & 0xff000000) >> 24) | (((x) & 0x00ff0000) >> 8) |                    \
   (((x) & 0x0000ff00) << 8) | (((x) & 0x000000ff) << 24))
#define _uswap_64(x, sfx)                                                      \
  ((((x) & 0xff00000000000000##sfx) >> 56) |                                   \
   (((x) & 0x00ff000000000000##sfx) >> 40) |                                   \
   (((x) & 0x0000ff0000000000##sfx) >> 24) |                                   \
   (((x) & 0x000000ff00000000##sfx) >> 8) |                                    \
   (((x) & 0x00000000ff000000##sfx) << 8) |                                    \
   (((x) & 0x0000000000ff0000##sfx) << 24) |                                   \
   (((x) & 0x000000000000ff00##sfx) << 40) |                                   \
   (((x) & 0x00000000000000ff##sfx) << 56))
#if defined(__GNUC__)
#define uswap_64(x) _uswap_64(x, ull)
#else
#define uswap_64(x) _uswap_64(x, )
#endif

#if __BYTE_ORDER == __LITTLE_ENDIAN
#define cpu_to_le16(x) (x)
#define cpu_to_le32(x) (x)
#define cpu_to_le64(x) (x)
#define le16_to_cpu(x) (x)
#define le32_to_cpu(x) (x)
#define le64_to_cpu(x) (x)
#define cpu_to_be16(x) uswap_16(x)
#define cpu_to_be32(x) uswap_32(x)
#define cpu_to_be64(x) uswap_64(x)
#define be16_to_cpu(x) uswap_16(x)
#define be32_to_cpu(x) uswap_32(x)
#define be64_to_cpu(x) uswap_64(x)
#else
#define cpu_to_le16(x) uswap_16(x)
#define cpu_to_le32(x) uswap_32(x)
#define cpu_to_le64(x) uswap_64(x)
#define le16_to_cpu(x) uswap_16(x)
#define le32_to_cpu(x) uswap_32(x)
#define le64_to_cpu(x) uswap_64(x)
#define cpu_to_be16(x) (x)
#define cpu_to_be32(x) (x)
#define cpu_to_be64(x) (x)
#define be16_to_cpu(x) (x)
#define be32_to_cpu(x) (x)
#define be64_to_cpu(x) (x)
#endif

#define __cpu_to_le16 cpu_to_le16
#define __cpu_to_le32 cpu_to_le32
#define __cpu_to_le64 cpu_to_le64
#define __le16_to_cpu le16_to_cpu
#define __le32_to_cpu le32_to_cpu
#define __le64_to_cpu le64_to_cpu
#define __cpu_to_be16 cpu_to_be16
#define __cpu_to_be32 cpu_to_be32
#define __cpu_to_be64 cpu_to_be64
#define __be16_to_cpu be16_to_cpu
#define __be32_to_cpu be32_to_cpu
#define __be64_to_cpu be64_to_cpu
