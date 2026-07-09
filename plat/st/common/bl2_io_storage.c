/*
 * Copyright (c) 2015-2024, Arm Limited and Contributors. All rights reserved.
 *
 * SPDX-License-Identifier: BSD-3-Clause
 */

#include <assert.h>
#include <string.h>

#include <arch_helpers.h>
#include <common/debug.h>
#include <common/desc_image_load.h>
#if BAREMETAL_IMAGE_LOADER
#include <common/tf_crc32.h>
#include <errno.h>
#endif
#include <drivers/auth/auth_mod.h>
#include <drivers/fwu/fwu.h>
#include <drivers/fwu/fwu_metadata.h>
#if STM32MP_HYPERFLASH
#include <drivers/hyperflash.h>
#endif
#include <drivers/io/io_block.h>
#include <drivers/io/io_driver.h>
#include <drivers/io/io_encrypted.h>
#include <drivers/io/io_fip.h>
#include <drivers/io/io_memmap.h>
#include <drivers/io/io_mtd.h>
#include <drivers/io/io_storage.h>
#include <drivers/mmc.h>
#include <drivers/partition/efi.h>
#include <drivers/partition/partition.h>
#include <drivers/raw_nand.h>
#include <drivers/spi_nand.h>
#include <drivers/spi_nor.h>
#include <drivers/st/stm32_fmc2_nand.h>
#ifdef STM32MP1X
#include <drivers/st/stm32_qspi.h>
#endif
#ifdef STM32MP2X
#include <drivers/st/stm32_ospi.h>
#endif
#include <drivers/st/stm32_sdmmc2.h>
#include <drivers/usb_device.h>
#include <lib/fconf/fconf.h>
#include <lib/mmio.h>
#include <lib/utils.h>
#include <plat/common/platform.h>
#include <tools_share/firmware_image_package.h>

#include <platform_def.h>
#include <stm32cubeprogrammer.h>
#include <stm32mp_efi.h>
#include <stm32mp_fconf_getter.h>
#include <stm32mp_io_storage.h>
#include <usb_dfu.h>

/* IO devices */
uintptr_t fip_dev_handle;
uintptr_t storage_dev_handle;

static const io_dev_connector_t *fip_dev_con;
static uint32_t nand_block_sz;

#ifndef DECRYPTION_SUPPORT_none
static const io_dev_connector_t *enc_dev_con;
uintptr_t enc_dev_handle;
#endif

#if STM32MP_SDMMC || STM32MP_EMMC
static struct mmc_device_info mmc_info;

static uint8_t block_buffer[MMC_BLOCK_SIZE] __aligned(MMC_BLOCK_SIZE);

static io_block_dev_spec_t mmc_block_dev_spec = {
	/* It's used as temp buffer in block driver */
	.buffer = {
		.offset = (size_t)&block_buffer,
		.length = MMC_BLOCK_SIZE,
	},
	.ops = {
		.read = mmc_read_blocks,
		.write = NULL,
	},
	.block_size = MMC_BLOCK_SIZE,
};

static const io_dev_connector_t *mmc_dev_con;
#endif /* STM32MP_SDMMC || STM32MP_EMMC */

#if STM32MP_SPI_NOR
static io_mtd_dev_spec_t spi_nor_dev_spec = {
	.ops = {
		.init = spi_nor_init,
		.read = spi_nor_read,
		.reset = spi_nor_reset,
	},
};
#endif

#if STM32MP_RAW_NAND
static io_mtd_dev_spec_t nand_dev_spec = {
	.ops = {
		.init = nand_raw_init,
		.read = nand_read,
		.seek = nand_seek_bb
	},
};

static const io_dev_connector_t *nand_dev_con;
#endif

#if STM32MP_SPI_NAND
static io_mtd_dev_spec_t spi_nand_dev_spec = {
	.ops = {
		.init = spi_nand_init,
		.read = nand_read,
		.seek = nand_seek_bb
	},
};
#endif

#if STM32MP_HYPERFLASH
static io_mtd_dev_spec_t hyperflash_dev_spec = {
	.ops = {
		.init = hyperflash_init,
		.read = hyperflash_read,
	},
};

static const io_dev_connector_t *hyperflash_dev_con;
#endif

#if STM32MP_SPI_NAND || STM32MP_SPI_NOR
static const io_dev_connector_t *spi_dev_con;
#endif

#if STM32MP_UART_PROGRAMMER || STM32MP_USB_PROGRAMMER
static const io_dev_connector_t *memmap_dev_con;
#endif

io_block_spec_t image_block_spec = {
	.offset = 0U,
	.length = 0U,
};

int open_fip(const uintptr_t spec)
{
	return io_dev_init(fip_dev_handle, (uintptr_t)FIP_IMAGE_ID);
}

#ifndef DECRYPTION_SUPPORT_none
int open_enc_fip(const uintptr_t spec)
{
	int result;
	uintptr_t local_image_handle;

	result = io_dev_init(enc_dev_handle, (uintptr_t)ENC_IMAGE_ID);
	if (result != 0) {
		return result;
	}

	result = io_open(enc_dev_handle, spec, &local_image_handle);
	if (result != 0) {
		return result;
	}

	VERBOSE("Using encrypted FIP\n");
	io_close(local_image_handle);

	return 0;
}
#endif

int open_storage(const uintptr_t spec)
{
	return io_dev_init(storage_dev_handle, 0);
}

#if STM32MP_EMMC_BOOT
static uint32_t get_boot_part_fip_header(void)
{
	io_block_spec_t emmc_boot_fip_block_spec = {
		.offset = STM32MP_EMMC_BOOT_FIP_OFFSET,
		.length = MMC_BLOCK_SIZE, /* We are interested only in first 4 bytes */
	};
	uint32_t magic = 0U;
	int io_result;
	size_t bytes_read;
	uintptr_t fip_hdr_handle;

	io_result = io_open(storage_dev_handle, (uintptr_t)&emmc_boot_fip_block_spec,
			    &fip_hdr_handle);
	assert(io_result == 0);

	io_result = io_read(fip_hdr_handle, (uintptr_t)&magic, sizeof(magic),
			    &bytes_read);
	if ((io_result != 0) || (bytes_read != sizeof(magic))) {
		panic();
	}

	io_close(fip_hdr_handle);

	VERBOSE("%s: eMMC boot magic at offset 256K: %08x\n",
		__func__, magic);

	return magic;
}
#endif

static void print_boot_device(boot_api_context_t *boot_context)
{
	switch (boot_context->boot_interface_selected) {
	case BOOT_API_CTX_BOOT_INTERFACE_SEL_FLASH_SD:
		INFO("Using SDMMC\n");
		break;
	case BOOT_API_CTX_BOOT_INTERFACE_SEL_FLASH_EMMC:
		INFO("Using EMMC\n");
		break;
	case BOOT_API_CTX_BOOT_INTERFACE_SEL_FLASH_NOR_SPI:
		INFO("Using SPI NOR\n");
		break;
	case BOOT_API_CTX_BOOT_INTERFACE_SEL_FLASH_NAND_FMC:
		INFO("Using FMC NAND\n");
		break;
	case BOOT_API_CTX_BOOT_INTERFACE_SEL_FLASH_NAND_SPI:
		INFO("Using SPI NAND\n");
		break;
#if STM32MP_HYPERFLASH
	case BOOT_API_CTX_BOOT_INTERFACE_SEL_HYPERFLASH_OSPI:
		INFO("Using HYPERFLASH\n");
		break;
#endif
	case BOOT_API_CTX_BOOT_INTERFACE_SEL_SERIAL_UART:
		INFO("Using UART\n");
		break;
	case BOOT_API_CTX_BOOT_INTERFACE_SEL_SERIAL_USB:
		INFO("Using USB\n");
		break;
	default:
		ERROR("Boot interface %u not found\n",
		      boot_context->boot_interface_selected);
		panic();
		break;
	}

	if (boot_context->boot_interface_instance != 0U) {
		INFO("  Instance %d\n", boot_context->boot_interface_instance);
	}
}

#if STM32MP_SDMMC || STM32MP_EMMC
static void boot_mmc(enum mmc_device_type mmc_dev_type,
		     uint16_t boot_interface_instance)
{
	int io_result __maybe_unused;
	struct stm32_sdmmc2_params params;

	zeromem(&params, sizeof(struct stm32_sdmmc2_params));

	mmc_info.mmc_dev_type = mmc_dev_type;

	switch (boot_interface_instance) {
	case 1:
		params.reg_base = STM32MP_SDMMC1_BASE;
		break;
	case 2:
		params.reg_base = STM32MP_SDMMC2_BASE;
		break;
	case 3:
		params.reg_base = STM32MP_SDMMC3_BASE;
		break;
	default:
		WARN("SDMMC instance not found, using default\n");
		if (mmc_dev_type == MMC_IS_SD) {
			params.reg_base = STM32MP_SDMMC1_BASE;
		} else {
			params.reg_base = STM32MP_SDMMC2_BASE;
		}
		break;
	}

	if (mmc_dev_type != MMC_IS_EMMC) {
		params.flags = MMC_FLAG_SD_CMD6;
	}

	params.device_info = &mmc_info;
	if (stm32_sdmmc2_mmc_init(&params) != 0) {
		ERROR("SDMMC%u init failed\n", boot_interface_instance);
		panic();
	}

	/* Open MMC as a block device to read FIP */
	io_result = register_io_dev_block(&mmc_dev_con);
	if (io_result != 0) {
		panic();
	}

	io_result = io_dev_open(mmc_dev_con, (uintptr_t)&mmc_block_dev_spec,
				&storage_dev_handle);
	assert(io_result == 0);

#if STM32MP_EMMC_BOOT
	if (mmc_dev_type == MMC_IS_EMMC) {
		io_result = mmc_part_switch_current_boot();
		assert(io_result == 0);

		if (get_boot_part_fip_header() != TOC_HEADER_NAME) {
			WARN("%s: Can't find FIP header on eMMC boot partition. Trying GPT\n",
			     __func__);
			io_result = mmc_part_switch_user();
			assert(io_result == 0);
			return;
		}

		VERBOSE("%s: FIP header found on eMMC boot partition\n",
			__func__);
		image_block_spec.offset = STM32MP_EMMC_BOOT_FIP_OFFSET;
		image_block_spec.length = mmc_boot_part_size() - STM32MP_EMMC_BOOT_FIP_OFFSET;
	}
#endif
}
#endif /* STM32MP_SDMMC || STM32MP_EMMC */

#if STM32MP_SPI_NOR
static void boot_spi_nor(boot_api_context_t *boot_context)
{
	int io_result __maybe_unused = 0;

#ifdef STM32MP1X
	io_result = stm32_qspi_init();
#endif
#ifdef STM32MP2X
	io_result = stm32_ospi_init();
#endif

	assert(io_result == 0);

	io_result = register_io_dev_mtd(&spi_dev_con);
	assert(io_result == 0);

	/* Open connections to device */
	io_result = io_dev_open(spi_dev_con,
				(uintptr_t)&spi_nor_dev_spec,
				&storage_dev_handle);
	assert(io_result == 0);
}
#endif /* STM32MP_SPI_NOR */

#if STM32MP_RAW_NAND
static void boot_fmc2_nand(boot_api_context_t *boot_context)
{
	int io_result __maybe_unused;

	io_result = stm32_fmc2_init();
	assert(io_result == 0);

	/* Register the IO device on this platform */
	io_result = register_io_dev_mtd(&nand_dev_con);
	assert(io_result == 0);

	/* Open connections to device */
	io_result = io_dev_open(nand_dev_con, (uintptr_t)&nand_dev_spec,
				&storage_dev_handle);
	assert(io_result == 0);

	nand_block_sz = nand_dev_spec.erase_size;
}
#endif /* STM32MP_RAW_NAND */

#if STM32MP_SPI_NAND
static void boot_spi_nand(boot_api_context_t *boot_context)
{
	int io_result __maybe_unused = 0;

#ifdef STM32MP1X
	io_result = stm32_qspi_init();
#endif
#ifdef STM32MP2X
	io_result = stm32_ospi_init();
#endif
	assert(io_result == 0);

	io_result = register_io_dev_mtd(&spi_dev_con);
	assert(io_result == 0);

	/* Open connections to device */
	io_result = io_dev_open(spi_dev_con,
				(uintptr_t)&spi_nand_dev_spec,
				&storage_dev_handle);
	assert(io_result == 0);

	nand_block_sz = spi_nand_dev_spec.erase_size;
}
#endif /* STM32MP_SPI_NAND */

#if STM32MP_HYPERFLASH
static void boot_hyperflash(boot_api_context_t *boot_context)
{
	int io_result __maybe_unused = 0;

	io_result = stm32_ospi_init();
	assert(io_result == 0);

	io_result = register_io_dev_mtd(&hyperflash_dev_con);
	assert(io_result == 0);

	/* Open connections to device */
	io_result = io_dev_open(hyperflash_dev_con,
				(uintptr_t)&hyperflash_dev_spec,
				&storage_dev_handle);
	assert(io_result == 0);
}
#endif /* STM32MP_HYPERFLASH */

#if STM32MP_UART_PROGRAMMER || STM32MP_USB_PROGRAMMER
static void mmap_io_setup(void)
{
	int io_result __maybe_unused;

	io_result = register_io_dev_memmap(&memmap_dev_con);
	assert(io_result == 0);

	io_result = io_dev_open(memmap_dev_con, (uintptr_t)NULL,
				&storage_dev_handle);
	assert(io_result == 0);
}

#if STM32MP_UART_PROGRAMMER
static void stm32cubeprogrammer_uart(uint8_t phase, uintptr_t base, size_t len)
{
	int ret __maybe_unused;
	boot_api_context_t *boot_context =
		(boot_api_context_t *)stm32mp_get_boot_ctx_address();
	uintptr_t uart_base;

	uart_base = get_uart_address(boot_context->boot_interface_instance);
	ret = stm32cubeprog_uart_load(uart_base, phase, base, len);
	assert(ret == 0);
}
#endif

#if STM32MP_USB_PROGRAMMER
static void stm32cubeprogrammer_usb(uint8_t phase, uintptr_t base, size_t len)
{
	int ret __maybe_unused;
	static struct usb_handle *pdev;

	/* Init USB on platform */
	if (pdev == NULL) {
		pdev = usb_dfu_plat_init();
	}

	ret = stm32cubeprog_usb_load(pdev, phase, base, len);
	assert(ret == 0);
}
#endif
#endif /* STM32MP_UART_PROGRAMMER || STM32MP_USB_PROGRAMMER */

void stm32mp_io_setup(void)
{
	int io_result __maybe_unused;
	boot_api_context_t *boot_context =
		(boot_api_context_t *)stm32mp_get_boot_ctx_address();

	print_boot_device(boot_context);

	if ((boot_context->boot_partition_used_toboot == 1U) ||
	    (boot_context->boot_partition_used_toboot == 2U)) {
		INFO("Boot used partition fsbl%u\n",
		     boot_context->boot_partition_used_toboot);
	}

	io_result = register_io_dev_fip(&fip_dev_con);
	assert(io_result == 0);

	io_result = io_dev_open(fip_dev_con, (uintptr_t)NULL,
				&fip_dev_handle);

#ifndef DECRYPTION_SUPPORT_none
	io_result = register_io_dev_enc(&enc_dev_con);
	assert(io_result == 0);

	io_result = io_dev_open(enc_dev_con, (uintptr_t)NULL,
				&enc_dev_handle);
	assert(io_result == 0);
#endif

	switch (boot_context->boot_interface_selected) {
#if STM32MP_SDMMC
	case BOOT_API_CTX_BOOT_INTERFACE_SEL_FLASH_SD:
		dmbsy();
		boot_mmc(MMC_IS_SD, boot_context->boot_interface_instance);
		break;
#endif
#if STM32MP_EMMC
	case BOOT_API_CTX_BOOT_INTERFACE_SEL_FLASH_EMMC:
		dmbsy();
		boot_mmc(MMC_IS_EMMC, boot_context->boot_interface_instance);
		break;
#endif
#if STM32MP_SPI_NOR
	case BOOT_API_CTX_BOOT_INTERFACE_SEL_FLASH_NOR_SPI:
		dmbsy();
		boot_spi_nor(boot_context);
		break;
#endif
#if STM32MP_RAW_NAND
	case BOOT_API_CTX_BOOT_INTERFACE_SEL_FLASH_NAND_FMC:
		dmbsy();
		boot_fmc2_nand(boot_context);
		break;
#endif
#if STM32MP_SPI_NAND
	case BOOT_API_CTX_BOOT_INTERFACE_SEL_FLASH_NAND_SPI:
		dmbsy();
		boot_spi_nand(boot_context);
		break;
#endif
#if STM32MP_HYPERFLASH
	case BOOT_API_CTX_BOOT_INTERFACE_SEL_HYPERFLASH_OSPI:
		dmbsy();
		boot_hyperflash(boot_context);
		break;
#endif
#if STM32MP_UART_PROGRAMMER || STM32MP_USB_PROGRAMMER
#if STM32MP_UART_PROGRAMMER
	case BOOT_API_CTX_BOOT_INTERFACE_SEL_SERIAL_UART:
#endif
#if STM32MP_USB_PROGRAMMER
	case BOOT_API_CTX_BOOT_INTERFACE_SEL_SERIAL_USB:
#endif
		dmbsy();
		mmap_io_setup();
		break;
#endif

	default:
		ERROR("Boot interface %d not supported\n",
		      boot_context->boot_interface_selected);
		panic();
		break;
	}
}

void stm32mp_io_exit(void)
{
	int io_result __maybe_unused;

	/* Close connection to device */
	io_result = io_dev_close(storage_dev_handle);
	assert(io_result == 0);
}

#if BAREMETAL_IMAGE_LOADER
/*
 * Load the baremetal application from the GPT partition named "app".
 *
 * The partition holds a U-Boot legacy image ("uimg"): a 64-byte big-endian
 * header followed by the raw payload. The header supplies the payload size,
 * load address and entry point, and its two CRC32s let us reject a stale,
 * corrupt or partially-written image. Generated by
 * stm32mp2-baremetal/scripts/uimg_header.py (or U-Boot mkimage).
 */

#define UIMG_MAGIC		U(0x27051956)
#define UIMG_NAME_LEN		32U

struct uimg_header {
	uint32_t magic;		/* Image header magic number (UIMG_MAGIC) */
	uint32_t hcrc;		/* CRC32 of the header, with this field zeroed */
	uint32_t time;		/* Image creation timestamp */
	uint32_t size;		/* Payload size in bytes */
	uint32_t load;		/* Payload load address */
	uint32_t ep;		/* Entry point address */
	uint32_t dcrc;		/* CRC32 of the payload */
	uint8_t os;
	uint8_t arch;
	uint8_t type;
	uint8_t comp;
	uint8_t name[UIMG_NAME_LEN];
} __packed;

#define BAREMETAL_APP_PARTITION	"app"

int stm32mp_load_baremetal_app(bl_mem_params_node_t *bl_mem_params)
{
	static io_block_spec_t app_block_spec; /* io keeps a reference: must persist */
	const partition_entry_t *entry;
	struct uimg_header hdr;
	char name[UIMG_NAME_LEN + 1U];
	uintptr_t handle;
	size_t bytes_read;
	uint32_t size, load, ep, dcrc, hcrc;
	uintptr_t base = bl_mem_params->image_info.image_base;
	uint32_t max_size = bl_mem_params->image_info.image_max_size;
	int ret;

	entry = get_partition_entry(BAREMETAL_APP_PARTITION);
	if (entry == NULL) {
		ERROR("Could NOT find the '%s' partition!\n",
		      BAREMETAL_APP_PARTITION);
		return -ENOENT;
	}

	/* Read and validate the uimg header */
	app_block_spec.offset = entry->start;
	app_block_spec.length = sizeof(hdr);

	ret = io_open(storage_dev_handle, (uintptr_t)&app_block_spec, &handle);
	if (ret != 0) {
		return ret;
	}
	ret = io_read(handle, (uintptr_t)&hdr, sizeof(hdr), &bytes_read);
	io_close(handle);
	if ((ret != 0) || (bytes_read != sizeof(hdr))) {
		ERROR("Failed to read app image header (%d)\n", ret);
		return (ret != 0) ? ret : -EIO;
	}

	if (__builtin_bswap32(hdr.magic) != UIMG_MAGIC) {
		ERROR("'%s' partition does not hold a uimg image (magic %x)\n",
		      BAREMETAL_APP_PARTITION, __builtin_bswap32(hdr.magic));
		return -EINVAL;
	}

	hcrc = __builtin_bswap32(hdr.hcrc);
	hdr.hcrc = 0U;
	if (tf_crc32(0U, (const unsigned char *)&hdr, sizeof(hdr)) != hcrc) {
		ERROR("App image header CRC mismatch\n");
		return -EINVAL;
	}

	size = __builtin_bswap32(hdr.size);
	load = __builtin_bswap32(hdr.load);
	ep = __builtin_bswap32(hdr.ep);
	dcrc = __builtin_bswap32(hdr.dcrc);

	if ((size == 0U) || (size > max_size) ||
	    (load < base) || ((load + size) > (base + max_size)) ||
	    (ep < load) || (ep >= (load + size))) {
		ERROR("App image rejected: size %x load %x entry %x (allowed: %x..%x)\n",
		      size, load, ep, (uint32_t)base, (uint32_t)(base + max_size));
		return -EINVAL;
	}

	if ((entry->length < sizeof(hdr)) ||
	    (size > (entry->length - sizeof(hdr)))) {
		ERROR("App image (%x bytes) exceeds the '%s' partition\n",
		      size, BAREMETAL_APP_PARTITION);
		return -EINVAL;
	}

#if STM32MP_SDMMC || STM32MP_EMMC
	/*
	 * The payload starts 64 bytes into the partition, so every block is
	 * read through the block driver's bounce buffer. Temporarily use free
	 * DDR above the app region instead of the default single-block buffer:
	 * the whole payload transfers in a few large chunks. DDR is fully
	 * mapped and initialized by this point (bl2_platform_setup).
	 */
	mmc_block_dev_spec.buffer.offset = base + max_size;
	mmc_block_dev_spec.buffer.length = UL(0x100000);
#endif

	/*
	 * Read the payload, which starts sizeof(hdr) bytes into the partition.
	 * The spec offset must stay block-aligned: io_block computes its
	 * intra-block skip from the seek position only, and silently returns
	 * shifted data if the spec base itself is unaligned. So open the spec
	 * at the partition start and seek past the header instead.
	 */
	app_block_spec.offset = entry->start;
	app_block_spec.length = sizeof(hdr) + size;

	ret = io_open(storage_dev_handle, (uintptr_t)&app_block_spec, &handle);
	if (ret == 0) {
		ret = io_seek(handle, IO_SEEK_SET, sizeof(hdr));
		if (ret == 0) {
			ret = io_read(handle, (uintptr_t)load, size, &bytes_read);
		}
		io_close(handle);
		if ((ret == 0) && (bytes_read != size)) {
			ret = -EIO;
		}
	}

#if STM32MP_SDMMC || STM32MP_EMMC
	mmc_block_dev_spec.buffer.offset = (size_t)&block_buffer;
	mmc_block_dev_spec.buffer.length = MMC_BLOCK_SIZE;
#endif

	if (ret != 0) {
		ERROR("Failed to read app image payload (%d)\n", ret);
		return ret;
	}

	if (tf_crc32(0U, (const unsigned char *)(uintptr_t)load, size) != dcrc) {
		ERROR("App image payload CRC mismatch\n");
		return -EINVAL;
	}

	flush_dcache_range(load, size);

	memcpy(name, hdr.name, UIMG_NAME_LEN);
	name[UIMG_NAME_LEN] = '\0';
	INFO("Loaded app '%s': %u bytes at %x, entry %x\n", name, size, load, ep);

	bl_mem_params->image_info.image_base = load;
	bl_mem_params->image_info.image_size = size;
	bl_mem_params->ep_info.pc = ep;

	return 0;
}
#endif /* BAREMETAL_IMAGE_LOADER */

int bl2_plat_handle_pre_image_load(unsigned int image_id)
{
	static bool gpt_init_done __maybe_unused;
	uint16_t boot_itf = stm32mp_get_boot_itf_selected();

	if (stm32mp_skip_boot_device_after_standby()) {
		return 0;
	}

	switch (boot_itf) {
#if STM32MP_SDMMC || STM32MP_EMMC
	case BOOT_API_CTX_BOOT_INTERFACE_SEL_FLASH_EMMC:
#if STM32MP_EMMC_BOOT
		if (image_block_spec.offset == STM32MP_EMMC_BOOT_FIP_OFFSET) {
			break;
		}
#endif
		/* fallthrough */
	case BOOT_API_CTX_BOOT_INTERFACE_SEL_FLASH_SD:
		if (!gpt_init_done) {
/*
 * With FWU Multi Bank feature enabled, the selection of
 * the image to boot will be done by fwu_init calling the
 * platform hook, plat_fwu_set_images_source.
 */
#if !PSA_FWU_SUPPORT
			const partition_entry_t *entry;
			const struct efi_guid fip_guid = STM32MP_FIP_GUID;

			partition_init(GPT_IMAGE_ID);
			entry = get_partition_entry_by_type(&fip_guid);
			if (entry == NULL) {
				entry = get_partition_entry(FIP_IMAGE_NAME);
				if (entry == NULL) {
					ERROR("Could NOT find the %s partition!\n",
					      FIP_IMAGE_NAME);

					return -ENOENT;
				}
			}

			image_block_spec.offset = entry->start;
			image_block_spec.length = entry->length;
#endif
			gpt_init_done = true;
		} else {
			bl_mem_params_node_t *bl_mem_params = get_bl_mem_params_node(image_id);

			assert(bl_mem_params != NULL);

			mmc_block_dev_spec.buffer.offset = bl_mem_params->image_info.image_base;
			mmc_block_dev_spec.buffer.length = bl_mem_params->image_info.image_max_size;
		}

		break;
#endif

#if STM32MP_RAW_NAND || STM32MP_SPI_NAND
#if STM32MP_RAW_NAND
	case BOOT_API_CTX_BOOT_INTERFACE_SEL_FLASH_NAND_FMC:
#endif
#if STM32MP_SPI_NAND
	case BOOT_API_CTX_BOOT_INTERFACE_SEL_FLASH_NAND_SPI:
#endif
/*
 * With FWU Multi Bank feature enabled, the selection of
 * the image to boot will be done by fwu_init calling the
 * platform hook, plat_fwu_set_images_source.
 */
#if !PSA_FWU_SUPPORT
		image_block_spec.offset = STM32MP_NAND_FIP_OFFSET;
#endif
		break;
#endif

#if STM32MP_SPI_NOR
	case BOOT_API_CTX_BOOT_INTERFACE_SEL_FLASH_NOR_SPI:
/*
 * With FWU Multi Bank feature enabled, the selection of
 * the image to boot will be done by fwu_init calling the
 * platform hook, plat_fwu_set_images_source.
 */
#if !PSA_FWU_SUPPORT
		image_block_spec.offset = STM32MP_NOR_FIP_OFFSET;
#endif
		break;
#endif

#if STM32MP_HYPERFLASH
	case BOOT_API_CTX_BOOT_INTERFACE_SEL_HYPERFLASH_OSPI:
/*
 * With FWU Multi Bank feature enabled, the selection of
 * the image to boot will be done by fwu_init calling the
 * platform hook, plat_fwu_set_images_source.
 */
#if !PSA_FWU_SUPPORT
		image_block_spec.offset = STM32MP_HYPERFLASH_FIP_OFFSET;
#endif
		break;
#endif

#if STM32MP_UART_PROGRAMMER
	case BOOT_API_CTX_BOOT_INTERFACE_SEL_SERIAL_UART:
#if STM32MP_DDR_FIP_IO_STORAGE
		if (image_id == DDR_FW_ID) {
			stm32cubeprogrammer_uart(PHASE_DDR_FW,
						 DWL_DDR_BUFFER_BASE,
						 DWL_DDR_BUFFER_SIZE);
			/* FIP loaded at DWL address */
			image_block_spec.offset = DWL_DDR_BUFFER_BASE;
			image_block_spec.length = DWL_DDR_BUFFER_SIZE;
		}
#endif
		if (image_id == FW_CONFIG_ID) {
#if STM32MP_DDR_FIP_IO_STORAGE && TRUSTED_BOARD_BOOT
			/*
			 * Clear authentication state of STM32MP certificate that will
			 * now be loaded from other FIP file
			 */
			auth_img_flags[STM32MP_CONFIG_CERT_ID] = 0U;
#endif
			stm32cubeprogrammer_uart(PHASE_SSBL,
						 DWL_BUFFER_BASE,
						 DWL_BUFFER_SIZE);
			/* FIP loaded at DWL address */
			image_block_spec.offset = DWL_BUFFER_BASE;
			image_block_spec.length = DWL_BUFFER_SIZE;
		}
		break;
#endif
#if STM32MP_USB_PROGRAMMER
	case BOOT_API_CTX_BOOT_INTERFACE_SEL_SERIAL_USB:
#if STM32MP_DDR_FIP_IO_STORAGE
		if (image_id == DDR_FW_ID) {
			stm32cubeprogrammer_usb(PHASE_DDR_FW,
						DWL_DDR_BUFFER_BASE,
						DWL_DDR_BUFFER_SIZE);
			/* FIP loaded at DWL address */
			image_block_spec.offset = DWL_DDR_BUFFER_BASE;
			image_block_spec.length = DWL_DDR_BUFFER_SIZE;
		}
#endif
		if (image_id == FW_CONFIG_ID) {
#if STM32MP_DDR_FIP_IO_STORAGE && TRUSTED_BOARD_BOOT
			/*
			 * Clear authentication state of STM32MP certificate that will
			 * now be loaded from other FIP file
			 */
			auth_img_flags[STM32MP_CONFIG_CERT_ID] = 0U;
#endif
			stm32cubeprogrammer_usb(PHASE_SSBL,
						DWL_BUFFER_BASE,
						DWL_BUFFER_SIZE);
			/* FIP loaded at DWL address */
			image_block_spec.offset = DWL_BUFFER_BASE;
			image_block_spec.length = DWL_BUFFER_SIZE;
		}
		break;
#endif

	default:
		ERROR("FIP Not found\n");
		panic();
	}

	return 0;
}

/*
 * Return an IO device handle and specification which can be used to access
 * an image. Use this to enforce platform load policy.
 */
int plat_get_image_source(unsigned int image_id, uintptr_t *dev_handle,
			  uintptr_t *image_spec)
{
	int rc;
	const struct plat_io_policy *policy;

	policy = FCONF_GET_PROPERTY(stm32mp, io_policies, image_id);
	rc = policy->check(policy->image_spec);
	if (rc == 0) {
		*image_spec = policy->image_spec;
		*dev_handle = *(policy->dev_handle);
	}

	return rc;
}

/*
 * This function shall return 0 if it cannot find an alternate
 * image to be loaded or it returns 1 otherwise.
 */
int plat_try_backup_partitions(unsigned int image_id)
{
	static unsigned int backup_id;
	static unsigned int backup_block_nb;

	/* Check if NAND storage used */
	if (nand_block_sz == 0U) {
		return 0;
	}

	if (backup_id != image_id) {
		backup_block_nb = PLATFORM_MTD_MAX_PART_SIZE / nand_block_sz;
		backup_id = image_id;
	}

	if (backup_block_nb-- == 0U) {
		return 0;
	}

#if PSA_FWU_SUPPORT
	if (((image_block_spec.offset < STM32MP_NAND_FIP_B_OFFSET) &&
	     ((image_block_spec.offset + nand_block_sz) >= STM32MP_NAND_FIP_B_OFFSET)) ||
	    (image_block_spec.offset + nand_block_sz >= STM32MP_NAND_FIP_B_MAX_OFFSET)) {
		return 0;
	}
#endif

	image_block_spec.offset += nand_block_sz;

	return 1;
}

#if PSA_FWU_SUPPORT
/*
 * In each boot in non-trial mode, we set the BKP register to
 * FWU_MAX_TRIAL_REBOOT, and return the active_index from metadata.
 *
 * As long as the update agent didn't update the "accepted" field in metadata
 * (i.e. we are in trial mode), we select the new active_index.
 * To avoid infinite boot loop at trial boot we decrement a BKP register.
 * If this counter is 0:
 *     - an unexpected TAMPER event raised (that resets the BKP registers to 0)
 *     - a power-off occurs before the update agent was able to update the
 *       "accepted' field
 *     - we already boot FWU_MAX_TRIAL_REBOOT times in trial mode.
 * we select the previous_active_index.
 */
uint32_t plat_fwu_get_boot_idx(void)
{
	/*
	 * Select boot index and update boot counter only once per boot
	 * even if this function is called several times.
	 */
	static uint32_t boot_idx = INVALID_BOOT_IDX;
	int err = 0;

	if (boot_idx == INVALID_BOOT_IDX) {
		const struct fwu_metadata *data = fwu_get_metadata();
		uint32_t bootcount = 0;

		boot_idx = data->active_index;

		switch (data->bank_state[boot_idx]) {
		case FWU_BANK_STATE_ACCEPTED:
			err = stm32_set_max_fwu_trial_boot_cnt();
			break;
		case FWU_BANK_STATE_VALID:
			err = stm32_get_and_dec_fwu_trial_boot_cnt(&bootcount);
			if (err == 0) {
				if (bootcount == 1U) {
					WARN("Trial FWU fails %u times\n",
					     (FWU_MAX_TRIAL_REBOOT - 1U));
					boot_idx = fwu_get_alternate_boot_bank();
				} else if (bootcount == 0U) {
					WARN("Trial backup register empty : set max boot count\n");
					err = stm32_set_max_fwu_trial_boot_cnt();
				} else {
					VERBOSE("Trial FWU: %u\n",
						FWU_MAX_TRIAL_REBOOT - bootcount);
				}
			}
			break;
		case FWU_BANK_STATE_INVALID:
		default:
			ERROR("The active bank(%u) of the platform is in Invalid State.\n",
			      boot_idx);
			boot_idx = fwu_get_alternate_boot_bank();
			err = stm32_clear_fwu_trial_boot_cnt();
			break;
		}

		if (err != 0) {
			ERROR("%s: Bkp register access failed. Bank state: %d\n",
				__func__, data->bank_state[boot_idx]);
			panic();
		}
	}

	return boot_idx;
}

static void *stm32_get_image_spec(const struct efi_guid *img_type_guid)
{
	unsigned int i;

	for (i = 0U; i < MAX_NUMBER_IDS; i++) {
		if ((guidcmp(&policies[i].img_type_guid, img_type_guid)) == 0) {
			return (void *)policies[i].image_spec;
		}
	}

	return NULL;
}

void plat_fwu_set_images_source(const struct fwu_metadata *metadata)
{
	unsigned int i;
	uint32_t boot_idx;
	const partition_entry_t *entry __maybe_unused;
	const struct fwu_image_entry *img_entry;
	const void *img_type_guid;
	const void *img_guid;
	io_block_spec_t *image_spec;
	const uint16_t boot_itf = stm32mp_get_boot_itf_selected();

	boot_idx = plat_fwu_get_boot_idx();
	assert(boot_idx < NR_OF_FW_BANKS);
	VERBOSE("Selecting to boot from bank %u\n", boot_idx);

	img_entry = (void *)&metadata->fw_desc.img_entry;
	for (i = 0U; i < NR_OF_IMAGES_IN_FW_BANK; i++) {
		img_type_guid = &img_entry[i].img_type_guid;

		img_guid = &img_entry[i].img_bank_info[boot_idx].img_guid;

		image_spec = stm32_get_image_spec(img_type_guid);
		if (image_spec == NULL) {
			ERROR("Unable to get image spec for the image in the metadata\n");
			panic();
		}

		switch (boot_itf) {
#if (STM32MP_SDMMC || STM32MP_EMMC)
		case BOOT_API_CTX_BOOT_INTERFACE_SEL_FLASH_SD:
		case BOOT_API_CTX_BOOT_INTERFACE_SEL_FLASH_EMMC:
			entry = get_partition_entry_by_guid(img_guid);
			if (entry == NULL) {
				ERROR("No partition with the uuid mentioned in metadata\n");
				panic();
			}

			image_spec->offset = entry->start;
			image_spec->length = entry->length;
			break;
#endif
#if STM32MP_SPI_NOR
		case BOOT_API_CTX_BOOT_INTERFACE_SEL_FLASH_NOR_SPI:
			if (guidcmp(img_guid, &STM32MP_NOR_FIP_A_GUID) == 0) {
				image_spec->offset = STM32MP_NOR_FIP_A_OFFSET;
			} else if (guidcmp(img_guid, &STM32MP_NOR_FIP_B_GUID) == 0) {
				image_spec->offset = STM32MP_NOR_FIP_B_OFFSET;
			} else {
				ERROR("Invalid uuid mentioned in metadata\n");
				panic();
			}
			break;
#endif
#if (STM32MP_RAW_NAND || STM32MP_SPI_NAND)
		case BOOT_API_CTX_BOOT_INTERFACE_SEL_FLASH_NAND_FMC:
		case BOOT_API_CTX_BOOT_INTERFACE_SEL_FLASH_NAND_SPI:
			if (guidcmp(img_guid, &STM32MP_NAND_FIP_A_GUID) == 0) {
				image_spec->offset = STM32MP_NAND_FIP_A_OFFSET;
			} else if (guidcmp(img_guid, &STM32MP_NAND_FIP_B_GUID) == 0) {
				image_spec->offset = STM32MP_NAND_FIP_B_OFFSET;
			} else {
				ERROR("Invalid uuid mentioned in metadata\n");
				panic();
			}
			break;
#endif
#if STM32MP_HYPERFLASH
		case BOOT_API_CTX_BOOT_INTERFACE_SEL_HYPERFLASH_OSPI:
			if (guidcmp(img_guid, &STM32MP_HYPERFLASH_FIP_A_GUID) == 0) {
				image_spec->offset = STM32MP_HYPERFLASH_FIP_A_OFFSET;
			} else if (guidcmp(img_guid, &STM32MP_HYPERFLASH_FIP_B_GUID) == 0) {
				image_spec->offset = STM32MP_HYPERFLASH_FIP_B_OFFSET;
			} else {
				ERROR("Invalid uuid mentioned in metadata\n");
				panic();
			}
			break;
#endif
		default:
			panic();
			break;
		}
	}
}

static int plat_set_image_source(unsigned int image_id,
				 uintptr_t *handle,
				 uintptr_t *image_spec)
{
	struct plat_io_policy *policy;
	io_block_spec_t *spec __maybe_unused;
	const partition_entry_t *entry __maybe_unused;
	const uint16_t boot_itf = stm32mp_get_boot_itf_selected();
	const struct efi_guid metadata_type_guid __maybe_unused = FWU_METADATA_GUID;

	policy = &policies[image_id];
	spec = (io_block_spec_t *)policy->image_spec;

	switch (boot_itf) {
#if (STM32MP_SDMMC || STM32MP_EMMC)
	case BOOT_API_CTX_BOOT_INTERFACE_SEL_FLASH_SD:
	case BOOT_API_CTX_BOOT_INTERFACE_SEL_FLASH_EMMC:
		partition_init(GPT_IMAGE_ID);

		entry = get_partition_entry_by_type(&metadata_type_guid);
		if (entry == NULL) {
			entry = (image_id == FWU_METADATA_IMAGE_ID) ?
				get_partition_entry(METADATA_PART_1) :
				get_partition_entry(METADATA_PART_2);


			if (entry == NULL) {
				ERROR("Unable to find a metadata partition\n");
				return -ENOENT;
			}
		}

		spec->offset = entry->start;
		spec->length = entry->length;
		break;
#endif

#if STM32MP_SPI_NOR
	case BOOT_API_CTX_BOOT_INTERFACE_SEL_FLASH_NOR_SPI:
		if (image_id == FWU_METADATA_IMAGE_ID) {
			spec->offset = STM32MP_NOR_METADATA1_OFFSET;
		} else {
			spec->offset = STM32MP_NOR_METADATA2_OFFSET;
		}

		spec->length = sizeof(struct fwu_metadata);
		break;
#endif

#if (STM32MP_RAW_NAND || STM32MP_SPI_NAND)
	case BOOT_API_CTX_BOOT_INTERFACE_SEL_FLASH_NAND_FMC:
	case BOOT_API_CTX_BOOT_INTERFACE_SEL_FLASH_NAND_SPI:
		if (image_id == FWU_METADATA_IMAGE_ID) {
			spec->offset = STM32MP_NAND_METADATA1_OFFSET;
		} else {
			spec->offset = STM32MP_NAND_METADATA2_OFFSET;
		}

		spec->length = sizeof(struct fwu_metadata);
		break;
#endif

#if STM32MP_HYPERFLASH
		case BOOT_API_CTX_BOOT_INTERFACE_SEL_HYPERFLASH_OSPI:
		if (image_id == FWU_METADATA_IMAGE_ID) {
			spec->offset = STM32MP_HYPERFLASH_METADATA1_OFFSET;
		} else {
			spec->offset = STM32MP_HYPERFLASH_METADATA2_OFFSET;
		}

		spec->length = sizeof(struct fwu_metadata);
		break;
#endif

	default:
		panic();
		break;
	}
	*image_spec = policy->image_spec;
	*handle = *policy->dev_handle;

	return 0;
}

int plat_fwu_set_metadata_image_source(unsigned int image_id,
				       uintptr_t *handle,
				       uintptr_t *image_spec)
{
	assert((image_id == FWU_METADATA_IMAGE_ID) ||
	       (image_id == BKUP_FWU_METADATA_IMAGE_ID));

	return plat_set_image_source(image_id, handle, image_spec);
}

bool plat_fwu_is_enabled(void)
{
	return !stm32mp_skip_boot_device_after_standby();
}
#endif /* PSA_FWU_SUPPORT */
