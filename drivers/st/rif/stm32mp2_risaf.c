/*
 * Copyright (c) 2021-2022, STMicroelectronics - All Rights Reserved
 *
 * SPDX-License-Identifier: BSD-3-Clause
 */

#include <assert.h>
#include <errno.h>
#include <stdint.h>

#include <arch_helpers.h>
#include <common/debug.h>
#include <drivers/delay_timer.h>
#include <drivers/st/stm32mp2_risaf.h>
#include <dt-bindings/soc/rif.h>
#include <lib/mmio.h>
#include <lib/utils_def.h>
#include <libfdt.h>

#include <platform_def.h>
#include <plat/common/platform.h>
#include <stm32mp_fconf_getter.h>

/* RISAF general registers (base relative) */
#define _RISAF_CR			U(0x00)
#define _RISAF_SR			U(0x04)
#define _RISAF_KEYR			U(0x30)
#define _RISAF_HWCFGR			U(0xFF0)

/* RISAF general register field description */
/* _RISAF_CR register fields */
#define _RISAF_CR_GLOCK			BIT(0)
/* _RISAF_SR register fields */
#define _RISAF_SR_KEYVALID		BIT(0)
#define _RISAF_SR_KEYRDY		BIT(1)
#define _RISAF_SR_ENCDIS		BIT(2)
/* _RISAF_HWCFGR register fields */
#define _RISAF_HWCFGR_CFG1_SHIFT	0
#define _RISAF_HWCFGR_CFG1_MASK		GENMASK_32(7, 0)
#define _RISAF_HWCFGR_CFG2_SHIFT	8
#define _RISAF_HWCFGR_CFG2_MASK		GENMASK_32(15, 8)
#define _RISAF_HWCFGR_CFG3_SHIFT	16
#define _RISAF_HWCFGR_CFG3_MASK		GENMASK_32(23, 16)
#define _RISAF_HWCFGR_CFG4_SHIFT	24
#define _RISAF_HWCFGR_CFG4_MASK		GENMASK_32(31, 24)

/* RISAF region registers (base relative) */
#define _RISAF_REG_BASE			U(0x40)
#define _RISAF_REG_SIZE			U(0x40)
#define _RISAF_REG(n)			(_RISAF_REG_BASE + (((n) - 1) * _RISAF_REG_SIZE))
#define _RISAF_REG_CFGR_OFFSET		U(0x0)
#define _RISAF_REG_CFGR(n)		(_RISAF_REG(n) + _RISAF_REG_CFGR_OFFSET)
#define _RISAF_REG_STARTR_OFFSET	U(0x4)
#define _RISAF_REG_STARTR(n)		(_RISAF_REG(n) + _RISAF_REG_STARTR_OFFSET)
#define _RISAF_REG_ENDR_OFFSET		U(0x8)
#define _RISAF_REG_ENDR(n)		(_RISAF_REG(n) + _RISAF_REG_ENDR_OFFSET)
#define _RISAF_REG_CIDCFGR_OFFSET	U(0xC)
#define _RISAF_REG_CIDCFGR(n)		(_RISAF_REG(n) + _RISAF_REG_CIDCFGR_OFFSET)

/* RISAF region register field description */
/* _RISAF_REG_CFGR(n) register fields */
#define _RISAF_REG_CFGR_BREN_SHIFT	0
#define _RISAF_REG_CFGR_BREN		BIT(_RISAF_REG_CFGR_BREN_SHIFT)
#define _RISAF_REG_CFGR_SEC_SHIFT	8
#define _RISAF_REG_CFGR_SEC		BIT(_RISAF_REG_CFGR_SEC_SHIFT)
#define _RISAF_REG_CFGR_ENC_SHIFT	15
#define _RISAF_REG_CFGR_ENC		BIT(_RISAF_REG_CFGR_ENC_SHIFT)
#define _RISAF_REG_CFGR_PRIVC_SHIFT	16
#define _RISAF_REG_CFGR_PRIVC_MASK	GENMASK_32(23, 16)
#define _RISAF_REG_CFGR_ALL_MASK	(_RISAF_REG_CFGR_BREN | _RISAF_REG_CFGR_SEC | \
					 _RISAF_REG_CFGR_ENC | _RISAF_REG_CFGR_PRIVC_MASK)
/* _RISAF_REG_CIDCFGR(n) register fields */
#define _RISAF_REG_CIDCFGR_RDENC_SHIFT		0
#define _RISAF_REG_CIDCFGR_RDENC_MASK		GENMASK_32(7, 0)
#define _RISAF_REG_CIDCFGR_WRENC_SHIFT		16
#define _RISAF_REG_CIDCFGR_WRENC_MASK		GENMASK_32(23, 16)
#define _RISAF_REG_CIDCFGR_ALL_MASK		(_RISAF_REG_CIDCFGR_RDENC_MASK | \
						 _RISAF_REG_CIDCFGR_WRENC_MASK)

/* Device Tree related definitions */
#define DT_RISAF_COMPAT			"st,stm32-risaf"
#define DT_RISAF_REG_ID_MASK		U(0xF)
#define DT_RISAF_EN_SHIFT		5
#define DT_RISAF_EN_MASK		BIT(DT_RISAF_EN_SHIFT)
#define DT_RISAF_SEC_SHIFT		6
#define DT_RISAF_SEC_MASK		BIT(DT_RISAF_SEC_SHIFT)
#define DT_RISAF_ENC_SHIFT		7
#define DT_RISAF_ENC_MASK		BIT(DT_RISAF_ENC_SHIFT)
#define DT_RISAF_PRIV_SHIFT		8
#define DT_RISAF_PRIV_MASK		GENMASK_32(15, 8)
#define DT_RISAF_READ_SHIFT		16
#define DT_RISAF_READ_MASK		GENMASK_32(23, 16)
#define DT_RISAF_WRITE_SHIFT		24
#define DT_RISAF_WRITE_MASK		GENMASK_32(31, 24)

/* RISAF max properties */
#define RISAF_REGION_DT_PARAM		3
#define RISAF_REGION_DT_SIZE		(RISAF_REGION_DT_PARAM * sizeof(uint32_t))
#define RISAF_TIMEOUT_1MS_IN_US		U(1000)

#pragma weak stm32_risaf_get_instance
#pragma weak stm32_risaf_get_base
#pragma weak stm32_risaf_get_max_region
#pragma weak stm32_risaf_get_memory_base
#pragma weak stm32_risaf_get_memory_size
#pragma weak stm32_risaf_preconf_process
#pragma weak stm32_risaf_postconf_process

struct stm32mp2_risaf_region {
	int instance;
	uint32_t cfg;
	uint32_t addr;
	uint32_t len;
};

struct stm32mp2_risaf_platdata {
	uintptr_t *base;
	struct stm32mp2_risaf_region *region;
	int nregions;
};

struct risaf_dt_id_attr {
	fdt32_t id_attr[RISAF_MAX_REGION * RISAF_REGION_DT_PARAM];
};

static struct stm32mp2_risaf_platdata stm32mp2_risaf;

int stm32_risaf_get_instance(uintptr_t base)
{
	return -ENODEV;
}

uintptr_t stm32_risaf_get_base(int instance)
{
	return 0U;
}

int stm32_risaf_get_max_region(int instance)
{
	return -ENODEV;
}

uintptr_t stm32_risaf_get_memory_base(int instance)
{
	return 0U;
}

size_t stm32_risaf_get_memory_size(int instance)
{
	return 0U;
}

void stm32_risaf_preconf_process(uint32_t inst_mask)
{
}

void stm32_risaf_postconf_process(uint32_t inst_mask)
{
}

#if ENABLE_ASSERTIONS
static bool valid_protreg_id(int instance, uint32_t id)
{
	uint32_t max_id;

	max_id = mmio_read_32(stm32mp2_risaf.base[instance] + _RISAF_HWCFGR);
	max_id = (max_id & _RISAF_HWCFGR_CFG1_MASK) >> _RISAF_HWCFGR_CFG1_SHIFT;

	return id < max_id;
}

static bool valid_instance(int instance)
{
	return (instance < RISAF_MAX_INSTANCE) && (stm32mp2_risaf.base[instance] != 0U);
}
#endif

static bool risaf_is_hw_encryption_functional(int instance)
{
	return (mmio_read_32(stm32mp2_risaf.base[instance] + _RISAF_SR) & _RISAF_SR_ENCDIS) !=
	       _RISAF_SR_ENCDIS;
}

static int check_region_address(int instance, uint32_t addr, uint32_t len)
{
	uint32_t granularity;
	uint64_t end_address;
	uintptr_t mem_base = stm32_risaf_get_memory_base(instance);

	if (addr < mem_base) {
		ERROR("RISAF%d: region start address lower than memory base\n", instance + 1);
		return -EINVAL;
	}

	if (len == 0) {
		ERROR("RISAF%d: region length is equal to zero\n", instance + 1);
		return -EINVAL;
	}

	/* Get physical end address */
	end_address = mem_base + stm32_risaf_get_memory_size(instance) - 1U;
	if (((uint64_t)addr > end_address) || ((uint64_t)(addr - 1U + len) > end_address)) {
		ERROR("RISAF%d: start/end address higher than physical end\n", instance + 1);
		return -EINVAL;
	}

	/* Get IP region granularity */
	granularity = mmio_read_32(stm32mp2_risaf.base[instance] + _RISAF_HWCFGR);
	granularity = BIT((granularity & _RISAF_HWCFGR_CFG3_MASK) >> _RISAF_HWCFGR_CFG3_SHIFT);
	if (((addr % granularity) != 0U) || ((len % granularity) != 0U)) {
		ERROR("RISAF%d: start/end address granularity not respected\n", instance + 1);
		return -EINVAL;
	}

	return 0;
}

static int risaf_configure_region(int instance, uint32_t region_id, uint32_t cfg,
				  uint32_t cid_cfg, uint32_t saddr, uint32_t eaddr)
{
	uintptr_t base = stm32mp2_risaf.base[instance];
	uint32_t hwcfgr;
	uint32_t mask_lsb;
	uint32_t mask_msb;
	uint32_t mask;

	assert(valid_instance(instance));
	assert(valid_protreg_id(instance, region_id));

	mmio_clrbits_32(base + _RISAF_REG_CFGR(region_id), _RISAF_REG_CFGR_BREN);

	/* Get address mask depending on RISAF instance HW configuration */
	hwcfgr =  mmio_read_32(base + _RISAF_HWCFGR);
	mask_lsb = (hwcfgr & _RISAF_HWCFGR_CFG3_MASK) >> _RISAF_HWCFGR_CFG3_SHIFT;
	mask_msb = mask_lsb + ((hwcfgr & _RISAF_HWCFGR_CFG4_MASK) >> _RISAF_HWCFGR_CFG4_SHIFT) - 1U;
	mask = GENMASK_32(mask_msb, mask_lsb);

	mmio_clrsetbits_32(base + _RISAF_REG_STARTR(region_id), mask,
			   (saddr - stm32_risaf_get_memory_base(instance)) & mask);
	mmio_clrsetbits_32(base + _RISAF_REG_ENDR(region_id), mask,
			   (eaddr - stm32_risaf_get_memory_base(instance)) & mask);

	mmio_clrsetbits_32(base + _RISAF_REG_CIDCFGR(region_id), _RISAF_REG_CIDCFGR_ALL_MASK,
			   cid_cfg & _RISAF_REG_CIDCFGR_ALL_MASK);

	mmio_clrsetbits_32(base + _RISAF_REG_CFGR(region_id),
			   _RISAF_REG_CFGR_ALL_MASK, cfg & _RISAF_REG_CFGR_ALL_MASK);

	if ((cfg & _RISAF_REG_CFGR_ENC) == _RISAF_REG_CFGR_ENC) {
		if (!risaf_is_hw_encryption_functional(instance)) {
			ERROR("RISAF%d: encryption feature error\n", instance + 1);
			return -EIO;
		}
	}

	return 0;
}

static void risaf_conf_protreg(void)
{
	struct stm32mp2_risaf_platdata *pdata = &stm32mp2_risaf;
	int idx;

	for (idx = 0; idx < RISAF_MAX_INSTANCE; idx++) {
		int n;

		if (pdata->base[idx] == 0) {
			continue;
		}

		for (n = 0; n < pdata->nregions; n++) {
			uint32_t id;
			uint32_t value;
			uint32_t cfg;
			uint32_t cid_cfg;
			uint32_t start_addr;
			uint32_t end_addr;

			if (pdata->region[n].instance != idx) {
				continue;
			}

			if (check_region_address(idx, pdata->region[n].addr,
						 pdata->region[n].len) != 0) {
				panic();
			}

			value = pdata->region[n].cfg;
			id = (value & DT_RISAF_REG_ID_MASK);
			assert(valid_protreg_id(idx, id));

			cfg = (((value & DT_RISAF_EN_MASK) >> DT_RISAF_EN_SHIFT) <<
			       _RISAF_REG_CFGR_BREN_SHIFT) |
			      (((value & DT_RISAF_SEC_MASK) >> DT_RISAF_SEC_SHIFT) <<
			       _RISAF_REG_CFGR_SEC_SHIFT) |
			      (((value & DT_RISAF_ENC_MASK) >> DT_RISAF_ENC_SHIFT) <<
			       _RISAF_REG_CFGR_ENC_SHIFT) |
			      (((value & DT_RISAF_PRIV_MASK) >> DT_RISAF_PRIV_SHIFT) <<
			       _RISAF_REG_CFGR_PRIVC_SHIFT);

			cid_cfg = (((value & DT_RISAF_WRITE_MASK) >> DT_RISAF_WRITE_SHIFT) <<
				   _RISAF_REG_CIDCFGR_WRENC_SHIFT) |
				  (((value & DT_RISAF_READ_MASK) >> DT_RISAF_READ_SHIFT) <<
				   _RISAF_REG_CIDCFGR_RDENC_SHIFT);

			start_addr = pdata->region[n].addr;
			end_addr = (start_addr - 1U) + pdata->region[n].len;

			if (risaf_configure_region(idx, id, cfg, cid_cfg,
						   start_addr, end_addr) < 0) {
				panic();
			}
		}
	}
}

static int risaf_get_dt_node(struct dt_node_info *info, int offset)
{
	return dt_get_node(info, offset, DT_RISAF_COMPAT);
}

static int risaf_get_base_from_fdt(void)
{
	struct dt_node_info risaf_info;
	int node = -1;
	void *fdt;

	if (fdt_get_address(&fdt) == 0) {
		return -ENOENT;
	}

	for (node = risaf_get_dt_node(&risaf_info, node); node >= 0;
	     node = risaf_get_dt_node(&risaf_info, node)) {
		int idx;

		idx = stm32_risaf_get_instance(risaf_info.base);
		if (idx < 0) {
			continue;
		}

		stm32mp2_risaf.base[idx] = risaf_info.base;
	}

	return 0;
}

static int risaf_parse_fdt(void)
{
	struct stm32mp2_risaf_platdata *pdata = &stm32mp2_risaf;
	struct dt_node_info risaf_info;
	int node = -1;
	void *fdt;

	if (fdt_get_address(&fdt) == 0) {
		return -ENOENT;
	}

	for (node = risaf_get_dt_node(&risaf_info, node); node >= 0;
	     node = risaf_get_dt_node(&risaf_info, node)) {
		int idx;
		int nregions;
		int inst_maxregions;
		int i;
		int len = 0;
		const struct risaf_dt_id_attr *conf_list;

		idx = stm32_risaf_get_instance(risaf_info.base);
		if (idx < 0) {
			continue;
		}

		conf_list = (const struct risaf_dt_id_attr *)fdt_getprop(fdt, node, "st,protreg",
									 &len);
		if (conf_list == NULL) {
			INFO("RISAF%d: No configuration in DT, use default\n", idx + 1);
			continue;
		}

		if ((len % RISAF_REGION_DT_SIZE) != 0) {
			ERROR("RISAF%d: Wrongly formatted DT configuration\n", idx + 1);
			return -EINVAL;
		}

		nregions = len / RISAF_REGION_DT_SIZE;

		inst_maxregions = stm32_risaf_get_max_region(idx);
		if (inst_maxregions <= 0) {
			continue;
		}

		if ((nregions > inst_maxregions) ||
		    ((pdata->nregions + nregions) > RISAF_MAX_REGION)) {
			ERROR("RISAF%d: Too many entries in DT configuration\n", idx + 1);
			return -EINVAL;
		}

		for (i = 0; i < nregions; i++) {
			pdata->region[pdata->nregions + i].instance = idx;
			pdata->region[pdata->nregions + i].cfg =
				fdt32_to_cpu(conf_list->id_attr[i * RISAF_REGION_DT_PARAM]);
			pdata->region[pdata->nregions + i].addr =
				fdt32_to_cpu(conf_list->id_attr[(i * RISAF_REGION_DT_PARAM) + 1U]);
			pdata->region[pdata->nregions + i].len =
				fdt32_to_cpu(conf_list->id_attr[(i * RISAF_REGION_DT_PARAM) + 2U]);
		}

		pdata->nregions += nregions;
	}

	return 0;
}

static uint32_t risaf_get_instance_mask(void)
{
	uint32_t inst_mask = 0U;
	int idx;

	/* Get all instances present in stm32mp2_risaf */
	for (idx = 0; idx < RISAF_MAX_INSTANCE; idx++) {
		if (stm32mp2_risaf.base[idx] != 0U) {
			inst_mask |= BIT(idx);
		}
	}

	return inst_mask;
}

static uintptr_t risaf_base[RISAF_MAX_INSTANCE];
static struct stm32mp2_risaf_region risaf_region[RISAF_MAX_REGION];

static int risaf_get_platdata(struct stm32mp2_risaf_platdata *pdata)
{
	pdata->base = risaf_base;
	pdata->region = risaf_region;

	return 0;
}

/*
 * @brief  Write the MCE master key for a given instance.
 * @param  instance: RISAF instance ID.
 *         mkey: Pointer to the master key buffer.
 * @retval 0 if OK, negative value else.
 */
int stm32mp2_risaf_write_master_key(int instance, uint8_t *mkey)
{
	uint64_t timeout_ref;
	uint32_t i;
	uintptr_t base = stm32mp2_risaf.base[instance];

	if (base == 0U) {
		return -EINVAL;
	}

	if (mkey == NULL) {
		return -EINVAL;
	}

	for (i = 0U; i < RISAF_KEY_SIZE_IN_BYTES; i += sizeof(uint32_t)) {
		uint32_t key_val = 0U;

		memcpy(&key_val, mkey + i, sizeof(uint32_t));

		mmio_write_32(base + _RISAF_KEYR + i, key_val);
	}

	timeout_ref = timeout_init_us(RISAF_TIMEOUT_1MS_IN_US);

	while (((mmio_read_32(base + _RISAF_SR) & _RISAF_SR_KEYVALID) != _RISAF_SR_KEYVALID) ||
	       ((mmio_read_32(base + _RISAF_SR) & _RISAF_SR_KEYRDY) != _RISAF_SR_KEYRDY)) {
		if (timeout_elapsed(timeout_ref)) {
			return -EIO;
		}
	}

	return 0;
}

/*
 * @brief  Lock the RISAF IP registers for a given instance.
 * @param  instance: RISAF instance ID.
 * @retval 0 if OK, negative value else.
 */
int stm32mp2_risaf_lock(int instance)
{
	uintptr_t base = stm32mp2_risaf.base[instance];

	if (base == 0U) {
		return -EINVAL;
	}

	mmio_setbits_32(base + _RISAF_CR, _RISAF_CR_GLOCK);

	return 0;
}

/*
 * @brief  Get the RISAF lock state for a given instance.
 * @param  instance: RISAF instance ID.
 *         state: lock state, true if locked, false else.
 * @retval 0 if OK, negative value else.
 */
int stm32mp2_risaf_is_locked(int instance, bool *state)
{
	uintptr_t base = stm32mp2_risaf.base[instance];

	if (base == 0U) {
		return -EINVAL;
	}

	*state = (mmio_read_32(base + _RISAF_CR) & _RISAF_CR_GLOCK) == _RISAF_CR_GLOCK;

	return 0;
}

int stm32mp2_risaf_init(void)
{
	int err;

	err = risaf_get_platdata(&stm32mp2_risaf);
	if (err != 0) {
		return err;
	}

	err = risaf_get_base_from_fdt();
	if (err != 0) {
		return err;
	}

	stm32_risaf_preconf_process(risaf_get_instance_mask());

	return err;
}

static int fconf_populate_risaf(uintptr_t config)
{
	int err;

	err = risaf_parse_fdt();
	if (err != 0) {
		return err;
	}

	risaf_conf_protreg();

	stm32_risaf_postconf_process(risaf_get_instance_mask());

	return err;
}

FCONF_REGISTER_POPULATOR(FW_CONFIG, risaf_config, fconf_populate_risaf);
