/*
 * Copyright (c) 2021, STMicroelectronics - All Rights Reserved
 *
 * SPDX-License-Identifier: GPL-2.0+ OR BSD-3-Clause
 */

#include <inttypes.h>

#include <libfdt.h>

#include <platform_def.h>

#include <common/debug.h>
#include <common/fdt_wrappers.h>
#include <drivers/clk.h>
#include <drivers/delay_timer.h>
#include <drivers/spi_mem.h>
#include <drivers/st/stm32_gpio.h>
#include <drivers/st/stm32_ospi.h>
#include <drivers/st/stm32mp_reset.h>
#include <lib/mmio.h>
#include <lib/utils_def.h>

/* Timeout for device interface reset */
#define _TIMEOUT_US_1_MS		1000U

/* OCTOSPI registers */
#define _OSPI_CR		0x00U
#define _OSPI_DCR1		0x08U
#define _OSPI_DCR2		0x0CU
#define _OSPI_SR		0x20U
#define _OSPI_FCR		0x24U
#define _OSPI_DLR		0x40U
#define _OSPI_AR		0x48U
#define _OSPI_DR		0x50U
#define _OSPI_CCR		0x100U
#define _OSPI_TCR		0x108U
#define _OSPI_IR		0x110U
#define _OSPI_ABR		0x120U

/* OCTOSPI control register */
#define _OSPI_CR_EN		BIT(0)
#define _OSPI_CR_ABORT		BIT(1)
#define _OSPI_CR_CSSEL		BIT(24)
#define _OSPI_CR_FMODE		GENMASK_32(29, 28)
#define _OSPI_CR_FMODE_SHIFT	28U
#define _OSPI_CR_FMODE_INDW	0U
#define _OSPI_CR_FMODE_INDR	1U
#define _OSPI_CR_FMODE_MM	3U

/* OCTOSPI device configuration register 1 */
#define _OSPI_DCR1_CKMODE	BIT(0)
#define _OSPI_DCR1_CSHT		GENMASK_32(13, 8)
#define _OSPI_DCR1_CSHT_SHIFT	8U
#define _OSPI_DCR1_DEVSIZE	GENMASK_32(20, 16)

/* OCTOSPI device configuration register 2 */
#define _OSPI_DCR2_PRESCALER	GENMASK_32(7, 0)

/* OCTOSPI status register */
#define _OSPI_SR_TEF		BIT(0)
#define _OSPI_SR_TCF		BIT(1)
#define _OSPI_SR_FTF		BIT(2)
#define _OSPI_SR_SMF		BIT(3)
#define _OSPI_SR_BUSY		BIT(5)

/* OCTOSPI flag clear register */
#define _OSPI_FCR_CTEF		BIT(0)
#define _OSPI_FCR_CTCF		BIT(1)
#define _OSPI_FCR_CSMF		BIT(3)

/* OCTOSPI communication configuration register */
#define _OSPI_CCR_ADMODE_SHIFT	8U
#define _OSPI_CCR_ADSIZE_SHIFT	12U
#define _OSPI_CCR_ABMODE_SHIFT	16U
#define _OSPI_CCR_ABSIZE_SHIFT	20U
#define _OSPI_CCR_DMODE_SHIFT	24U

/* OCTOSPI timing configuration register */
#define _OSPI_TCR_DCYC		GENMASK_32(4, 0)
#define _OSPI_TCR_SSHIFT	BIT(30)

#define _OSPI_MAX_CHIP		2U
#define _OSPI_MAX_RESET		2U

#define _OSPI_FIFO_TIMEOUT_US	30U
#define _OSPI_CMD_TIMEOUT_US	1000U
#define _OSPI_BUSY_TIMEOUT_US	100U
#define _OSPI_ABT_TIMEOUT_US	100U

#define _DT_OSPI_COMPAT		"st,stm32-omi"

#define _FREQ_100MHZ		100000000U

struct stm32_ospi_ctrl {
	uintptr_t reg_base;
	uintptr_t mm_base;
	size_t mm_size;
	unsigned long clock_id;
};

static struct stm32_ospi_ctrl stm32_ospi;

static uintptr_t ospi_base(void)
{
	return stm32_ospi.reg_base;
}

static int stm32_ospi_wait_for_not_busy(void)
{
	uint64_t timeout = timeout_init_us(_OSPI_BUSY_TIMEOUT_US);

	while ((mmio_read_32(ospi_base() + _OSPI_SR) & _OSPI_SR_BUSY) != 0U) {
		if (timeout_elapsed(timeout)) {
			ERROR("%s: busy timeout\n", __func__);
			return -ETIMEDOUT;
		}
	}

	return 0;
}

static int stm32_ospi_wait_cmd(const struct spi_mem_op *op)
{
	int ret = 0;
	uint64_t timeout;

	if (op->data.nbytes != 0U) {
		timeout = timeout_init_us(_OSPI_CMD_TIMEOUT_US);
		while ((mmio_read_32(ospi_base() + _OSPI_SR) &
		       _OSPI_SR_TCF) == 0U) {
			if (timeout_elapsed(timeout)) {
				ret = -ETIMEDOUT;
				ERROR("%s: cmd timeout\n", __func__);
				break;
			}
		}

		if ((ret == 0) && ((mmio_read_32(ospi_base() + _OSPI_SR) &
				   _OSPI_SR_TEF) != 0U)) {
			ERROR("%s: transfer error\n", __func__);
			ret = -EIO;
		}

		/* Clear flags */
		mmio_write_32(ospi_base() + _OSPI_FCR,
			      _OSPI_FCR_CTCF | _OSPI_FCR_CTEF);
	}

	if (ret == 0) {
		ret = stm32_ospi_wait_for_not_busy();
	}

	return ret;
}

static void stm32_ospi_read_fifo(uint8_t *val, uintptr_t addr)
{
	*val = mmio_read_8(addr);
}

static void stm32_ospi_write_fifo(uint8_t *val, uintptr_t addr)
{
	mmio_write_8(addr, *val);
}

static int stm32_ospi_poll(const struct spi_mem_op *op)
{
	void (*fifo)(uint8_t *val, uintptr_t addr);
	uint32_t len;
	uint8_t *buf;

	if (op->data.dir == SPI_MEM_DATA_IN) {
		fifo = stm32_ospi_read_fifo;
	} else {
		fifo = stm32_ospi_write_fifo;
	}

	buf = (uint8_t *)op->data.buf;

	for (len = op->data.nbytes; len != 0U; len--) {
		uint64_t timeout = timeout_init_us(_OSPI_FIFO_TIMEOUT_US);

		while ((mmio_read_32(ospi_base() + _OSPI_SR) &
			_OSPI_SR_FTF) == 0U) {
			if (timeout_elapsed(timeout)) {
				ERROR("%s: fifo timeout\n", __func__);
				return -ETIMEDOUT;
			}
		}

		fifo(buf++, ospi_base() + _OSPI_DR);
	}

	return 0;
}

static int stm32_ospi_mm(const struct spi_mem_op *op)
{
	memcpy(op->data.buf,
	       (void *)(stm32_ospi.mm_base + (size_t)op->addr.val),
	       op->data.nbytes);

	return 0;
}

static int stm32_ospi_tx(const struct spi_mem_op *op, uint8_t fmode)
{
	if (op->data.nbytes == 0U) {
		return 0;
	}

	if (fmode == _OSPI_CR_FMODE_MM) {
		return stm32_ospi_mm(op);
	}

	return stm32_ospi_poll(op);
}

static unsigned int stm32_ospi_get_mode(uint8_t buswidth)
{
	switch (buswidth) {
	case SPI_MEM_BUSWIDTH_8_LINE:
		return 4U;
	case SPI_MEM_BUSWIDTH_4_LINE:
		return 3U;
	default:
		return buswidth;
	}
}

static int stm32_ospi_send(const struct spi_mem_op *op, uint8_t fmode)
{
	uint64_t timeout;
	uint32_t ccr;
	uint32_t tcr;
	int ret;

	VERBOSE("%s: cmd:%x mode:%d.%d.%d.%d addr:%" PRIx64 " len:%x\n",
		__func__, op->cmd.opcode, op->cmd.buswidth, op->addr.buswidth,
		op->dummy.buswidth, op->data.buswidth,
		op->addr.val, op->data.nbytes);

	ret = stm32_ospi_wait_for_not_busy();
	if (ret != 0) {
		return ret;
	}

	if (op->data.nbytes != 0U) {
		mmio_write_32(ospi_base() + _OSPI_DLR, op->data.nbytes - 1U);
	}

	tcr = _OSPI_TCR_SSHIFT;
	if ((op->dummy.buswidth != 0U) && (op->dummy.nbytes != 0U)) {
		tcr |= op->dummy.nbytes * 8U / op->dummy.buswidth;
	}

	mmio_clrsetbits_32(ospi_base() + _OSPI_TCR, _OSPI_TCR_DCYC, tcr);

	mmio_clrsetbits_32(ospi_base() + _OSPI_CR, _OSPI_CR_FMODE,
			   fmode << _OSPI_CR_FMODE_SHIFT);

	ccr = stm32_ospi_get_mode(op->cmd.buswidth);

	if (op->addr.nbytes != 0U) {
		ccr |= (op->addr.nbytes - 1U) << _OSPI_CCR_ADSIZE_SHIFT;
		ccr |= stm32_ospi_get_mode(op->addr.buswidth) <<
		       _OSPI_CCR_ADMODE_SHIFT;
	}

	if (op->data.nbytes != 0U) {
		ccr |= stm32_ospi_get_mode(op->data.buswidth) <<
		       _OSPI_CCR_DMODE_SHIFT;
	}

	mmio_write_32(ospi_base() + _OSPI_CCR, ccr);

	mmio_write_32(ospi_base() + _OSPI_IR, op->cmd.opcode);

	if ((op->addr.nbytes != 0U) && (fmode != _OSPI_CR_FMODE_MM)) {
		mmio_write_32(ospi_base() + _OSPI_AR, op->addr.val);
	}

	ret = stm32_ospi_tx(op, fmode);

	/*
	 * Abort in:
	 * - Error case.
	 * - Memory mapped read: prefetching must be stopped if we read the last
	 *   byte of device (device size - fifo size). If device size is not
	 *   known then prefetching is always stopped.
	 */
	if ((ret != 0) || (fmode == _OSPI_CR_FMODE_MM)) {
		goto abort;
	}

	/* Wait end of TX in indirect mode */
	ret = stm32_ospi_wait_cmd(op);
	if (ret != 0) {
		goto abort;
	}

	return 0;

abort:
	mmio_setbits_32(ospi_base() + _OSPI_CR, _OSPI_CR_ABORT);

	/* Wait clear of abort bit by hardware */
	timeout = timeout_init_us(_OSPI_ABT_TIMEOUT_US);
	while ((mmio_read_32(ospi_base() + _OSPI_CR) & _OSPI_CR_ABORT) != 0U) {
		if (timeout_elapsed(timeout)) {
			ret = -ETIMEDOUT;
			break;
		}
	}

	mmio_write_32(ospi_base() + _OSPI_FCR, _OSPI_FCR_CTCF);

	if (ret != 0) {
		ERROR("%s: exec op error\n", __func__);
	}

	return ret;
}

static int stm32_ospi_exec_op(const struct spi_mem_op *op)
{
	uint8_t fmode = _OSPI_CR_FMODE_INDW;

	if ((op->data.dir == SPI_MEM_DATA_IN) && (op->data.nbytes != 0U)) {
		fmode = _OSPI_CR_FMODE_INDR;
	}

	return stm32_ospi_send(op, fmode);
}

static int stm32_ospi_dirmap_read(const struct spi_mem_op *op)
{
	size_t addr_max;
	uint8_t fmode = _OSPI_CR_FMODE_INDR;

	addr_max = op->addr.val + op->data.nbytes + 1U;
	if ((addr_max < stm32_ospi.mm_size) && (op->addr.buswidth != 0U)) {
		fmode = _OSPI_CR_FMODE_MM;
	}

	return stm32_ospi_send(op, fmode);
}

static int stm32_ospi_claim_bus(unsigned int cs)
{
	uint32_t cr;

	if (cs >= _OSPI_MAX_CHIP) {
		return -ENODEV;
	}

	/* Set chip select and enable the controller */
	cr = _OSPI_CR_EN;
	if (cs == 1U) {
		cr |= _OSPI_CR_CSSEL;
	}

	mmio_clrsetbits_32(ospi_base() + _OSPI_CR, _OSPI_CR_CSSEL, cr);

	return 0;
}

static void stm32_ospi_release_bus(void)
{
	mmio_clrbits_32(ospi_base() + _OSPI_CR, _OSPI_CR_EN);
}

static int stm32_ospi_set_speed(unsigned int hz)
{
	unsigned long ospi_clk = clk_get_rate(stm32_ospi.clock_id);
	uint32_t prescaler = UINT8_MAX;
	uint32_t csht;
	int ret;

	if (ospi_clk == 0U) {
		return -EINVAL;
	}

	if (hz > 0U) {
		prescaler = div_round_up(ospi_clk, hz) - 1U;
		if (prescaler > UINT8_MAX) {
			prescaler = UINT8_MAX;
		}
	}

	csht = div_round_up((5U * ospi_clk) / (prescaler + 1U), _FREQ_100MHZ);
	csht = ((csht - 1U) << _OSPI_DCR1_CSHT_SHIFT) & _OSPI_DCR1_CSHT;

	ret = stm32_ospi_wait_for_not_busy();
	if (ret != 0) {
		return ret;
	}

	mmio_clrsetbits_32(ospi_base() + _OSPI_DCR2, _OSPI_DCR2_PRESCALER,
			   prescaler);

	mmio_clrsetbits_32(ospi_base() + _OSPI_DCR1, _OSPI_DCR1_CSHT, csht);

	VERBOSE("%s: speed=%lu\n", __func__, ospi_clk / (prescaler + 1U));

	return 0;
}

static int stm32_ospi_set_mode(unsigned int mode)
{
	int ret;

	if ((mode & SPI_CS_HIGH) != 0U) {
		return -ENODEV;
	}

	ret = stm32_ospi_wait_for_not_busy();
	if (ret != 0) {
		return ret;
	}

	if (((mode & SPI_CPHA) != 0U) && ((mode & SPI_CPOL) != 0U)) {
		mmio_setbits_32(ospi_base() + _OSPI_DCR1, _OSPI_DCR1_CKMODE);
	} else if (((mode & SPI_CPHA) == 0U) && ((mode & SPI_CPOL) == 0U)) {
		mmio_clrbits_32(ospi_base() + _OSPI_DCR1, _OSPI_DCR1_CKMODE);
	} else {
		return -ENODEV;
	}

#if DEBUG
	VERBOSE("%s: mode=0x%x\n", __func__, mode);

	if ((mode & SPI_RX_OCTAL) != 0U) {
		VERBOSE("rx: octal\n");
	} else if ((mode & SPI_RX_QUAD) != 0U) {
		VERBOSE("rx: quad\n");
	} else if ((mode & SPI_RX_DUAL) != 0U) {
		VERBOSE("rx: dual\n");
	} else {
		VERBOSE("rx: single\n");
	}

	if ((mode & SPI_TX_OCTAL) != 0U) {
		VERBOSE("tx: octal\n");
	} else if ((mode & SPI_TX_QUAD) != 0U) {
		VERBOSE("tx: quad\n");
	} else if ((mode & SPI_TX_DUAL) != 0U) {
		VERBOSE("tx: dual\n");
	} else {
		VERBOSE("tx: single\n");
	}
#endif

	return 0;
}

static const struct spi_bus_ops stm32_ospi_bus_ops = {
	.claim_bus = stm32_ospi_claim_bus,
	.release_bus = stm32_ospi_release_bus,
	.set_speed = stm32_ospi_set_speed,
	.set_mode = stm32_ospi_set_mode,
	.exec_op = stm32_ospi_exec_op,
	.dirmap_read = stm32_ospi_dirmap_read,
};

int stm32_ospi_init(void)
{
	size_t size;
	int ospi_node;
	struct dt_node_info info;
	const fdt32_t *cuint;
	void *fdt = NULL;
	int ret;
	int len;
	uint32_t i;
	unsigned int reset_id;

	if (fdt_get_address(&fdt) == 0) {
		return -FDT_ERR_NOTFOUND;
	}

	ospi_node = dt_get_node(&info, -1, _DT_OSPI_COMPAT);
	if (ospi_node < 0) {
		ERROR("No OSPI ctrl found\n");
		return -FDT_ERR_NOTFOUND;
	}

	if (info.status == DT_DISABLED) {
		return -FDT_ERR_NOTFOUND;
	}

	if (info.clock < 0) {
		return -FDT_ERR_BADVALUE;
	}

	ret = fdt_get_reg_props_by_name(fdt, ospi_node, "ospi",
					&stm32_ospi.reg_base, &size);
	if (ret != 0) {
		return ret;
	}

	ret = fdt_get_reg_props_by_name(fdt, ospi_node, "ospi_mm",
					&stm32_ospi.mm_base,
					&stm32_ospi.mm_size);
	if (ret != 0) {
		return ret;
	}

	if (dt_set_pinctrl_config(ospi_node) != 0) {
		return -FDT_ERR_BADVALUE;
	}

	stm32_ospi.clock_id = (unsigned long)info.clock;

	clk_enable(stm32_ospi.clock_id);

	cuint = fdt_getprop(fdt, ospi_node, "resets", &len);
	cuint++;

	/* Reset: array of <phandle, reset_id> */
	for (i = 0U; i < ((uint32_t)len / (sizeof(uint32_t) * _OSPI_MAX_RESET));
	     i++, cuint += _OSPI_MAX_RESET) {
		reset_id = (unsigned int)fdt32_to_cpu(*cuint);

		ret = stm32mp_reset_assert(reset_id, _TIMEOUT_US_1_MS);
		if (ret != 0) {
			panic();
		}

		ret = stm32mp_reset_deassert(reset_id, _TIMEOUT_US_1_MS);
		if (ret != 0) {
			panic();
		}
	}

	mmio_write_32(ospi_base() + _OSPI_DCR1, _OSPI_DCR1_DEVSIZE);

	return spi_mem_init_slave(fdt, ospi_node, &stm32_ospi_bus_ops);
};
