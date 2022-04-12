/*
 * Copyright (c) 2017-2022, STMicroelectronics - All Rights Reserved
 *
 * SPDX-License-Identifier: BSD-3-Clause
 */

#ifndef BSEC_H
#define BSEC_H

#include <stdbool.h>
#include <stdint.h>

#include <lib/utils_def.h>

/*
 * IP configuration
 */
#define BSEC_OTP_MASK			GENMASK(4, 0)
#define BSEC_OTP_BANK_SHIFT		5
#define BSEC_TIMEOUT_VALUE		0xFFFF

/*
 * Return status
 */
#define BSEC_OK				0U
#define BSEC_ERROR			0xFFFFFFFFU
#define BSEC_DISTURBED			0xFFFFFFFEU
#define BSEC_INVALID_PARAM		0xFFFFFFFCU
#define BSEC_PROG_FAIL			0xFFFFFFFBU
#define BSEC_LOCK_FAIL			0xFFFFFFFAU
#define BSEC_TIMEOUT			0xFFFFFFF9U
#define BSEC_RETRY			0xFFFFFFF8U
#define BSEC_NOT_SUPPORTED		0xFFFFFFF7U
#define BSEC_WRITE_LOCKED		0xFFFFFFF6U

/*
 * OTP MODE
 */
#define BSEC_MODE_OPEN1			0x00U
#define BSEC_MODE_SECURED		0x01U
#define BSEC_MODE_OPEN2			0x02U
#define BSEC_MODE_INVALID		0x04U

/*
 * OTP Lock services definition.
 * Value must corresponding to the bit number in the register.
 * Special case: (bit number << 1) for BSEC3.
 */
#define BSEC_LOCK_UPPER_OTP		0x00
#define BSEC_LOCK_GWLOCK		0x01
#define BSEC_LOCK_DEBUG			0x02
#define BSEC_LOCK_PROGRAM		0x03
#define BSEC_LOCK_KVLOCK		0x04

uint32_t bsec_probe(void);
uint32_t bsec_get_base(void);

uint32_t bsec_shadow_register(uint32_t otp);
uint32_t bsec_read_otp(uint32_t *val, uint32_t otp);
uint32_t bsec_write_otp(uint32_t val, uint32_t otp);
uint32_t bsec_program_otp(uint32_t val, uint32_t otp);
uint32_t bsec_permanent_lock_otp(uint32_t otp);

uint32_t bsec_read_debug_conf(void);

void bsec_write_scratch(uint32_t val);
uint32_t bsec_read_scratch(void);

uint32_t bsec_get_status(void);
uint32_t bsec_get_hw_conf(void);
uint32_t bsec_get_version(void);
uint32_t bsec_get_id(void);
uint32_t bsec_get_magic_id(void);

uint32_t bsec_set_sr_lock(uint32_t otp);
uint32_t bsec_read_sr_lock(uint32_t otp, bool *value);
uint32_t bsec_set_sw_lock(uint32_t otp);
uint32_t bsec_read_sw_lock(uint32_t otp, bool *value);
uint32_t bsec_set_sp_lock(uint32_t otp);
uint32_t bsec_read_sp_lock(uint32_t otp, bool *value);
uint32_t bsec_read_permanent_lock(uint32_t otp, bool *value);
uint32_t bsec_otp_lock(uint32_t service);
bool bsec_mode_is_closed_device(void);
uint32_t bsec_shadow_read_otp(uint32_t *otp_value, uint32_t word);
uint32_t bsec_check_nsec_access_rights(uint32_t otp);

#endif /* BSEC_H */
