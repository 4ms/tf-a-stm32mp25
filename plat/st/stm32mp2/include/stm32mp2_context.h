/*
 * Copyright (c) 2023, ARM Limited and Contributors. All rights reserved.
 *
 * SPDX-License-Identifier: BSD-3-Clause
 */

#ifndef STM32MP2_CONTEXT_H
#define STM32MP2_CONTEXT_H

#include <stdint.h>

void stm32mp_pm_save_enc_mkey_in_context(uint8_t *data);
void stm32mp_pm_get_enc_mkey_from_context(uint8_t *data);

#endif /* STM32MP2_CONTEXT_H */
