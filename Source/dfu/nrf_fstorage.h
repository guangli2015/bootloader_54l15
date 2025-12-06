/**
 * Copyright (c) 2016 - 2021, Nordic Semiconductor ASA
 *
 * All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without modification,
 * are permitted provided that the following conditions are met:
 *
 * 1. Redistributions of source code must retain the above copyright notice, this
 *    list of conditions and the following disclaimer.
 *
 * 2. Redistributions in binary form, except as embedded into a Nordic
 *    Semiconductor ASA integrated circuit in a product or a software update for
 *    such product, must reproduce the above copyright notice, this list of
 *    conditions and the following disclaimer in the documentation and/or other
 *    materials provided with the distribution.
 *
 * 3. Neither the name of Nordic Semiconductor ASA nor the names of its
 *    contributors may be used to endorse or promote products derived from this
 *    software without specific prior written permission.
 *
 * 4. This software, with or without modification, must only be used with a
 *    Nordic Semiconductor ASA integrated circuit.
 *
 * 5. Any software provided in binary form under this license must not be reverse
 *    engineered, decompiled, modified and/or disassembled.
 *
 * THIS SOFTWARE IS PROVIDED BY NORDIC SEMICONDUCTOR ASA "AS IS" AND ANY EXPRESS
 * OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED WARRANTIES
 * OF MERCHANTABILITY, NONINFRINGEMENT, AND FITNESS FOR A PARTICULAR PURPOSE ARE
 * DISCLAIMED. IN NO EVENT SHALL NORDIC SEMICONDUCTOR ASA OR CONTRIBUTORS BE
 * LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
 * CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE
 * GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION)
 * HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
 * LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT
 * OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 *
 */
#ifndef NRF_FSTORAGE_H__
#define NRF_FSTORAGE_H__

/**
 * @file
 *
 * @defgroup nrf_fstorage Flash storage (fstorage)
 * @ingroup app_common
 * @{
 *
 * @brief   Flash abstraction library that provides basic read, write, and erase operations.
 *
 * @details The fstorage library can be implemented in different ways. Two implementations are provided:
 * - The @ref nrf_fstorage_sd implements flash access through the SoftDevice.
 * - The @ref nrf_fstorage_nvmc implements flash access through the non-volatile memory controller.
 *
 * You can select the implementation that should be used independently for each instance of fstorage.
 */

#include <stdint.h>
#include <stdbool.h>


#ifdef __cplusplus
extern "C" {
#endif





/**@brief   Function for querying the status of fstorage.
 *
 * @details An uninitialized instance of fstorage is treated as not busy.
 *
 * @param[in]   p_fs    The fstorage instance. Pass NULL to query all instances.
 *
 * @returns If @p p_fs is @c NULL, this function returns true if any fstorage instance is busy or false otherwise.
 * @returns If @p p_fs is not @c NULL, this function returns true if the fstorage instance is busy or false otherwise.
 */
 bool nrf_fstorage_is_busy(void const * p_fs);

/** @} */


#ifdef __cplusplus
}
#endif

#endif // NRF_FSTORAGE_H__
