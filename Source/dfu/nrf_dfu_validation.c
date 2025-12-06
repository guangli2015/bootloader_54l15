/**
 * Copyright (c) 2017 - 2021, Nordic Semiconductor ASA
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
#include <stdbool.h>
#include <string.h>
#include "nrf_dfu_types.h"
#include "nrf_dfu_settings.h"
#include "nrf_dfu_utils.h"

#include "nrf_dfu_validation.h"
#include <stdlib.h>
#include "log.h"

#define NRF_LOG_DEBUG LOG_INF
#define NRF_LOG_ERROR LOG_INF


#define EXT_ERR(err) (nrf_dfu_result_t)((uint32_t)NRF_DFU_RES_CODE_EXT_ERROR + (uint32_t)err)

/* Whether a complete init command has been received and prevalidated, but the firmware
 * is not yet fully transferred. This value will also be correct after reset.
 */
static bool               m_valid_init_cmd_present = false;

static uint8_t*           m_init_packet_data_ptr   = 0;
static uint32_t           m_init_packet_data_len   = 0;






/** @brief Value length structure holding the public key.
 *
 * @details The pk value pointed to is the public key present in dfu_public_key.c
 */


/** @brief Whether nrf_crypto and local keys have been initialized.
 */
static bool                                         m_crypto_initialized = false;

/** @brief Flag used by parser code to indicate that the init command has been found to be invalid.
 */
static bool                                         m_init_packet_valid = false;






uint32_t crc32_compute(uint8_t const * p_data, uint32_t size, uint32_t const * p_crc)
{
    uint32_t crc;

    crc = (p_crc == NULL) ? 0xFFFFFFFF : ~(*p_crc);
    for (uint32_t i = 0; i < size; i++)
    {
        crc = crc ^ p_data[i];
        for (uint32_t j = 8; j > 0; j--)
        {
            crc = (crc >> 1) ^ (0xEDB88320U & ((crc & 1) ? 0xFFFFFFFF : 0));
        }
    }
    return ~crc;
}

void nrf_dfu_validation_init(void)
{
    // If the command is stored to flash, init command was valid.
    if ((s_dfu_settings.progress.command_size != 0))
    {
        m_valid_init_cmd_present = true;
    }
    else
    {
        m_valid_init_cmd_present = false;
    }
}


nrf_dfu_result_t nrf_dfu_validation_init_cmd_create(uint32_t size)
{
    nrf_dfu_result_t ret_val = NRF_DFU_RES_CODE_SUCCESS;
    if (size == 0)
    {
        ret_val = NRF_DFU_RES_CODE_INVALID_PARAMETER;
    }
    else if (size > INIT_COMMAND_MAX_SIZE)
    {
        ret_val = NRF_DFU_RES_CODE_INSUFFICIENT_RESOURCES;
    }
    else
    {
        // Set DFU to uninitialized.
        m_valid_init_cmd_present = false;

        // Reset all progress.
        nrf_dfu_settings_progress_reset();

        // Set the init command size.
        s_dfu_settings.progress.command_size = size;
    }
    return ret_val;
}


nrf_dfu_result_t nrf_dfu_validation_init_cmd_append(uint8_t const * p_data, uint32_t length)
{
    nrf_dfu_result_t ret_val = NRF_DFU_RES_CODE_SUCCESS;
    if ((length + s_dfu_settings.progress.command_offset) > s_dfu_settings.progress.command_size)
    {
        LOG_INF("Init command larger than expected.");
        ret_val = NRF_DFU_RES_CODE_INVALID_PARAMETER;
    }
    else
    {
        // Copy the received data to RAM, update offset and calculate CRC.
        memcpy(&s_dfu_settings.init_command[s_dfu_settings.progress.command_offset],
                p_data,
                length);

        s_dfu_settings.progress.command_offset += length;
        s_dfu_settings.progress.command_crc = crc32_compute(p_data,
                                                            length,
                                                            &s_dfu_settings.progress.command_crc);
    }
    return ret_val;
}


void nrf_dfu_validation_init_cmd_status_get(uint32_t * p_offset,
                                            uint32_t * p_crc,
                                            uint32_t * p_max_size)
{
    *p_offset   = s_dfu_settings.progress.command_offset;
    *p_crc      = s_dfu_settings.progress.command_crc;
    *p_max_size = INIT_COMMAND_MAX_SIZE;
}


bool nrf_dfu_validation_init_cmd_present(void)
{
    return m_valid_init_cmd_present;
}













nrf_dfu_result_t nrf_dfu_validation_init_cmd_execute(uint32_t * p_dst_data_addr,
                                                     uint32_t * p_data_len)
{
    nrf_dfu_result_t ret_val = NRF_DFU_RES_CODE_SUCCESS;

    if (s_dfu_settings.progress.command_offset != s_dfu_settings.progress.command_size)
    {
        // The object wasn't the right (requested) size.
        NRF_LOG_ERROR("Execute with faulty offset");
        ret_val = NRF_DFU_RES_CODE_OPERATION_NOT_PERMITTED;
    }
    else if (m_valid_init_cmd_present)
    {
        *p_dst_data_addr = nrf_dfu_bank1_start_addr();
        ret_val          = update_data_size_get(mp_init, p_data_len);
    }
    else if (stored_init_cmd_decode())
    {
        // Will only get here if init command was received since last reset.
        // An init command should not be written to flash until after it's been checked here.
        ret_val = nrf_dfu_validation_prevalidate();

        *p_dst_data_addr = 0;
        *p_data_len      = 0;

        // Get size of binary.
        if (ret_val == NRF_DFU_RES_CODE_SUCCESS)
        {
            ret_val = update_data_size_get(mp_init, p_data_len);
        }

        // Get address where to flash the binary.
        if (ret_val == NRF_DFU_RES_CODE_SUCCESS)
        {
            ret_val = update_data_addr_get(mp_init, *p_data_len, p_dst_data_addr);
        }

        // Set flag validating the init command.
        if (ret_val == NRF_DFU_RES_CODE_SUCCESS)
        {
            m_valid_init_cmd_present = true;
        }
        else
        {
            nrf_dfu_settings_progress_reset();
        }
    }
    else
    {
        NRF_LOG_ERROR("Failed to decode init packet");
        ret_val = NRF_DFU_RES_CODE_INVALID_OBJECT;
    }

    return ret_val;
}

