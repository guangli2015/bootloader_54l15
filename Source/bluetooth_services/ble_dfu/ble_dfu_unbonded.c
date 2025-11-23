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

#include <stdint.h>
#include <stdbool.h>
#include <stddef.h>

#include "nrf_error.h"
#include "ble_dfu.h"
#include "log.h"
#include "nrf_sdh_soc.h"

#define LOG_DEBUG
#define VERIFY_FALSE(statement, err_code)   \
do                                          \
{                                           \
    if ((statement))                        \
    {                                       \
        return err_code;                    \
    }                                       \
} while (0)
#define VERIFY_PARAM_NOT_NULL(param) VERIFY_FALSE(((param) == NULL), NRF_ERROR_NULL)
#define VERIFY_FALSE_VOID(statement) VERIFY_FALSE((statement), )
#define VERIFY_PARAM_NOT_NULL_VOID(param) VERIFY_FALSE_VOID(((param) == NULL))
#define VERIFY_SUCCESS(statement)                       \
do                                                      \
{                                                       \
    uint32_t _err_code = (uint32_t) (statement);        \
    if (_err_code != NRF_SUCCESS)                       \
    {                                                   \
        return _err_code;                               \
    }                                                   \
} while(0)

#define NRF_DFU_ADV_NAME_MAX_LENGTH     (20)


void ble_dfu_buttonless_on_sys_evt(uint32_t, void * );
uint32_t nrf_dfu_svci_vector_table_set(void);
uint32_t nrf_dfu_svci_vector_table_unset(void);



// Register SoC observer for the Buttonless Secure DFU service
NRF_SDH_SOC_OBSERVER(m_dfu_buttonless_soc_obs, ble_dfu_buttonless_on_sys_evt, NULL,BLE_DFU_SOC_OBSERVER_PRIO);

ble_dfu_buttonless_t      * mp_dfu = NULL;



/**@brief Function for entering the bootloader.
 */
static uint32_t enter_bootloader()
{
    uint32_t err_code;


    // Set the flag indicating that we expect DFU mode.
    // This will be handled on acknowledgement of the characteristic indication.
    mp_dfu->is_waiting_for_reset = true;

    err_code = ble_dfu_buttonless_resp_send(DFU_OP_ENTER_BOOTLOADER, DFU_RSP_SUCCESS);
    if (err_code != NRF_SUCCESS)
    {
        mp_dfu->is_waiting_for_reset = false;
    }

    return err_code;
}


uint32_t ble_dfu_buttonless_backend_init(ble_dfu_buttonless_t * p_dfu)
{
    VERIFY_PARAM_NOT_NULL(p_dfu);

    mp_dfu = p_dfu;

    return NRF_SUCCESS;
}





void ble_dfu_buttonless_on_sys_evt(uint32_t sys_evt, void * p_context)
{
    uint32_t err_code;
#if 0
    if (!nrf_dfu_set_adv_name_is_initialized())
    {
        return;
    }

    err_code = nrf_dfu_set_adv_name_on_sys_evt(sys_evt);
    if (err_code == NRF_ERROR_INVALID_STATE)
    {
        // The system event is not from an operation started by buttonless DFU.
        // No action is taken, and nothing is reported.
    }
    else if (err_code == NRF_SUCCESS)
    {
        // The async operation is finished.
        // Set the flag indicating that we are waiting for indication response
        // to activate the reset.
        mp_dfu->is_waiting_for_svci = false;

        // Report back the positive response
        err_code = ble_dfu_buttonless_resp_send(DFU_OP_SET_ADV_NAME, DFU_RSP_SUCCESS);
        if (err_code != NRF_SUCCESS)
        {
            mp_dfu->evt_handler(BLE_DFU_EVT_RESPONSE_SEND_ERROR);
        }
    }
    else
    {
        // Invalid error code reported back.
        mp_dfu->is_waiting_for_svci = false;

        err_code = ble_dfu_buttonless_resp_send(DFU_OP_SET_ADV_NAME, DFU_RSP_BUSY);
        if (err_code != NRF_SUCCESS)
        {
            mp_dfu->evt_handler(BLE_DFU_EVT_RESPONSE_SEND_ERROR);
        }

        // Report the failure to enter DFU mode
        mp_dfu->evt_handler(BLE_DFU_EVT_BOOTLOADER_ENTER_FAILED);
    }
#endif
}


uint32_t ble_dfu_buttonless_char_add(ble_dfu_buttonless_t * p_dfu)
{
#if 0
    ble_add_char_params_t add_char_params;

    memset(&add_char_params, 0, sizeof(add_char_params));
    add_char_params.uuid                = BLE_DFU_BUTTONLESS_CHAR_UUID;
    add_char_params.uuid_type           = p_dfu->uuid_type;
    add_char_params.char_props.indicate = 1;
    add_char_params.char_props.write    = 1;
    add_char_params.is_defered_write    = true;
    add_char_params.is_var_len          = true;
    add_char_params.max_len             = BLE_GATT_ATT_MTU_DEFAULT;

    add_char_params.cccd_write_access = SEC_OPEN;
    add_char_params.write_access      = SEC_OPEN;
    add_char_params.read_access       = SEC_OPEN;

    return characteristic_add(p_dfu->service_handle, &add_char_params, &p_dfu->control_point_char);
#endif
    /* Add Button characteristic. */
	ble_uuid_t char_uuid = {
		.type =  p_dfu->uuid_type,
		.uuid = BLE_DFU_BUTTONLESS_CHAR_UUID,
	};
	ble_gatts_attr_md_t cccd_md = {
		.vloc = BLE_GATTS_VLOC_STACK,
	};
	ble_gatts_char_md_t char_md = {
		.char_props = {
			.write = true,
			.indicate = true,
		},
		.p_cccd_md = &cccd_md,
	};
	ble_gatts_attr_md_t attr_md = {
		.vloc = BLE_GATTS_VLOC_STACK,
                .wr_auth = true,
                .vlen = true,
	};
	ble_gatts_attr_t attr_char_value = {
		.p_uuid = &char_uuid,
		.p_attr_md = &attr_md,
		//.p_value = &initial_value,
		//.init_len = sizeof(uint8_t),
		.max_len = 200,
	};
	BLE_GAP_CONN_SEC_MODE_SET_OPEN(&attr_md.read_perm);
        BLE_GAP_CONN_SEC_MODE_SET_OPEN(&attr_md.write_perm);
	BLE_GAP_CONN_SEC_MODE_SET_OPEN(&cccd_md.read_perm);
	BLE_GAP_CONN_SEC_MODE_SET_OPEN(&cccd_md.write_perm);

	return sd_ble_gatts_characteristic_add(p_dfu->service_handle, &char_md, &attr_char_value,
					      &p_dfu->control_point_char);
	

}


void ble_dfu_buttonless_on_ctrl_pt_write(ble_gatts_evt_write_t const * p_evt_write)
{
    uint32_t err_code;
    ble_dfu_buttonless_rsp_code_t rsp_code = DFU_RSP_OPERATION_FAILED;

    // Start executing the control point write operation
    /*lint -e415 -e416 -save "Out of bounds access"*/
    switch (p_evt_write->data[0])
    {
        case DFU_OP_ENTER_BOOTLOADER:
            err_code = enter_bootloader();
            if (err_code == NRF_SUCCESS)
            {
                rsp_code = DFU_RSP_SUCCESS;
            }
            else if (err_code == NRF_ERROR_BUSY)
            {
                rsp_code = DFU_RSP_BUSY;
            }
            break;

        default:
            rsp_code = DFU_RSP_OP_CODE_NOT_SUPPORTED;
            break;
    }
    /*lint -restore*/


    // Report back in case of error
    if (rsp_code != DFU_RSP_SUCCESS)
    {
        err_code = ble_dfu_buttonless_resp_send((ble_dfu_buttonless_op_code_t)p_evt_write->data[0], rsp_code);
        if (err_code != NRF_SUCCESS)
        {
            mp_dfu->evt_handler(BLE_DFU_EVT_RESPONSE_SEND_ERROR);

        }
        // Report the error to the main application
        mp_dfu->evt_handler(BLE_DFU_EVT_BOOTLOADER_ENTER_FAILED);
    }
}

uint32_t ble_dfu_buttonless_bootloader_start_prepare(void)
{
    uint32_t err_code;

    // Indicate to main app that DFU mode is starting.
    mp_dfu->evt_handler(BLE_DFU_EVT_BOOTLOADER_ENTER_PREPARE);

    err_code = ble_dfu_buttonless_bootloader_start_finalize();
    return err_code;
}


