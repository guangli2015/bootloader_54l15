/*
 * Copyright (c) 2025 Nordic Semiconductor ASA
 *
 * SPDX-License-Identifier: LicenseRef-Nordic-5-Clause
 */

#include <nrf_sdh.h>
#include <nrf_sdh_ble.h>
#include <ble_adv.h>
#include <ble_gap.h>
#include <nrf_soc.h>


#include <board-config.h>

#include "prj_config.h"
#include "log.h"
#include "ble_dfu.h"
#define LOG_DBG
#define LOG_ERR
#define LOG_WRN
#define __ASSERT

#define BLE_UUID_DEVICE_INFORMATION_SERVICE                      0x180A     /**< Device Information service UUID. */
BLE_ADV_DEF(ble_adv); /* BLE advertising instance */

/* Device information service is single-instance */

static uint16_t conn_handle = BLE_CONN_HANDLE_INVALID;

static void on_ble_evt(const ble_evt_t *evt, void *ctx)
{
	int err;

	switch (evt->header.evt_id) {
	case BLE_GAP_EVT_CONNECTED:
		LOG_INF("Peer connected");
		conn_handle = evt->evt.gap_evt.conn_handle;
		err = sd_ble_gatts_sys_attr_set(conn_handle, NULL, 0, 0);
		if (err) {
			LOG_ERR("Failed to set system attributes, nrf_error %#x", err);
		}
		break;

	case BLE_GAP_EVT_DISCONNECTED:
		LOG_INF("Peer disconnected");
		if (conn_handle == evt->evt.gap_evt.conn_handle) {
			conn_handle = BLE_CONN_HANDLE_INVALID;
		}
		break;

	case BLE_GAP_EVT_AUTH_STATUS:
		LOG_INF("Authentication status: %#x",
		       evt->evt.gap_evt.params.auth_status.auth_status);
		break;

	case BLE_GAP_EVT_SEC_PARAMS_REQUEST:
		/* Pairing not supported */
		err = sd_ble_gap_sec_params_reply(evt->evt.gap_evt.conn_handle,
						  BLE_GAP_SEC_STATUS_PAIRING_NOT_SUPP, NULL, NULL);
		if (err) {
			LOG_ERR("Failed to reply with Security params, nrf_error %#x", err);
		}
		break;

	case BLE_GATTS_EVT_SYS_ATTR_MISSING:
		LOG_INF("BLE_GATTS_EVT_SYS_ATTR_MISSING");
		/* No system attributes have been stored */
		err = sd_ble_gatts_sys_attr_set(conn_handle, NULL, 0, 0);
		if (err) {
			LOG_ERR("Failed to set system attributes, nrf_error %#x", err);
		}
		break;
	}
}
NRF_SDH_BLE_OBSERVER(sdh_ble, on_ble_evt, NULL, 0);

static void ble_adv_evt_handler(struct ble_adv *adv, const struct ble_adv_evt *adv_evt)
{
	switch (adv_evt->evt_type) {
	case BLE_ADV_EVT_ERROR:
		LOG_ERR("Advertising error %d", adv_evt->error.reason);
		break;
	default:
		break;
	}
}






/**@brief Function for starting advertising. */
static void advertising_start(void * p_erase_bonds)
{
    bool erase_bonds = *(bool*)p_erase_bonds;
    int err;
   
        err = ble_adv_start(&ble_adv, BLE_ADV_MODE_FAST);
	if (err) {
		LOG_ERR("Failed to start advertising, err %d", err);
		
	}

	LOG_INF("Advertising as %s", CONFIG_BLE_ADV_NAME);
      
    
}
static ble_uuid_t m_adv_uuids[] = {{BLE_UUID_DEVICE_INFORMATION_SERVICE, BLE_UUID_TYPE_BLE}};
static void ble_dfu_evt_handler(ble_dfu_buttonless_evt_type_t event)
{
    switch (event)
    {
        case BLE_DFU_EVT_BOOTLOADER_ENTER_PREPARE:
        {
            LOG_INF("Device is preparing to enter bootloader mode.\r\n");

          
            break;
        }

        case BLE_DFU_EVT_BOOTLOADER_ENTER:
            // YOUR_JOB: Write app-specific unwritten data to FLASH, control finalization of this
            //           by delaying reset by reporting false in app_shutdown_handler
            LOG_INF("Device will enter bootloader mode.\r\n");
            break;

        case BLE_DFU_EVT_BOOTLOADER_ENTER_FAILED:
            LOG_INF("Request to enter bootloader mode failed asynchroneously.");
            // YOUR_JOB: Take corrective measures to resolve the issue
            //           like calling APP_ERROR_CHECK to reset the device.
            break;

        case BLE_DFU_EVT_RESPONSE_SEND_ERROR:
            LOG_INF("Request to send a response to client failed.");
            // YOUR_JOB: Take corrective measures to resolve the issue
            //           like calling APP_ERROR_CHECK to reset the device.
            
            break;

        default:
            LOG_INF("Unknown event from ble_dfu_buttonless.");
            break;
    }
}
int ble_buttonless_dfu_sample(void)
{
	int err;
	struct ble_adv_config ble_adv_config = {
		.conn_cfg_tag = CONFIG_NRF_SDH_BLE_CONN_TAG,
		.evt_handler = ble_adv_evt_handler,
		.adv_data = {
			.name_type = BLE_ADV_DATA_FULL_NAME,
			.flags = BLE_GAP_ADV_FLAGS_LE_ONLY_GENERAL_DISC_MODE,
		},
	};


	LOG_INF("BLE buttonless dfu sample started\r\n");

	err = nrf_sdh_enable_request();
	if (err) {
		LOG_ERR("Failed to enable SoftDevice, err %d", err);
		goto idle;
	}

	LOG_INF("SoftDevice enabled\r\n");

	err = nrf_sdh_ble_enable(CONFIG_NRF_SDH_BLE_CONN_TAG);
	if (err) {
		LOG_ERR("Failed to enable BLE, err %d", err);
		goto idle;
	}

	LOG_INF("Bluetooth enabled\r\n");

       ble_dfu_buttonless_init_t dfus_init = {0};
        dfus_init.evt_handler = ble_dfu_evt_handler;

        err = ble_dfu_buttonless_init(&dfus_init);
	//err = ble_lbs_init(&ble_lbs, &lbs_cfg);
	if (err) {
		LOG_ERR("Failed to setup LED Button Service, err %d", err);
		goto idle;
	}



	/* Adding the LBS UUID to the scan response data. */
	ble_uuid_t adv_uuid_list[] = {
		{ .uuid = BLE_UUID_DEVICE_INFORMATION_SERVICE, .type = BLE_UUID_TYPE_BLE },
	};
	ble_adv_config.sr_data.uuid_lists.complete.uuid = &adv_uuid_list[0];
	ble_adv_config.sr_data.uuid_lists.complete.len = ARRAY_SIZE(adv_uuid_list);

	LOG_INF("Services initialized\r\n");

	err = ble_adv_init(&ble_adv, &ble_adv_config);
	if (err) {
		LOG_ERR("Failed to initialize BLE advertising, err %d", err);
		goto idle;
	}
        LOG_INF("adv initialized\r\n");
#if 1
	err = ble_adv_start(&ble_adv, BLE_ADV_MODE_FAST);
	if (err) {
		LOG_ERR("Failed to start advertising, err %d", err);
		goto idle;
	}

	LOG_INF("Advertising as %s", CONFIG_BLE_ADV_NAME);
#endif
#if 0
        bool erase_bonds;
        erase_bonds = false;
        // Create a FreeRTOS task for the BLE stack.
        // The task will run advertising_start() before entering its loop.
        nrf_sdh_freertos_init(advertising_start, &erase_bonds);
#endif
idle:
      while (true) {
		
		/* Wait for an event. */
		__WFE();

		/* Clear Event Register */
		__SEV();
		__WFE();
	}
	return 0;
}
