/*
 * Copyright (c) 2025 Nordic Semiconductor ASA
 *
 * SPDX-License-Identifier: LicenseRef-Nordic-5-Clause
 */

#include <string.h>

#include <nrf_error.h>
#define CONFIG_SOFTDEVICE
#if defined(CONFIG_SOFTDEVICE)
#include <nrf_sdh.h>
#include <nrf_soc.h>
#endif

#include <bm_storage.h>
#include "log.h"
#define LOG_ERR LOG_INF

/* Two disjoint storage partitions to showcase multiple clients of the storage library. */

#define STORAGE0_START 0x0000b000
#define STORAGE0_SIZE 0x146800



/* Write buffer size must be a multiple of the program unit.
 * To support both RRAM (16 bytes) and SoftDevice (4 bytes) backends,
 * that is 16 bytes.
 */
#define BUFFER_BLOCK_SIZE 20

/* Forward declarations. */
static void bm_storage_evt_handler(struct bm_storage_evt *evt);


/* Tracks the number of write operations that are in the process of being executed. */
static volatile int outstanding_writes;

static struct bm_storage storage = {
	.evt_handler = bm_storage_evt_handler,
	.start_addr = STORAGE0_START,
	.end_addr = STORAGE0_START + STORAGE0_SIZE,
};


static void bm_storage_evt_handler(struct bm_storage_evt *evt)
{
	switch (evt->id) {
	case BM_STORAGE_EVT_WRITE_RESULT:
		LOG_INF("Handler : bm_storage_evt: WRITE_RESULT %d, DISPATCH_TYPE %d\r\n",
			evt->result, evt->dispatch_type);
		//outstanding_writes--;
		break;
	case BM_STORAGE_EVT_ERASE_RESULT:
		/* Not used. */
		break;
	default:
		break;
	}
      if (evt->ctx)
    {
        //lint -save -e611 (Suspicious cast)
        ((nrf_dfu_flash_callback_t)(evt->ctx))((void*)evt->src);
        //lint -restore
    }
}




static uint32_t storage_inits(void)
{
	uint32_t err;

	err = bm_storage_init(&storage);
	if (err != NRF_SUCCESS) {
		LOG_ERR("bm_storage_init() failed, err %#x", err);
		return err;
	}

	return NRF_SUCCESS;
}

static uint32_t storage_uninits(void)
{
	uint32_t err;

	err = bm_storage_uninit(&storage);
	if (err != NRF_SUCCESS && err != NRF_ERROR_NOT_SUPPORTED) {
		LOG_ERR("bm_storage_uninit() failed, err %#x", err);
		return err;
	}

	

	return NRF_SUCCESS;
}
static  uint8_t input_a[BUFFER_BLOCK_SIZE] = "Helll";
 static uint8_t input_b[BUFFER_BLOCK_SIZE] = "Wordd!";
static uint32_t storage_writes(void)
{
	uint32_t err;

	LOG_INF("Writing in Partition A, addr: 0x%08X, size: %d\r\n", storage.start_addr,
		sizeof(input_a));

	err = bm_storage_write(&storage, storage.start_addr, input_a, sizeof(input_a), NULL);
	if (err != NRF_SUCCESS) {
		LOG_ERR("bm_storage_write() failed, err %#x", err);
		return err;
	}


	return NRF_SUCCESS;
}

