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
#define STORAGE0_SIZE 0x1000

#define STORAGE1_START 0x0000c000
#define STORAGE1_SIZE 0x1000

/* Write buffer size must be a multiple of the program unit.
 * To support both RRAM (16 bytes) and SoftDevice (4 bytes) backends,
 * that is 16 bytes.
 */
#define BUFFER_BLOCK_SIZE 16

/* Forward declarations. */
static void bm_storage_evt_handler_a(struct bm_storage_evt *evt);
static void bm_storage_evt_handler_b(struct bm_storage_evt *evt);

/* Tracks the number of write operations that are in the process of being executed. */
static volatile int outstanding_writes;

static struct bm_storage storage_a = {
	.evt_handler = bm_storage_evt_handler_a,
	.start_addr = STORAGE0_START,
	.end_addr = STORAGE0_START + STORAGE0_SIZE,
};

static struct bm_storage storage_b = {
	.evt_handler = bm_storage_evt_handler_b,
	.start_addr = STORAGE1_START,
	.end_addr = STORAGE1_START + STORAGE1_SIZE,
};

static void bm_storage_evt_handler_a(struct bm_storage_evt *evt)
{
	switch (evt->id) {
	case BM_STORAGE_EVT_WRITE_RESULT:
		LOG_INF("Handler A: bm_storage_evt: WRITE_RESULT %d, DISPATCH_TYPE %d\r\n",
			evt->result, evt->dispatch_type);
		outstanding_writes--;
		break;
	case BM_STORAGE_EVT_ERASE_RESULT:
		/* Not used. */
		break;
	default:
		break;
	}
}

static void bm_storage_evt_handler_b(struct bm_storage_evt *evt)
{
	switch (evt->id) {
	case BM_STORAGE_EVT_WRITE_RESULT:
		LOG_INF("Handler B: bm_storage_evt: WRITE_RESULT %d, DISPATCH_TYPE %d\r\n",
			evt->result, evt->dispatch_type);
		outstanding_writes--;
		break;
	case BM_STORAGE_EVT_ERASE_RESULT:
		/* Not used. */
		break;
	default:
		break;
	}
}

static void wait_for_outstanding_writes(void)
{
	LOG_INF("Waiting for writes to complete...\r\n");
	while (outstanding_writes > 0) {
		/* Wait for an event. */
		__WFE();

		/* Clear Event Register */
		__SEV();
		__WFE();
	}
}

static uint32_t storage_inits(void)
{
	uint32_t err;

	err = bm_storage_init(&storage_a);
	if (err != NRF_SUCCESS) {
		LOG_ERR("bm_storage_init() failed, err %#x", err);
		return err;
	}

	err = bm_storage_init(&storage_b);
	if (err != NRF_SUCCESS) {
		LOG_ERR("bm_storage_init() failed, err %#x", err);
		return err;
	}

	return NRF_SUCCESS;
}

static uint32_t storage_uninits(void)
{
	uint32_t err;

	err = bm_storage_uninit(&storage_a);
	if (err != NRF_SUCCESS && err != NRF_ERROR_NOT_SUPPORTED) {
		LOG_ERR("bm_storage_uninit() failed, err %#x", err);
		return err;
	}

	err = bm_storage_uninit(&storage_b);
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
	//char input_a[BUFFER_BLOCK_SIZE] = "Hello";
	//char input_b[BUFFER_BLOCK_SIZE] = "World!";

	/* Prepare writes. */
	outstanding_writes = 2;

	LOG_INF("Writing in Partition A, addr: 0x%08X, size: %d\r\n", storage_a.start_addr,
		sizeof(input_a));

	err = bm_storage_write(&storage_a, storage_a.start_addr, input_a, sizeof(input_a), NULL);
	if (err != NRF_SUCCESS) {
		LOG_ERR("bm_storage_write() failed, err %#x", err);
		return err;
	}

	LOG_INF("Writing in Partition B, addr: 0x%08X, size: %d\r\n", storage_b.start_addr,
		sizeof(input_b));

	err = bm_storage_write(&storage_b, storage_b.start_addr, input_b, sizeof(input_b),
			       NULL);
	if (err != NRF_SUCCESS) {
		LOG_ERR("bm_storage_write() failed, err %#x", err);
		return err;
	}

	return NRF_SUCCESS;
}
static    uint8_t erase[BUFFER_BLOCK_SIZE] = { 0 };
static uint32_t storage_erases(void)
{
	uint32_t err;
	//char erase[BUFFER_BLOCK_SIZE] = { 0 };
memset(erase, 0xaa, sizeof(erase));
	/* Prepare writes. */
	outstanding_writes = 2;

	LOG_INF("Erasing in Partition A, addr: 0x%08X, size: %d\r\n", storage_a.start_addr,
		sizeof(erase));

	err = bm_storage_write(&storage_a, storage_a.start_addr, erase, sizeof(erase), NULL);
	if (err != NRF_SUCCESS) {
		LOG_ERR("bm_storage_write() failed, err %#x", err);
		return err;
	}

	LOG_INF("Erasing in Partition B, addr: 0x%08X, size: %d\r\n", storage_b.start_addr,
		sizeof(erase));

	err = bm_storage_write(&storage_b, storage_b.start_addr, erase, sizeof(erase), NULL);
	if (err != NRF_SUCCESS) {
		LOG_ERR("bm_storage_write() failed, err %#x", err);
		return err;
	}

	return NRF_SUCCESS;
}
static uint8_t output[BUFFER_BLOCK_SIZE] = { 0 };
static uint32_t storage_reads(void)
{
	uint32_t err;
	

	err = bm_storage_read(&storage_a, storage_a.start_addr, output, sizeof(output));
	if (err != NRF_SUCCESS) {
		LOG_ERR("bm_storage_read() failed, err %#x", err);
		return err;
	}

	//LOG_HEXDUMP_INF(output, sizeof(output), "output A:");
        LOG_INF("output A: ");
        for(int i =0;i<BUFFER_BLOCK_SIZE;i++)
        {
          LOG_INF(" %x ",output[i]);
        }
        LOG_INF("\r\n");
	memset(output, 0, sizeof(output));
        
	err = bm_storage_read(&storage_b, storage_b.start_addr, output, sizeof(output));
	if (err != NRF_SUCCESS) {
		LOG_ERR("bm_storage_read() failed, err %#x", err);
		return err;
	}

	//LOG_HEXDUMP_INF(output, sizeof(output), "output B:");
        LOG_INF("outputB: ");
        for(int i =0;i<BUFFER_BLOCK_SIZE;i++)
        {
          LOG_INF(" %x ",output[i]);
        }
         LOG_INF("\r\n");
	return NRF_SUCCESS;
}

int bm_storage_test(void)
{
	uint32_t err;

	LOG_INF("Storage sample started\r\n");

#if defined(CONFIG_SOFTDEVICE)
	err = nrf_sdh_enable_request();
	if (err) {
		LOG_ERR("Failed to enable SoftDevice, err %#x", err);
		goto idle;
	}
#endif

	err = storage_inits();
	if (err) {
		goto idle;
	}

	LOG_INF("Reading persisted data\r\n");

	err = storage_reads();
	if (err) {
		goto idle;
	}

	err = storage_erases();
	if (err) {
		goto idle;
	}

	wait_for_outstanding_writes();

	err = storage_reads();
	if (err) {
		goto idle;
	}

#if 0
	/* When targeting SoftDevice, the storage backend will behave synchronously or
	 * asynchronously if the SoftDevice is respectively disabled or enabled, at runtime.
	 * Disable the SoftDevice here to showcase this dynamic functionality.
	 */
	err = nrf_sdh_disable_request();
	if (err) {
		LOG_ERR("Failed to disable SoftDevice, err %#x", err);
		goto idle;
	}
#endif
#if 1
	err = storage_writes();
	if (err) {
		goto idle;
	}

	wait_for_outstanding_writes();

	err = storage_reads();
	if (err) {
		goto idle;
	}
#endif
	err = storage_uninits();
	if (err) {
		goto idle;
	}

	LOG_INF("Storage sample finished.\r\n");
for (volatile int i = 0; i < 10000; i++) 
{
  __NOP();
}
idle:
	/* Enter main loop. */
	while (true) {
		//while (LOG_PROCESS()) {
		//}

		/* Wait for an event. */
		__WFE();

		/* Clear Event Register */
		__SEV();
		__WFE();
	}
}
