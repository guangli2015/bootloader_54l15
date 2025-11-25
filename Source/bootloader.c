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
/* Attention!
 *  To maintain compliance with Nordic Semiconductor ASA's Bluetooth profile
 *  qualification listings, this section of source code must not be modified.
 */

#include <nrf_sdh.h>
#include <nrf_sdh_ble.h>
#include <ble_adv.h>
#include <ble_gap.h>
#include <nrf_soc.h>

#include "nrf_soc.h"
#include "log.h"
#include <stdbool.h>
#include "prj_config.h"

#include "SEGGER_RTT.h"
#include <hal/nrf_clock.h>
#define APP_START_ADDRESS 0x0000b000
#define BOOTLOADER_DFU_GPREGRET_MASK            (0xF8)      /**< Mask for GPGPREGRET bits used for the magic pattern written to GPREGRET register to signal between main app and DFU. */
#define BOOTLOADER_DFU_GPREGRET                 (0xB0)      /**< Magic pattern written to GPREGRET register to signal between main app and DFU. The 3 lower bits are assumed to be used for signalling purposes.*/
#define BOOTLOADER_DFU_START_BIT_MASK           (0x01)      /**< Bit mask to signal from main application to enter DFU mode using a buttonless service. */

#define BOOTLOADER_DFU_GPREGRET2_MASK           (0xF8)      /**< Mask for GPGPREGRET2 bits used for the magic pattern written to GPREGRET2 register to signal between main app and DFU. */
#define BOOTLOADER_DFU_GPREGRET2                (0xA8)      /**< Magic pattern written to GPREGRET2 register to signal between main app and DFU. The 3 lower bits are assumed to be used for signalling purposes.*/
#define BOOTLOADER_DFU_SKIP_CRC_BIT_MASK        (0x01)      /**< Bit mask to signal from main application that CRC-check is not needed for image verification. */


#define BOOTLOADER_DFU_START_MASK    (BOOTLOADER_DFU_GPREGRET_MASK | BOOTLOADER_DFU_START_BIT_MASK)
#define BOOTLOADER_DFU_START    (BOOTLOADER_DFU_GPREGRET | BOOTLOADER_DFU_START_BIT_MASK)      /**< Magic number to signal that bootloader should enter DFU mode because of signal from Buttonless DFU in main app.*/
#define BOOTLOADER_DFU_SKIP_CRC_MASK (BOOTLOADER_DFU_GPREGRET2_MASK | BOOTLOADER_DFU_SKIP_CRC_BIT_MASK)
#define BOOTLOADER_DFU_SKIP_CRC (BOOTLOADER_DFU_GPREGRET2 | BOOTLOADER_DFU_SKIP_CRC_BIT_MASK)  /**< Magic number to signal that CRC can be skipped due to low power modes.*/

#define VERIFY_SUCCESS(statement)                       \
do                                                      \
{                                                       \
    uint32_t _err_code = (uint32_t) (statement);        \
    if (_err_code != NRF_SUCCESS)                       \
    {                                                   \
        return _err_code;                               \
    }                                                   \
} while(0)

bool ble_dfu_enter_check(void)
{
    uint32_t err_code;
    uint32_t content;
    LOG_INF("In ble_dfu_buttonless_bootloader_start_finalize\r\n");
    err_code = sd_power_gpregret_get(0, &content);
    //VERIFY_SUCCESS(err_code);
     if (err_code != NRF_SUCCESS)                       \
    {   
        sd_power_gpregret_clr(0, 0xffffffff);
        LOG_INF("sd_power_gpregret_get fail\r\n");                                                \
        return false;                               \
    }  
    
    if ((content & BOOTLOADER_DFU_START_MASK) == BOOTLOADER_DFU_START)
    {
        sd_power_gpregret_clr(0, 0xffffffff);
        LOG_INF("DFU mode requested via GPREGRET.\r\n");
        return true;
    }
   
    return false;
}




void cleanup_arm_nvic(void) {
	/* Allow any pending interrupts to be recognized */
	__ISB();
	__disable_irq();

	/* Disable NVIC interrupts */
	for (uint8_t i = 0; i < ARRAY_SIZE(NVIC->ICER); i++) {
		NVIC->ICER[i] = 0xFFFFFFFF;
	}
	/* Clear pending NVIC interrupts */
	for (uint8_t i = 0; i < ARRAY_SIZE(NVIC->ICPR); i++) {
		NVIC->ICPR[i] = 0xFFFFFFFF;
	}
}	
static void nrf_cleanup_clock(void)
{
    nrf_clock_int_disable(NRF_CLOCK, 0xFFFFFFFF);
}


struct arm_vector_table {
    uint32_t msp;
    uint32_t reset;
};

__STATIC_INLINE void jump_to_app(uint32_t vector_table_addr)
{

    static struct arm_vector_table *vt;
        vt = (struct arm_vector_table *)(vector_table_addr);
    SEGGER_RTT_printf(0,"vector_table_addr 0x%x MSP=0x%x, Reset_Handler=0x%x\r\n", vector_table_addr,vt->msp, vt->reset);


    nrf_cleanup_clock();

    __set_PSPLIM(0);
    __set_MSPLIM(0);


    __set_CONTROL(0x00); /* application will configures core on its own */
    __ISB();

    __set_MSP(vt->msp);

    ((void (*)(void))vt->reset)();
}

void bootloader_start(void)
{
    int err;
    const uint32_t app_vector_table = APP_START_ADDRESS;
    bool dfu_enter_check = false;
#if 0
    err = nrf_sdh_enable_request();
	if (err) {
		LOG_INF("Failed to enable SoftDevice, err %d\r\n", err);
		return;
	}

	LOG_INF("SoftDevice enabled\r\n");

	err = nrf_sdh_ble_enable(CONFIG_NRF_SDH_BLE_CONN_TAG);
	if (err) {
		LOG_INF("Failed to enable BLE, err %d\r\n", err);
		return;
	}

	LOG_INF("Bluetooth enabled\r\n");
 #endif  
//dfu_enter_check =ble_dfu_enter_check();
  if(dfu_enter_check)
  {
      LOG_INF("Bootloader: enter dfu mode \r\n", app_vector_table);
  }
  else
  {
#if 0
        err = nrf_sdh_disable_request();
	if (err) {
		LOG_INF("Failed to disable SoftDevice, err %d\r\n", err);
		return;
	}
#endif
    LOG_INF("start application at 0x%x\r\n", app_vector_table);

    jump_to_app(app_vector_table);

    while (1); // 不应该到这里

  }

}

