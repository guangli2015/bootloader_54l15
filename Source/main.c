/*********************************************************************
*                    SEGGER Microcontroller GmbH                     *
*                        The Embedded Experts                        *
**********************************************************************

-------------------------- END-OF-HEADER -----------------------------

File    : main.c
Purpose : Generic application start

*/
/* FreeRTOS include. */
#include "FreeRTOS.h"
#include "task.h"
#include "timers.h"
/*#define NRFX_UARTE_ENABLED 1
#define NRFX_UARTE30_ENABLED 1
#define NRFX_UARTE00_ENABLED 1
#define NRFX_UARTE20_ENABLED 1


#define NRFX_UARTE21_ENABLED 1


#define NRFX_UARTE22_ENABLED 1*/

#define BOARD_APP_UARTE_PIN_TX NRF_PIN_PORT_TO_PIN_NUMBER(0, 0)


#define BOARD_APP_UARTE_PIN_RX NRF_PIN_PORT_TO_PIN_NUMBER(1, 0)

#define BOARD_APP_UARTE_PIN_RTS NRF_PIN_PORT_TO_PIN_NUMBER(2, 0)


#define BOARD_APP_UARTE_PIN_CTS NRF_PIN_PORT_TO_PIN_NUMBER(3, 0)
#include <hal/nrf_gpio.h>
#include <nrfx_uarte.h>
#include "test_section.h"
#include <stdio.h>
#include <string.h>
#include  "log.h"
#include "irq_connect.h"

#include <nrfx_grtc.h>
#include <nrfx_clock.h>
#include "err_num.h"
#include "bm_buttons.h"
#include <stdbool.h>



/** @brief Macro for extracting absolute pin number from the relative pin and port numbers. */
#define NRF_PIN_PORT_TO_PIN_NUMBER(pin, port) (((pin) & 0x1F) | ((port) << 5))
#define BOARD_PIN_LED_1 NRF_PIN_PORT_TO_PIN_NUMBER(10, 1)

#ifndef BOARD_PIN_LED_0
#define BOARD_PIN_LED_0 NRF_PIN_PORT_TO_PIN_NUMBER(9, 2)
#endif
#ifndef BOARD_PIN_LED_1
#define BOARD_PIN_LED_1 NRF_PIN_PORT_TO_PIN_NUMBER(10, 1)
#endif
#ifndef BOARD_PIN_LED_2
#define BOARD_PIN_LED_2 NRF_PIN_PORT_TO_PIN_NUMBER(7, 2)
#endif
#ifndef BOARD_PIN_LED_3
#define BOARD_PIN_LED_3 NRF_PIN_PORT_TO_PIN_NUMBER(14, 1)
#endif
#ifndef BOARD_PIN_BTN_0
#define BOARD_PIN_BTN_0 NRF_PIN_PORT_TO_PIN_NUMBER(13, 1)
#endif


void vApplicationStackOverflowHook( TaskHandle_t xTask, char *pcTaskName )
{
    /* Force an assert. */
    configASSERT( pcTaskName == 0 );
}

static void led_toggle_task_function (void * pvParameter)
{
   
    while (true)
    {
       // nrf_gpio_pin_write(BOARD_PIN_LED_0, 1);
          nrf_gpio_pin_toggle(BOARD_PIN_LED_1);
        /* Delay a task for a given number of ticks */
        vTaskDelay(3000);

        /* Tasks must be implemented to never return... */
    }
}
static TaskHandle_t m_softdevice_init_task; 
TaskHandle_t  led_toggle_task_handle;
TimerHandle_t led_toggle_timer_handle;  /**< Reference to LED1 toggling FreeRTOS timer. */



/**@brief The function to call when the LED1 FreeRTOS timer expires.
 *
 * @param[in] pvParameter   Pointer that will be used as the parameter for the timer.
 */
static void led_toggle_timer_callback (TimerHandle_t xTimer )
{
   
    nrf_gpio_pin_toggle(BOARD_PIN_LED_2);
}
StaticTimer_t myTimerBuffer;

extern int sftdevice_test(void);
extern int softdevice_irq_init(void);
extern int ble_lbs_sample(void);
extern void nrf_sdh_freertos_init(void * p_context);
void SysTick_Configuration(void);
/* This function gets events from the SoftDevice and processes them. */
static void softdevice_init_task(void * pvParameter)
{
    LOG_INF("Enter softdevice_init\r\n");


    ble_lbs_sample();
   

}
extern int ble_buttonless_dfu_sample(void);
extern void bootloader_start(void);
static uint64_t last_count; /* Time (SYSCOUNTER value) @last sys_clock_announce() */
int main(void)
{
SysTick_Configuration();
  softdevice_irq_init();

  log_init();

  //nrf_sdh_freertos_init( NULL);
  nrf_gpio_cfg_output(BOARD_PIN_LED_0);
  nrf_gpio_cfg_output(BOARD_PIN_LED_1);
  nrf_gpio_cfg_output(BOARD_PIN_LED_2);
  nrf_gpio_cfg_output(BOARD_PIN_LED_3);
 LOG_INF("%d\r\n",last_count);

#if 0
    BaseType_t xReturned1 = xTaskCreate(softdevice_init_task,
                                       "BLE_SD",
                                       512,
                                       NULL,
                                        3 ,
                                       &m_softdevice_init_task);
    if (xReturned1 != pdPASS)
    {
        LOG_INF("SoftDevice task not created\r\n");

    }

    xTaskCreate(led_toggle_task_function, "LED0", configMINIMAL_SECURE_STACK_SIZE + 200, NULL,  2, &led_toggle_task_handle);

        /* Start timer for LED1 blinking */
    led_toggle_timer_handle = xTimerCreateStatic( "LED1", 1000, pdTRUE, NULL, led_toggle_timer_callback,&myTimerBuffer);
    xTimerStart(led_toggle_timer_handle, 0);

        /* Start FreeRTOS scheduler. */
    vTaskStartScheduler();
#endif
bootloader_start();

    return 0;
}

void _start(void) {
main();
    
}


/*--------------------add GRTC driver for systick by Andrew------------------------------*/
#if 1
#define SYS_CLOCK_HW_CYCLES_PER_SEC 1000000
#define SYS_CLOCK_TICKS_PER_SEC 1000
#define CYC_PER_TICK                                                                               \
	((uint64_t)SYS_CLOCK_HW_CYCLES_PER_SEC / (uint64_t)SYS_CLOCK_TICKS_PER_SEC)
static void sys_clock_timeout_handler(int32_t id, uint64_t cc_val, void *p_context);
static nrfx_grtc_channel_t system_clock_channel_data = {
	.handler = sys_clock_timeout_handler,
	.p_context = NULL,
	.channel = (uint8_t)-1,
};

static inline uint64_t counter(void)
{
	uint64_t now;
	nrfx_grtc_syscounter_get(&now);
	return now;
}
static inline uint64_t counter_sub(uint64_t a, uint64_t b)
{
	return (a - b);
}
/*
 * Program a new callback in the absolute time given by <value>
 */
static void system_timeout_set_abs(uint64_t value)
{
	nrfx_grtc_syscounter_cc_absolute_set(&system_clock_channel_data, value,
					     true);
}
static void sys_clock_timeout_handler(int32_t id, uint64_t cc_val, void *p_context)
{
	//ARG_UNUSED(id);
	//ARG_UNUSED(p_context);
	uint64_t dticks;
	uint64_t now = counter();

	//if (unlikely(now < cc_val)) {
	//	return;
	//}

	dticks = counter_sub(cc_val, last_count) / CYC_PER_TICK;

	last_count += dticks * CYC_PER_TICK;


	system_timeout_set_abs(last_count + CYC_PER_TICK);
#if 0
        uint32_t ulPreviousMask;

    ulPreviousMask = portSET_INTERRUPT_MASK_FROM_ISR();
    traceISR_ENTER();
    {
        /* Increment the RTOS tick. */
        if( xTaskIncrementTick() != pdFALSE )
        {
            traceISR_EXIT_TO_SCHEDULER();
            /* Pend a context switch. */
            portNVIC_INT_CTRL_REG = portNVIC_PENDSVSET_BIT;
        }
        else
        {
            traceISR_EXIT();
        }
    }
    portCLEAR_INTERRUPT_MASK_FROM_ISR( ulPreviousMask );
    #endif
}

static void clk_event_handler(nrfx_clock_evt_type_t event){}
static void system_timeout_set_relative(uint64_t value)
{
	if (value <= NRF_GRTC_SYSCOUNTER_CCADD_MASK) {
		nrfx_grtc_syscounter_cc_relative_set(&system_clock_channel_data, value, true,
						     NRFX_GRTC_CC_RELATIVE_SYSCOUNTER);
	} else {
		nrfx_grtc_syscounter_cc_absolute_set(&system_clock_channel_data, value + counter(),
						     true);
	}
}
static int sys_clock_driver_init(void)
{
  nrfx_err_t err_code;

  nrfx_grtc_clock_source_set(NRF_GRTC_CLKSEL_LFXO);

  err_code = nrfx_grtc_init(configLIBRARY_MAX_SYSCALL_INTERRUPT_PRIORITY);
  if (err_code != NRFX_SUCCESS) {
		return -1;
  }


  err_code = nrfx_grtc_syscounter_start(true, &system_clock_channel_data.channel);
  if (err_code != NRFX_SUCCESS) {
		return -1;
  }
	
  system_timeout_set_relative(CYC_PER_TICK);
  return 0;

}
void SysTick_Configuration(void)
{
  nrfx_clock_init(clk_event_handler);	
  //nrfx_clock_enable();
  sys_clock_driver_init();
  //nrfx_clock_lfclk_start();
}
#endif
#if 0
/* configUSE_STATIC_ALLOCATION is set to 1, so the application must provide an
 * implementation of vApplicationGetIdleTaskMemory() to provide the memory that
 * is used by the Idle task. */
void vApplicationGetIdleTaskMemory( StaticTask_t ** ppxIdleTaskTCBBuffer,
                                    StackType_t ** ppxIdleTaskStackBuffer,
                                    uint32_t * pulIdleTaskStackSize )
{
    /* If the buffers to be provided to the Idle task are declared inside this
     * function then they must be declared static - otherwise they will be
     * allocated on the stack and so not exists after this function exits. */
    static StaticTask_t xIdleTaskTCB;
    static StackType_t uxIdleTaskStack[ configMINIMAL_STACK_SIZE ] __attribute__( ( aligned( 32 ) ) );

    /* Pass out a pointer to the StaticTask_t structure in which the Idle
     * task's state will be stored. */
    *ppxIdleTaskTCBBuffer = &xIdleTaskTCB;

    /* Pass out the array that will be used as the Idle task's stack. */
    *ppxIdleTaskStackBuffer = uxIdleTaskStack;

    /* Pass out the size of the array pointed to by *ppxIdleTaskStackBuffer.
     * Note that, as the array is necessarily of type StackType_t,
     * configMINIMAL_STACK_SIZE is specified in words, not bytes. */
    *pulIdleTaskStackSize = configMINIMAL_STACK_SIZE;
}

/* configUSE_STATIC_ALLOCATION and configUSE_TIMERS are both set to 1, so the
 * application must provide an implementation of vApplicationGetTimerTaskMemory()
 * to provide the memory that is used by the Timer service task. */
void vApplicationGetTimerTaskMemory( StaticTask_t ** ppxTimerTaskTCBBuffer,
                                     StackType_t ** ppxTimerTaskStackBuffer,
                                     uint32_t * pulTimerTaskStackSize )
{
    /* If the buffers to be provided to the Timer task are declared inside this
     * function then they must be declared static - otherwise they will be
     * allocated on the stack and so not exists after this function exits. */
    static StaticTask_t xTimerTaskTCB;
    static StackType_t uxTimerTaskStack[ configTIMER_TASK_STACK_DEPTH ] __attribute__( ( aligned( 32 ) ) );

    /* Pass out a pointer to the StaticTask_t structure in which the Timer
     * task's state will be stored. */
    *ppxTimerTaskTCBBuffer = &xTimerTaskTCB;

    /* Pass out the array that will be used as the Timer task's stack. */
    *ppxTimerTaskStackBuffer = uxTimerTaskStack;

    /* Pass out the size of the array pointed to by *ppxTimerTaskStackBuffer.
     * Note that, as the array is necessarily of type StackType_t,
     * configTIMER_TASK_STACK_DEPTH is specified in words, not bytes. */
    *pulTimerTaskStackSize = configTIMER_TASK_STACK_DEPTH;
}
#endif
/*************************** End of file ****************************/
