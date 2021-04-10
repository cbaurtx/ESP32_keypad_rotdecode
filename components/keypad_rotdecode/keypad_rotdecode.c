/**
 * @file    keypad_rotdecode.c
 * @author  Christof Baur
 * @date    05.04.2021
 * @version 0.1
 * @copyright   Christof Baur
 * @brief   ESP-32 library to interface with a rotary encoder and buttons
 *
 * License: MIT
 */

#include <stdio.h>
#include <string.h>
#include <stdlib.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "driver/gpio.h"
#include "esp_sleep.h"

#include "soc/rtc_cntl_reg.h"
#include "soc/sens_reg.h"
#include "soc/rtc_periph.h"
#include "driver/rtc_io.h"
#include "driver/rtc_cntl.h"
#include "esp32/ulp.h"

#include "ulp_debounce_decode.h"
#include "keypad_rotdecode.h"
#include "sdkconfig.h"

#define  CONFIG_AB_RTC_B_GPIO_NUM (CONFIG_AB_RTC_A_GPIO_NUM + 1)


extern const uint8_t ulp_debounce_decode_bin_start[] asm("_binary_ulp_debounce_decode_bin_start");
extern const uint8_t ulp_debounce_decode_bin_end[]   asm("_binary_ulp_debounce_decode_bin_end");

static void IRAM_ATTR keyrot_isr(void *);
static void init_ulp_prog(void);
static void start_ulp_prog(void);
static void init_rtcios(void);
static void set_wake_mask(void);

static void init_rtcio(unsigned int rtcio);


/* shared with ISR */
static TaskHandle_t volatile recv_task;
static unsigned int volatile keyrot_code;
static int volatile cnt_rot;

static const uint8_t rtc2gpio_map[16] = {36, 37, 38, 39, 34, 35, 25, 26, 33, 32, 4, 0, 2, 15, 13, 12};

/**
 * Initialize keypad and rotary AB decoder
 *
 * @note Must be called from a task so
 * @note Uses the ULP (loads the ULP program into RTC memory)
 * @note Configure key inputs and encoder AB inputs using menuconfig
 * @note Inputs must have an external pull-down resistor and switchs to GND
 *
 */
void key_rot_init()
{
  /* register isr, enable interrupt */
  ESP_ERROR_CHECK(rtc_isr_register(keyrot_isr, NULL, RTC_CNTL_SAR_INT_ST_M));
  REG_SET_BIT(RTC_CNTL_INT_ENA_REG, RTC_CNTL_ULP_CP_INT_ENA_M);
  init_ulp_prog();
  init_rtcios();
  recv_task = xTaskGetCurrentTaskHandle();
  set_wake_mask();
  start_ulp_prog();
}


esp_err_t reg_wait_task()
{
    return(0);
}


static void IRAM_ATTR keyrot_isr(void* arg)
/**
 * ULP interrupt service routine. ULP interrupt triggered when ULP executes 'wake' instruction
 * and the Xtensa is not sleeping.
 */
{
    BaseType_t xHigherPriorityTaskWoken = pdFALSE;

    cnt_rot += (short int)ulp_rot_st;
    if (cnt_rot < 0)
        cnt_rot = CONFIG_ROT_CNT_MAX;
    if (cnt_rot > CONFIG_ROT_CNT_MAX)
        cnt_rot = 0;

    keyrot_code = ulp_evnt;
    keyrot_code |= ulp_btn << 2;
    keyrot_code |= cnt_rot << 16;

    ulp_evnt = 0;
    ulp_rot_st = 0;
    ulp_btn = 0;

    /* notify blocked task*/
    xHigherPriorityTaskWoken = pdFALSE;
    xTaskNotifyFromISR(recv_task, keyrot_code, eSetValueWithoutOverwrite, &xHigherPriorityTaskWoken);

    /* clear interruopt */
    WRITE_PERI_REG(RTC_CNTL_INT_CLR_REG, READ_PERI_REG(RTC_CNTL_INT_ST_REG));

    portYIELD_FROM_ISR();
}

static void init_ulp_prog(void)
{
    esp_err_t err = ulp_load_binary(0, ulp_debounce_decode_bin_start,
            (ulp_debounce_decode_bin_end - ulp_debounce_decode_bin_start) / sizeof(uint32_t));
    ESP_ERROR_CHECK(err);

    /* note: ULP variables are initialized by the ULP
     * Each 'ulp_xyz' variable corresponds to 'xyz' variable in the ULP program.
     * Need to include 'ulp_main.h' for this to work
    */
}


static void start_ulp_prog(void)
{
    /* Start the program */
    ESP_ERROR_CHECK(ulp_run(&ulp_entry - RTC_SLOW_MEM));
}

static void init_rtcios(void)
{
    int count;

    /* Init RTCIOs for AB encoder signals */
    init_rtcio(CONFIG_AB_RTC_A_GPIO_NUM);
    init_rtcio(CONFIG_AB_RTC_B_GPIO_NUM);

    /* Init RTCIOs for buttons */

    for(count=0;count<15;count++)
        if ((CONFIG_BTN_RTC_GPIO_MASK >> count) & 0x01)
            init_rtcio(count);

    /* Disconnect GPIO12 and GPIO15 to remove current drain through
     * pullup/pulldown resistors.
     * GPIO12 may be pulled high to select flash voltage.
     */
    rtc_gpio_isolate(GPIO_NUM_12);
    rtc_gpio_isolate(GPIO_NUM_15);
}

static void init_rtcio(unsigned int rtc_io)
{
 int gpio;

 gpio = rtc2gpio_map[rtc_io];

 rtc_gpio_init(gpio);                     // after sleep io mode is RTC, but to be sure...
 rtc_gpio_set_direction(gpio, RTC_GPIO_MODE_INPUT_ONLY); // direction input (also needed for input only pins)
 rtc_gpio_pulldown_dis(gpio);
 rtc_gpio_pullup_dis(gpio);
}

static void set_wake_mask(void)
{
    uint16_t  rtcio_mask;
    int count;

    rtcio_mask = 0;

    for(count=0;count<15;count++)
        if ((CONFIG_RTC_GPIO_WAKE_MASK >> count) & 1)
            rtcio_mask |= (1ULL << (uint64_t)rtc2gpio_map[count]);

    esp_sleep_enable_ext1_wakeup(rtcio_mask, ESP_EXT1_WAKEUP_ANY_HIGH);
}
