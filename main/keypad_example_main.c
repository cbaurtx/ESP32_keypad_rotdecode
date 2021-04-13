/**
 * @file    keypad_example_main.h
 * @author  Christof Baur
 * @date    13.04.2021
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
#include "esp_sleep.h"

#include "keypad_rotdecode.h"


void app_main(void)
{
    unsigned int key_code;
    esp_err_t err;

    key_rot_init();

    for(;;) {
      err = key_rot_read(&key_code, 500); // 5 sec later change to 1 minute timeout

      if(err == ESP_OK)
        switch (key_code & 3) {
          case EVNT_BTN_PRSS:
            printf("Button pressed: %x\n", (key_code & 0xfffc) >> 2);
            break;
          case EVNT_BTN_RELS:
            printf("Button released: %x\n", (key_code& 0xfffc) >> 2);
            break;
          case EVNT_ROT:
            printf("Turned know: %d\n", key_code >> 0x12 );
            break;
          default:
            printf("Invalid\n");

        }
      if(err == ESP_ERR_TIMEOUT)
        printf("Timed out\n");

      taskYIELD();
    }
}

