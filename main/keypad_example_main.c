/* Usage example
*/

#include <stdio.h>
#include <string.h>
#include <stdlib.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "esp_sleep.h"

#include "keypad_rotdecode.h"

// #include "ulp_debounce_decode.h"
// extern const uint8_t ulp_main_bin_start[] asm("_binary_ulp_main_bin_start");
// extern const uint8_t ulp_main_bin_end[]   asm("_binary_ulp_main_bin_end");

void app_main(void)
{
    long unsigned int key_code;
    key_rot_init();

    for(;;) {
        key_code = 0; //reg_wait_task();
        printf("Key event: %8lx\n", key_code);
        taskYIELD();
        }

    }
