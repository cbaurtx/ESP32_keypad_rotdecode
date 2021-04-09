/* Usage example
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
    long unsigned int key_code;
    key_rot_init();

    for(;;) {
        key_code = reg_wait_task();
        printf("Key event: %8lx\n", key_code);
        taskYIELD();
        }

    }
