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
    unsigned int key_code;
    esp_err_t err;

    key_rot_init();

    for(;;) {
      err = key_rot_read(&key_code, 500); // 5 sec later change to 1 minute timeout

      if(err == ESP_OK)
        printf("Key event: %8x\n", key_code);
      if(err == ESP_ERR_TIMEOUT)
        printf("Timed out\n");

      taskYIELD();
    }
}

