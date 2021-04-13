/**
 * @file    keypad_rotdecode.h
 * @author  Christof Baur
 * @date    13.04.2021
 * @version 0.1
 * @copyright   Christof Baur
 * @brief   ESP-32 library to interface with a rotary encoder and buttons
 *
 * License: MIT
 */

#ifndef KEYPAD_ROTDECODE_H
#define KEYPAD_ROTDECODE_H

#include "esp_err.h"

#define EVNT_BTN_PRSS 1
#define EVNT_BTN_RELS 2
#define EVNT_ROT 3

/**
 * Initialize keypad and rotary AB decoder
 *
 * @note Uses the ULP (loads the ULP program into RTC memory)
 * @note Configure key inputs and encoder AB inputs using menuconfig
 * @note Inputs must have an external pull-down resistor and switchs to GND
 *
 * @param host  pointer to structure defining host controller
 * @param out_card  pointer to structure which will receive information
 *                  about the card when the function completes
 * @return
 *      - ESP_OK on success
 */
void key_rot_init(void);

/**
 * Blocking read
 * Register the currently running task as task waiting for key inputs
 *
 * @note Only a single task can be registered
 *
 * @return
 *      - ESP_OK   on success
 *      - ESP_FAIL if task cannot be registered
 */
esp_err_t key_rot_read(unsigned int* key_code_p, int timeout);

#endif
