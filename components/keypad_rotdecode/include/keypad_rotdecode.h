/**
 * @file    keypad_rotdecode.h
 * @author  Christof Baur
 * @date    05.04.2021
 * @version 0.1
 * @copyright   Christof Baur
 * @brief   ESP-32 library to interface with a rotary encoder and buttons
 *
 * License: MIT
 */

#ifndef KEYPAD_ROTDECODE_H
#define KEYPAD_ROTDECODE_H

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
 * Register the currently running task as task waiting for key inputs
 *
 * @note Only a single task can be registered
 *
 * @return
 *      - ESP_OK   on success
 *      - ESP_FAIL if task cannot be registered
 */
esp_err_t reg_wait_task(void);

/**
 * Wait for key or rotary event; there is no timeout
 *
 * @note
 *
 * @return
 *      event code; btn and rot count
 *      0-1: envent 1 = button pressed, 2 = button released, 3 = rotation
 *      2-15: buttons (every bit corresponds to a button)
 *     16-31: rot counter
 */

uint32_t wait_keyrot(void);


#endif
