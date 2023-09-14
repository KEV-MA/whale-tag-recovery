/*
 * power_led.c
 *
 *  Created on: Sep 14, 2023
 *      Author: kevinma
 */

#include "Lib Inc/power_led.h"
#include "main.h"

void power_led_thread_entry(ULONG thread_input) {

	// Turns on power LED
	HAL_GPIO_TogglePin(GPIOB, GPIO_PIN_2);

}
