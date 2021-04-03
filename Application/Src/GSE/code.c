/*
 * code.c
 *
 *  Created on: 21 Feb 2021
 *      Author: Lucas Pallez
 *
 */
#include <cmsis_os.h>
#include <can_transmission.h>
#include <can_reception.h>
#include <debug/console.h>
#include <debug/led.h>
#include <GSE/code.h>
#include <stm32_hal_legacy.h>
#include <stm32f446xx.h>
#include <stm32f4xx_hal.h>
#include <stm32f4xx_hal_gpio.h>
#include <sys/_stdint.h>

#define CODE_SIZE 4

static uint8_t led_GSE_code_id;

void code_init(void)
{
	led_GSE_code_id = led_register_TK();
	rocket_log("Security code initialised.\n");
}


void TK_code_control(void const * argument)
{
	int code[CODE_SIZE];
	static uint32_t code_int;
	led_set_TK_rgb(led_GSE_code_id, 0, 255, 0);

	for(;;)
	{
		//S2
		code[3] = HAL_GPIO_ReadPin(GPIOA, GPIO_PIN_15); //Read D0 //BIT3
		code[1] = HAL_GPIO_ReadPin(GPIOB, GPIO_PIN_9); //Read D1 //BIT1
		code[0] = HAL_GPIO_ReadPin(GPIOB, GPIO_PIN_7); //Read D2 //BIT0
		code[2] = HAL_GPIO_ReadPin(GPIOC, GPIO_PIN_13); //Read D3 //BIT2

		//Convert into single int (binary sum)
		code_int = 0;
		for (int i = 0; i < CODE_SIZE; i++)
		    code_int = 2 * code_int + code[i];

		can_setFrame(code_int, DATA_ID_GSE_CODE, HAL_GetTick());
		HAL_IWDG_Refresh(&hiwdg);
		osDelay(500);
	}
}

uint8_t verify_security_code()
{
	if (can_getGSEState().code == can_getGSTCode())
		return 1;
	else
		return 0;
}
