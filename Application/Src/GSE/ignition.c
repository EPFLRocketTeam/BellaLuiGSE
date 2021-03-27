/*
 * ignition.c
 *
 *  Created on: 21 Feb 2021
 *      Author: Lucas Pallez
 *
 */

#include "main.h"
#include <cmsis_os.h>
#include <can_transmission.h>
#include <can_reception.h>
#include <debug/led.h>
#include <debug/console.h>
#include <GSE/code.h>
#include <GSE/ignition.h>
#include <misc/common.h>
#include <stm32f446xx.h>

#include <sys/_stdint.h>

uint32_t i=0;
uint8_t led_GSE_ignition_id;
void ignition_sys_init(void)
{

#if defined(HB1_CODE_BOARD) || defined (HB3_POWER_BOARD)
	//Set all S1 pins to low
	HAL_GPIO_WritePin(GPIOA, GPIO_PIN_0, GPIO_PIN_RESET);
	HAL_GPIO_WritePin(GPIOA, GPIO_PIN_1, GPIO_PIN_RESET);
	HAL_GPIO_WritePin(GPIOA, GPIO_PIN_11, GPIO_PIN_RESET);
	HAL_GPIO_WritePin(GPIOB, GPIO_PIN_12, GPIO_PIN_RESET);
#endif

	led_GSE_ignition_id = led_register_TK();

	rocket_log("Ignition systems initialised.\n");
	led_set_rgb(243,156,67);
}


void TK_ignition_control(void const * argument)
{
	//if xbee code = ignition (D0)
	//if xbee code = ignition sec (D1)

	uint8_t disconnect_order = 0;
	uint8_t old_disconnect_order = 0;
	uint8_t ignition_order = 0;
	uint8_t old_ignition_order = 0;

	//TODO Add sensor confirmation for main ignition and disconnect
	//TODO Add delay after ignition to shut it down automatically


	/*
	 * 22pin Pinout:
	 * 1.1 S1_D3
	 * 1.2 S1_D1
	 * 1.3 S1_D2
	 * 1.4 S1_D0
	 *
	 * Sens1 S1_A1
	 * Sens4 S1_A0
	 */

	 for(;;)
	 {
		 if(verify_security_code())
		 {
			led_set_TK_rgb(led_GSE_ignition_id, 255, 117, 16);

			 ignition_order = can_getIgnitionOrder();
//			 ignition_order = MAIN_IGNITION_ON;
			 rocket_log("Ignition Order received with payload: %d, old: %d\n", ignition_order, old_ignition_order);

			 if(old_ignition_order != ignition_order)
			 {
				 old_ignition_order = ignition_order;
				 switch (ignition_order)
				 {
					case MAIN_IGNITION_ON: //Main Ignition On
					{
						HAL_GPIO_WritePin(GPIOB, GPIO_PIN_12, GPIO_PIN_SET); //Turn on S1_D3
						HAL_GPIO_WritePin(GPIOA, GPIO_PIN_1, GPIO_PIN_SET); //Turn on S1_D1 ===================== TEST =============

						rocket_log("IGNITION ON!\n");
						can_setFrame(GPIO_PIN_SET, DATA_ID_MAIN_IGNITION_STATE, HAL_GetTick());
						break;
					}
					case MAIN_IGNITION_OFF: //Main Ignition Off
					{
						rocket_log("IGNITION OFF!\n");
						HAL_GPIO_WritePin(GPIOB, GPIO_PIN_12, GPIO_PIN_RESET); //Turn off S1_D3
						HAL_GPIO_WritePin(GPIOA, GPIO_PIN_1, GPIO_PIN_RESET); //Turn off S1_D1 ===================== TEST =============

						can_setFrame(GPIO_PIN_RESET, DATA_ID_MAIN_IGNITION_STATE, HAL_GetTick());
						break;
					}
					case SECONDARY_IGNITION_ON: //Secondary Ignition On
					{
						rocket_log("IGNITION SEC ON!\n");
						HAL_GPIO_WritePin(GPIOA, GPIO_PIN_1, GPIO_PIN_SET);
						can_setFrame(GPIO_PIN_SET, DATA_ID_SEC_IGNITION_STATE, HAL_GetTick());
						break;
					}
					case SECONDARY_IGNITION_OFF: //Secondary Ignition Off
					{
						rocket_log("IGNITION SEC OFF!\n");
						HAL_GPIO_WritePin(GPIOA, GPIO_PIN_1, GPIO_PIN_RESET);
						can_setFrame(GPIO_PIN_RESET, DATA_ID_SEC_IGNITION_STATE, HAL_GetTick());
						break;
					}

				 }
			 }
			 disconnect_order = can_getOrder();
			 if(old_disconnect_order != disconnect_order)
			 {
				 old_disconnect_order = disconnect_order;
				 switch (disconnect_order)
				 {
					case MAIN_DISCONNECT_ON: //Main Disconnect On
					{
						HAL_GPIO_WritePin(GPIOA, GPIO_PIN_0, GPIO_PIN_SET);
						rocket_log("DISCONNECT ON!\n");
						can_setFrame(GPIO_PIN_SET, DATA_ID_MAIN_DISCONNECT_STATE, HAL_GetTick());
						break;
					}
					case MAIN_DISCONNECT_OFF: //Main Disconnect Off
					{
						rocket_log("DISCONNECT OFF!\n");
						HAL_GPIO_WritePin(GPIOA, GPIO_PIN_0, GPIO_PIN_RESET);
						can_setFrame(GPIO_PIN_RESET, DATA_ID_MAIN_DISCONNECT_STATE, HAL_GetTick());
						break;
					}
					case SECONDARY_DISCONNECT_ON: //Secondary Disconnect On
					{
						rocket_log("DISCONNECT SEC ON!\n");
						HAL_GPIO_WritePin(GPIOA, GPIO_PIN_1, GPIO_PIN_SET);
						can_setFrame(GPIO_PIN_SET, DATA_ID_SEC_DISCONNECT_STATE, HAL_GetTick());
						break;
					}
					case SECONDARY_DISCONNECT_OFF: //Secondary Ignition Off
					{
						rocket_log("DISCONNECT SEC OFF!\n");
						HAL_GPIO_WritePin(GPIOA, GPIO_PIN_1, GPIO_PIN_RESET);
						can_setFrame(GPIO_PIN_RESET, DATA_ID_SEC_DISCONNECT_STATE, HAL_GetTick());
						break;
					}
				 }
			 }
		 }
		 else
		 {
			 led_set_TK_rgb(led_GSE_ignition_id, 255, 0, 0);
		 }
		 osDelay(1000);
	 }
}

