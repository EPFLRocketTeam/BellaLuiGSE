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

static uint8_t led_GSE_ignition_id;
void ignition_sys_init(void)
{

#if defined(HB2_CODE_BOARD) || defined (HB3_POWER_BOARD)
	//Set all S1 pins to low
	HAL_GPIO_WritePin(GPIOA, GPIO_PIN_0, GPIO_PIN_RESET);
	HAL_GPIO_WritePin(GPIOA, GPIO_PIN_1, GPIO_PIN_RESET);
	HAL_GPIO_WritePin(GPIOA, GPIO_PIN_11, GPIO_PIN_RESET);
	HAL_GPIO_WritePin(GPIOB, GPIO_PIN_12, GPIO_PIN_RESET);
#endif

	led_GSE_ignition_id = led_register_TK();

	rocket_log("Ignition systems initialised.\n");
}


void TK_ignition_control(void const * argument)
{

	static uint32_t ignition_order = 0;
	static uint32_t old_ignition_order = 0;

	//TODO Add sensor confirmation for main ignition and disconnect
	//TODO Add delay after ignition to shut it down automatically

	/*
	 * 22pin Pinout:
	 * 1.1 HB2_S1_D3 -> Secondary Ignition		(sensed)
	 * 1.2 HB2_S1_D1 -> Secondary Ignition
	 * 1.3 HB2_S1_D2 -> Secondary Disconnect
	 * 1.4 HB2_S1_D0 -> Secondary Disconnect	(sensed)
	 * 2.1 HB3_S1_D2 -> Main Ignition 			(sensed)
	 * 2.2 HB3_S1_D1 -> Main Ignition
	 * 2.3 HB3_S1_D3 -> Main Disconnect
	 * 2.4 HB3_S1_D0 -> Main Disconnect 		(sensed)
	 * 3.1 HB3_S2_D3 -> Main Fill Valve 		(sensed)
	 * 3.2 HB3_S2_D2 -> Main Fill Valve
	 * 3.3 HB3_S2_D1 -> Main Purge Valve
	 * 3.4 HB3_S2_D0 -> Main Purge Valve 		(sensed)
	 *
	 * Sens1 HB2_S1_A1
	 * Sens2 HB3_S1-A1
	 * Sens3 HB3_S2_A0
	 * Sens4 HB2_S1_A0
	 * Sens5 HB3_S1_A0
	 * Sens6 HB3_S2_A1
	 *
	 *
	 * 12pin Pinout:
	 * 1. 	3.4 -> Sens6
	 * 2. 	1.4 -> Sens4
	 * 3. 	2.3
	 * 4. 	1.2
	 * 5. 	3.2
	 * 6. 	2.1 -> Sens2
	 * 7. 	2.4 -> Sens5
	 * 8. 	3.3
	 * 9.	1.3
	 * 10.	2.2
	 * 11.	1.1 -> Sens1
	 * 12.	3.1 -> Sens3
	 *
	 */

	 for(;;)
	 {
		 led_set_TK_rgb(led_GSE_ignition_id, 255, 117, 16);
		 ignition_order = can_getIgnitionOrder();
//		 rocket_log("Ignition Order received with payload: %d, old: %d\n", ignition_order, old_ignition_order);

		 if(!verify_security_code())
		 {
			 led_set_TK_rgb(led_GSE_ignition_id, 255, 0, 0);
			 can_setFrame(0, DATA_ID_GST_CODE, HAL_GetTick());
		 }
		 if(old_ignition_order != ignition_order)
		 {
			 old_ignition_order = ignition_order;
			 switch (ignition_order)
			 {
				case MAIN_IGNITION_ON: //Main Ignition On
				{
					if(verify_security_code())
					{
#ifdef HB3_POWER_BOARD
						rocket_log("IGNITION ON!\n");
						HAL_GPIO_WritePin(GPIOA, GPIO_PIN_11, GPIO_PIN_SET); //Turn on S1_D2
						HAL_GPIO_WritePin(GPIOA, GPIO_PIN_1, GPIO_PIN_SET); //Turn on S1_D1
						can_setFrame(GPIO_PIN_SET, DATA_ID_MAIN_IGNITION_STATE, HAL_GetTick());
#endif
#ifdef HB2_CODE_BOARD
						//rocket_log("IGNITION SEC ON!\n");
						HAL_GPIO_WritePin(GPIOB, GPIO_PIN_12, GPIO_PIN_SET); //Turn on S1_D3
						HAL_GPIO_WritePin(GPIOA, GPIO_PIN_1, GPIO_PIN_SET); //Turn on S1_D1
						can_setFrame(GPIO_PIN_SET, DATA_ID_SEC_IGNITION_STATE, HAL_GetTick());
#endif
					}
					break;
				}
				case MAIN_IGNITION_OFF: //Main Ignition Off
				{
#ifdef HB3_POWER_BOARD
					//rocket_log("IGNITION OFF!\n");
					HAL_GPIO_WritePin(GPIOA, GPIO_PIN_11, GPIO_PIN_RESET); //Turn off S1_D2
					HAL_GPIO_WritePin(GPIOA, GPIO_PIN_1, GPIO_PIN_RESET); //Turn off S1_D1
					can_setFrame(GPIO_PIN_RESET, DATA_ID_MAIN_IGNITION_STATE, HAL_GetTick());
#endif
#ifdef HB2_CODE_BOARD
					//rocket_log("IGNITION SEC OFF!\n");
					HAL_GPIO_WritePin(GPIOB, GPIO_PIN_12, GPIO_PIN_RESET); //Turn off S1_D3
					HAL_GPIO_WritePin(GPIOA, GPIO_PIN_1, GPIO_PIN_RESET); //Turn off S1_D1
					can_setFrame(GPIO_PIN_RESET, DATA_ID_SEC_IGNITION_STATE, HAL_GetTick());
#endif
					break;


				}
#ifdef HB2_CODE_BOARD
				case SECONDARY_IGNITION_ON: //Secondary Ignition On
				{
					if(verify_security_code())
					{
						//rocket_log("IGNITION SEC ON!\n");
						HAL_GPIO_WritePin(GPIOB, GPIO_PIN_12, GPIO_PIN_SET); //Turn on S1_D3
						HAL_GPIO_WritePin(GPIOA, GPIO_PIN_1, GPIO_PIN_SET); //Turn on S1_D1
						can_setFrame(GPIO_PIN_SET, DATA_ID_SEC_IGNITION_STATE, HAL_GetTick());
					}
					break;
				}
#endif
#ifdef HB2_CODE_BOARD
				case SECONDARY_IGNITION_OFF: //Secondary Ignition Off
				{
					//rocket_log("IGNITION SEC OFF!\n");
					HAL_GPIO_WritePin(GPIOB, GPIO_PIN_12, GPIO_PIN_RESET); //Turn off S1_D3
					HAL_GPIO_WritePin(GPIOA, GPIO_PIN_1, GPIO_PIN_RESET); //Turn off S1_D1
					can_setFrame(GPIO_PIN_RESET, DATA_ID_SEC_IGNITION_STATE, HAL_GetTick());
					break;
				}
#endif
				default:
				{
					rocket_log("Unknown Ignition order received\n");
					led_set_TK_rgb(led_GSE_ignition_id, 255, 255, 255);
				}

			 }
		 }
		HAL_IWDG_Refresh(&hiwdg);
		 osDelay(20);
	 }
}

