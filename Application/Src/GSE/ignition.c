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

void ignition_sys_init(void)
{

#if defined(HB1_CODE_BOARD) || defined (HB3_POWER_BOARD)
	//Set all S1 valves to low
	HAL_GPIO_WritePin(GPIOA, GPIO_PIN_0, GPIO_PIN_RESET);
	HAL_GPIO_WritePin(GPIOA, GPIO_PIN_1, GPIO_PIN_RESET);
	HAL_GPIO_WritePin(GPIOA, GPIO_PIN_11, GPIO_PIN_RESET);
	HAL_GPIO_WritePin(GPIOB, GPIO_PIN_12, GPIO_PIN_RESET);
#endif

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

	 for(;;)
	 {
//		 rocket_log("GST Code: %d \n", can_getGSTCode());
//		 rocket_log("GSE Code: %d \n", can_getGSEState().code);
		 if(verify_security_code())
		 {
			 ignition_order = can_getIgnitionOrder();
			 if(old_ignition_order != ignition_order)
			 {
				 old_ignition_order = ignition_order;
				 switch (ignition_order)
				 {
					case MAIN_IGNITION_ON: //Main Ignition On
					{
						HAL_GPIO_WritePin(GPIOA, GPIO_PIN_0, GPIO_PIN_SET);
						rocket_log("IGNITION ON!\n");
						can_setFrame(GPIO_PIN_SET, DATA_ID_MAIN_IGNITION_STATE, HAL_GetTick());
						break;
					}
					case MAIN_IGNITION_OFF: //Main Ignition Off
					{
						rocket_log("IGNITION OFF!\n");
						HAL_GPIO_WritePin(GPIOA, GPIO_PIN_0, GPIO_PIN_RESET);
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
			 if(disconnect_order == DISCONNECT_HOSE)
				 if(old_disconnect_order != disconnect_order)
				 {
					disconnect_order = old_disconnect_order;
					HAL_GPIO_WritePin(GPIOA, GPIO_PIN_11, GPIO_PIN_SET);
					HAL_GPIO_WritePin(GPIOB, GPIO_PIN_12, GPIO_PIN_SET);
					can_setFrame(GPIO_PIN_SET, DATA_ID_HOSE_DISCONNECT_STATE, HAL_GetTick());
				 }
		 }
		 else
		 {
			 //TODO If wrong code input, what do
		 }
		 osDelay(1000);
	 }
}

