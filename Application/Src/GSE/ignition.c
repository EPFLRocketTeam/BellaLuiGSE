/*
 * ignition.c
 *
 *  Created on: 21 Feb 2021
 *      Author: Lucas Pallez
 *
 */

#include <cmsis_os.h>
#include <can_transmission.h>
#include <can_reception.h>
#include <debug/console.h>
#include <GSE/code.h>
#include <GSE/ignition.h>
#include <misc/common.h>
#include <stm32f446xx.h>

#include <sys/_stdint.h>

uint32_t i=0;

void ignition_sys_init(void)
{
	rocket_log("Ignition systems initialised.\n");

	//Set Ignition values to low
	HAL_GPIO_WritePin(GPIOA, GPIO_PIN_0, GPIO_PIN_RESET);
	HAL_GPIO_WritePin(GPIOA, GPIO_PIN_1, GPIO_PIN_RESET);
	HAL_GPIO_WritePin(GPIOA, GPIO_PIN_11, GPIO_PIN_RESET);
	HAL_GPIO_WritePin(GPIOB, GPIO_PIN_12, GPIO_PIN_RESET);


}


void TK_ignition_control(void const * argument)
{
	//if xbee code = ignition (D0)
	//if xbee code = ignition sec (D1)

	uint8_t disconnect_order = 0;
	uint8_t ignition_order = 0;
	uint8_t old_ignition_order = 0;
	float current = 0;

	//TODO Add sensor confirmation for main ignition and disconnect
	//TODO Add delay after ignition to shut it down automatically

	 for(;;)
	 {
//		 rocket_log("GST Code: %d \n", can_getGSTCode());
//		 rocket_log("GSE Code: %d \n", can_getGSEState().code);
		 if(verify_security_code(can_getGSTCode()))
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
			 {
				HAL_GPIO_WritePin(GPIOA, GPIO_PIN_11, GPIO_PIN_SET);
				HAL_GPIO_WritePin(GPIOB, GPIO_PIN_12, GPIO_PIN_SET);
					can_setFrame(GPIO_PIN_SET, DATA_ID_HOSE_DISCONNECT_STATE, HAL_GetTick());
			 }
		 }
		//Read Current sensor
		//current = read_current();
#ifdef IGNITION_1
		can_setFrame(current, DATA_ID_IGNITION_CURRENT_1, HAL_GetTick());
#endif
#ifdef IGNITION_2
		can_setFrame(current, DATA_ID_IGNITION_CURRENT_2, HAL_GetTick());
#endif
#ifdef DISCONNECT_1
		can_setFrame(current, DATA_ID_DISCONNECT_CURRENT_1, HAL_GetTick());
#endif
#ifdef DISCONNECT_2
		can_setFrame(current, DATA_ID_DISCONNECT_CURRENT_2, HAL_GetTick());
#endif
		osDelay(1000);
	 }
}

float read_current()
{
	float current = 0;

	//TODO implement current sensor reading
	return current;
}

