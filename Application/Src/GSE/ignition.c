/*
 * ignition.c
 *
 *  Created on: 21 Feb 2021
 *      Author: Lucas Pallez
 *
 */

#include <can_reception.h>
#include <can_transmission.h>
#include <cmsis_os.h>
#include <code.h>
#include <common.h>
#include <console.h>
#include <FreeRTOS.h>
#include <GSE/ignition.h>
#include <led.h>
#include <main.h>
#include <projdefs.h>
#include <stm32f446xx.h>
#include <stm32f4xx_hal.h>
#include <stm32f4xx_hal_gpio.h>
#include <stm32f4xx_hal_iwdg.h>
#include <timers.h>


static uint8_t led_GSE_ignition_id;
xTimerHandle IgnTimer;

static void vTimerCallbackIgnOff(xTimerHandle pxTimer)
{
//#ifdef HB3_POWER_BOARD
//	//rocket_log("IGNITION OFF!\n");
//	HAL_GPIO_WritePin(GPIOA, GPIO_PIN_11, GPIO_PIN_RESET); //Turn off S1_D2
//	HAL_GPIO_WritePin(GPIOA, GPIO_PIN_1, GPIO_PIN_RESET); //Turn off S1_D1
	can_setFrame(MAIN_IGNITION_OFF, DATA_ID_IGNITION, HAL_GetTick());
//	can_setFrame(GPIO_PIN_RESET, DATA_ID_MAIN_IGNITION_STATE, HAL_GetTick());
//
//#endif
//#ifdef HB2_CODE_BOARD
//	//rocket_log("IGNITION SEC OFF!\n");
//	HAL_GPIO_WritePin(GPIOB, GPIO_PIN_12, GPIO_PIN_RESET); //Turn off S1_D3
//	HAL_GPIO_WritePin(GPIOA, GPIO_PIN_1, GPIO_PIN_RESET); //Turn off S1_D1
//	can_setFrame(GPIO_PIN_RESET, DATA_ID_SEC_IGNITION_STATE, HAL_GetTick());
//#endif
	if (xTimerStop(IgnTimer, 0)!=pdPASS)
		Error_Handler(); /* failure!?! */
}

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

	IgnTimer = xTimerCreate(
	      "IgnTimer", /* name */
	      pdMS_TO_TICKS(5000), /* period/time */
	      pdFALSE, /* auto reload */
	      (void*)0, /* timer ID */
	      vTimerCallbackIgnOff); /* callback */
	if (IgnTimer==NULL)
		Error_Handler();
	else{}
	rocket_log("Ignition systems initialised.\n");
}


void TK_ignition_control(void const * argument)
{

	static uint32_t ignition_order = 0;
	static uint32_t new_ignition_order = 0;

	//TODO Add sensor confirmation for main ignition and disconnect
	//TODO Add delay after ignition to shut it down automatically

	/*
	 * Pinouts:
	 * PIN11 	1.1 	HB2_S1_D3 -> Secondary Ignition		(sensed)
	 * PIN4		1.2 	HB2_S1_D1 -> Secondary Ignition
	 * PIN9 	1.3 	HB2_S1_D2 -> Secondary Disconnect
	 * PIN2 	1.4 	HB2_S1_D0 -> Secondary Disconnect	(sensed)
	 * PIN6 	2.1 	HB3_S1_D2 -> Main Ignition 			(sensed)
	 * PIN10	2.2 	HB3_S1_D1 -> Main Ignition
	 * PIN3		2.3 	HB3_S1_D3 -> Main Disconnect
	 * PIN7 	2.4 	HB3_S1_D0 -> Main Disconnect 		(sensed)
	 * PIN12 	3.1 	HB3_S2_D3 -> Main Fill Valve 		(sensed)
	 * PIN5 	3.2 	HB3_S2_D2 -> Main Fill Valve
	 * PIN8 	3.3 	HB3_S2_D1 -> Main Purge Valve
	 * PIN1 	3.4 	HB3_S2_D0 -> Main Purge Valve 		(sensed)
	 *
	 * Sens1 HB2_S1_A1
	 * Sens2 HB3_S1-A1
	 * Sens3 HB3_S2_A0
	 * Sens4 HB2_S1_A0
	 * Sens5 HB3_S1_A0
	 * Sens6 HB3_S2_A1
	 *
	 */

	 for(;;)
	 {
		 led_set_TK_rgb(led_GSE_ignition_id, 255, 117, 16);
		 ignition_order = can_getIgnitionOrder();
//		 rocket_log("Ignition Order received with payload: %d, old: %d\n", ignition_order, old_ignition_order);
		 if(new_ignition_order != ignition_order)
		 {
			 new_ignition_order = ignition_order;
		 	 switch (ignition_order)
			 {
				case MAIN_IGNITION_ON: //Main Ignition On
				{
					if((verify_security_code()) && (can_getGSTCode() != 0))
					{
#ifdef HB3_POWER_BOARD
						//rocket_log("IGNITION ON!\n");
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
						if (xTimerStart(IgnTimer, 0)!=pdPASS)
							Error_Handler(); /* failure!?! */
						else{}
					}
					else
					{
						led_set_TK_rgb(led_GSE_ignition_id, 255, 0, 0);
//						can_setFrame(0, DATA_ID_GST_CODE, HAL_GetTick());
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
					else
					{
						led_set_TK_rgb(led_GSE_ignition_id, 255, 0, 0);
//						can_setFrame(0, DATA_ID_GST_CODE, HAL_GetTick());
					}
					break;
				}
				case SECONDARY_IGNITION_OFF: //Secondary Ignition Off
				{
					//rocket_log("IGNITION SEC OFF!\n");
					HAL_GPIO_WritePin(GPIOB, GPIO_PIN_12, GPIO_PIN_RESET); //Turn off S1_D3
					HAL_GPIO_WritePin(GPIOA, GPIO_PIN_1, GPIO_PIN_RESET); //Turn off S1_D1
					can_setFrame(GPIO_PIN_RESET, DATA_ID_SEC_IGNITION_STATE, HAL_GetTick());
					break;
				}
#endif
			 }
		 }
		HAL_IWDG_Refresh(&hiwdg);
		osDelay(20);
	 }
}
