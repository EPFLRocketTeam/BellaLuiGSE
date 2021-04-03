/*
 * valve.c
 *
 *  Created on: 21 Feb 2021
 *      Author: Lucas Pallez
 *
 */

#include <can_reception.h>
#include <can_transmission.h>
#include <cmsis_os.h>
#include <common.h>
#include <console.h>
#include <ignition.h>
#include <led.h>
#include <order.h>
#include <code.h>
#include <stm32f446xx.h>
#include <stm32f4xx_hal.h>
#include <stm32f4xx_hal_gpio.h>
#include <sys/_stdint.h>

static uint8_t led_GSE_order_id;

void order_sys_init(void)
{
#ifdef HB3_POWER_BOARD
	//Set all S2 valves to low
	HAL_GPIO_WritePin(GPIOA, GPIO_PIN_15, GPIO_PIN_RESET); 	//D0
	HAL_GPIO_WritePin(GPIOB, GPIO_PIN_9, GPIO_PIN_RESET);	//D1
	HAL_GPIO_WritePin(GPIOB, GPIO_PIN_7, GPIO_PIN_RESET);	//D2
	HAL_GPIO_WritePin(GPIOC, GPIO_PIN_13, GPIO_PIN_RESET);	//D3
#endif

	rocket_log("Plumbing system initialised.\n");
	led_GSE_order_id = led_register_TK();
	led_set_TK_rgb(led_GSE_order_id, 60, 180, 230);

}

void TK_order_control(void const * argument)
{

	static uint32_t order = 0;

	//TODO Add sensor confirmation
	 for(;;) {
		 order = can_getOrder();
			 switch (order)
			{
				case OPEN_FILL_VALVE:
				{
					//rocket_log("Open Fill\n");
					HAL_GPIO_WritePin(GPIOC, GPIO_PIN_13, GPIO_PIN_SET);
					HAL_GPIO_WritePin(GPIOB, GPIO_PIN_7, GPIO_PIN_SET);

					can_setFrame(GPIO_PIN_SET, DATA_ID_FILL_VALVE_STATE, HAL_GetTick());
					break;
				}
				case CLOSE_FILL_VALVE:
				{
					//rocket_log("Close Fill\n");
					HAL_GPIO_WritePin(GPIOC, GPIO_PIN_13, GPIO_PIN_RESET);
					HAL_GPIO_WritePin(GPIOB, GPIO_PIN_7, GPIO_PIN_RESET);

					can_setFrame(GPIO_PIN_RESET, DATA_ID_FILL_VALVE_STATE, HAL_GetTick());
					break;
				}
				case OPEN_PURGE_VALVE:
				{
					//rocket_log("Open Purge\n");
					HAL_GPIO_WritePin(GPIOA, GPIO_PIN_15, GPIO_PIN_SET);
					HAL_GPIO_WritePin(GPIOB, GPIO_PIN_9, GPIO_PIN_SET);

					can_setFrame(GPIO_PIN_SET, DATA_ID_PURGE_VALVE_STATE, HAL_GetTick());
					break;
				}
				case CLOSE_PURGE_VALVE:
				{
					//rocket_log("Close Purge\n");
					HAL_GPIO_WritePin(GPIOA, GPIO_PIN_15, GPIO_PIN_RESET);
					HAL_GPIO_WritePin(GPIOB, GPIO_PIN_9, GPIO_PIN_RESET);

					can_setFrame(GPIO_PIN_RESET, DATA_ID_PURGE_VALVE_STATE, HAL_GetTick());
					break;
				}
				case MAIN_DISCONNECT_ON: //Main Disconnect On
				{
					if(verify_security_code())
					{
#ifdef HB3_POWER_BOARD
						//rocket_log("MAIN DISCONNECT ON!\n");
						HAL_GPIO_WritePin(GPIOB, GPIO_PIN_12, GPIO_PIN_SET); //Turn on S1_D3
						HAL_GPIO_WritePin(GPIOA, GPIO_PIN_0, GPIO_PIN_SET); //Turn on S1_D0
						can_setFrame(GPIO_PIN_SET, DATA_ID_MAIN_DISCONNECT_STATE, HAL_GetTick());
#endif
#ifdef HB2_CODE_BOARD
						//rocket_log("SEC DISCONNECT ON!\n");
						HAL_GPIO_WritePin(GPIOA, GPIO_PIN_11, GPIO_PIN_SET); //Turn off S1_D2
						HAL_GPIO_WritePin(GPIOA, GPIO_PIN_0, GPIO_PIN_SET); //Turn off S1_D0
						can_setFrame(GPIO_PIN_SET, DATA_ID_SEC_DISCONNECT_STATE, HAL_GetTick());

#endif
					}
					else
					{
						led_set_TK_rgb(led_GSE_order_id, 255, 0, 0);
//						can_setFrame(0, DATA_ID_GST_CODE, HAL_GetTick());
					}
					break;
				}
				case MAIN_DISCONNECT_OFF: //Main Disconnect Off
				{
#ifdef HB3_POWER_BOARD
					//rocket_log("MAIN DISCONNECT OFF!\n");
					HAL_GPIO_WritePin(GPIOB, GPIO_PIN_12, GPIO_PIN_RESET); //Turn off S1_D3
					HAL_GPIO_WritePin(GPIOA, GPIO_PIN_0, GPIO_PIN_RESET); //Turn off S1_D0
					can_setFrame(GPIO_PIN_RESET, DATA_ID_MAIN_DISCONNECT_STATE, HAL_GetTick());
#endif
#ifdef HB2_CODE_BOARD
					//rocket_log("SEC DISCONNECT OFF!\n");
					HAL_GPIO_WritePin(GPIOA, GPIO_PIN_11, GPIO_PIN_RESET); //Turn off S1_D2
					HAL_GPIO_WritePin(GPIOA, GPIO_PIN_0, GPIO_PIN_RESET); //Turn off S1_D0
					can_setFrame(GPIO_PIN_RESET, DATA_ID_SEC_DISCONNECT_STATE, HAL_GetTick());
#endif
					break;

				}
#ifdef HB2_CODE_BOARD
				case SECONDARY_DISCONNECT_ON: //Secondary Disconnect On
				{
					if(verify_security_code())
					{
						//rocket_log("SEC DISCONNECT ON!\n");
						HAL_GPIO_WritePin(GPIOA, GPIO_PIN_11, GPIO_PIN_SET); //Turn on S1_D2
						HAL_GPIO_WritePin(GPIOA, GPIO_PIN_0, GPIO_PIN_SET); //Turn on S1_D0
						can_setFrame(GPIO_PIN_SET, DATA_ID_SEC_DISCONNECT_STATE, HAL_GetTick());
					}
					else
					{
						led_set_TK_rgb(led_GSE_order_id, 255, 0, 0);
//						can_setFrame(0, DATA_ID_GST_CODE, HAL_GetTick());
					}
					break;
				}
				case SECONDARY_DISCONNECT_OFF: //Secondary Ignition Off
				{
					//rocket_log("SEC DISCONNECT OFF!\n");
					HAL_GPIO_WritePin(GPIOA, GPIO_PIN_11, GPIO_PIN_RESET); //Turn off S1_D2
					HAL_GPIO_WritePin(GPIOA, GPIO_PIN_0, GPIO_PIN_RESET); //Turn off S1_D0
					can_setFrame(GPIO_PIN_RESET, DATA_ID_SEC_DISCONNECT_STATE, HAL_GetTick());
					break;
				}
#endif
			}
		 HAL_IWDG_Refresh(&hiwdg);
		 osDelay(20);
	 }
}

