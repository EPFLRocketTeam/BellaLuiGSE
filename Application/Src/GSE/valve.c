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
#include <debug/led.h>
#include <GSE/valve.h>
#include <misc/common.h>
#include <stm32_hal_legacy.h>
#include <stm32f446xx.h>
#include <stm32f4xx_hal.h>
#include <stm32f4xx_hal_gpio.h>
#include <sys/_stdint.h>


void valve_init(void)
{

//	//Set all S2 valves to low
//	HAL_GPIO_WritePin(GPIOA, GPIO_PIN_15, GPIO_PIN_RESET);
//	HAL_GPIO_WritePin(GPIOB, GPIO_PIN_9, GPIO_PIN_RESET);
//	HAL_GPIO_WritePin(GPIOB, GPIO_PIN_7, GPIO_PIN_RESET);
//	HAL_GPIO_WritePin(GPIOC, GPIO_PIN_13, GPIO_PIN_RESET);

	//Set all S1 valves to low
//	HAL_GPIO_WritePin(GPIOA, GPIO_PIN_0, GPIO_PIN_RESET);
//	HAL_GPIO_WritePin(GPIOA, GPIO_PIN_1, GPIO_PIN_RESET);
//	HAL_GPIO_WritePin(GPIOA, GPIO_PIN_11, GPIO_PIN_RESET);
//	HAL_GPIO_WritePin(GPIOB, GPIO_PIN_12, GPIO_PIN_RESET);

	//led_set_rgb(0, 0, 255);

}
void TK_GSE_valve_control(void const * argument)
{
	//if xbee code = purge (D0)
	//if xbee code = purge backup (D1)
	//if xbee code = fill (D2)
	//if xbee code = fill backup (D3)

	static uint32_t valve_order = 0;
	static uint32_t old_valve_order = 0;

	//TODO Add sensor confirmation
	 for(;;) {
		 valve_order = can_getOrder();
		 if(valve_order != old_valve_order)
			 switch (valve_order)
				{
					case OPEN_FILL_VALVE:
					{
						HAL_GPIO_WritePin(GPIOA, GPIO_PIN_0, GPIO_PIN_SET);
						HAL_GPIO_WritePin(GPIOA, GPIO_PIN_1, GPIO_PIN_SET);

						can_setFrame(GPIO_PIN_SET, DATA_ID_FILL_VALVE_STATE, HAL_GetTick());
						break;
					}
					case CLOSE_FILL_VALVE:
					{
						HAL_GPIO_WritePin(GPIOA, GPIO_PIN_0, GPIO_PIN_RESET);
						HAL_GPIO_WritePin(GPIOA, GPIO_PIN_1, GPIO_PIN_RESET);

						can_setFrame(GPIO_PIN_RESET, DATA_ID_FILL_VALVE_STATE, HAL_GetTick());
						break;
					}
					case OPEN_PURGE_VALVE:
					{
						HAL_GPIO_WritePin(GPIOA, GPIO_PIN_11, GPIO_PIN_SET);
						HAL_GPIO_WritePin(GPIOB, GPIO_PIN_12, GPIO_PIN_SET);

						can_setFrame(GPIO_PIN_SET, DATA_ID_PURGE_VALVE_STATE, HAL_GetTick());
						break;
					}
					case CLOSE_PURGE_VALVE:
					{
						HAL_GPIO_WritePin(GPIOA, GPIO_PIN_11, GPIO_PIN_RESET);
						HAL_GPIO_WritePin(GPIOB, GPIO_PIN_12, GPIO_PIN_RESET);

						can_setFrame(GPIO_PIN_RESET, DATA_ID_PURGE_VALVE_STATE, HAL_GetTick());
						break;
					}
//					case OPEN_FILL_VALVE_BACKUP:
//					{
//						HAL_GPIO_WritePin(GPIOC, GPIO_PIN_13, GPIO_PIN_RESET);
//
//						can_setFrame(GPIO_PIN_RESET, DATA_ID_FILL_VALVE_STATE, HAL_GetTick());
//						break;
//					}
//					case CLOSE_FILL_VALVE_BACKUP:
//					{
//						HAL_GPIO_WritePin(GPIOC, GPIO_PIN_13, GPIO_PIN_SET);
//
//						can_setFrame(GPIO_PIN_SET, DATA_ID_FILL_VALVE_STATE, HAL_GetTick());
//						break;
//					}
//					case OPEN_PURGE_VALVE_BACKUP:
//					{
//						HAL_GPIO_WritePin(GPIOB, GPIO_PIN_9, GPIO_PIN_RESET);
//
//						can_setFrame(GPIO_PIN_RESET, DATA_ID_PURGE_VALVE_STATE, HAL_GetTick());
//						break;
//					}
//					case CLOSE_PURGE_VALVE_BACKUP:
//					{
//						HAL_GPIO_WritePin(GPIOB, GPIO_PIN_9, GPIO_PIN_SET);
//
//						can_setFrame(GPIO_PIN_SET, DATA_ID_PURGE_VALVE_STATE, HAL_GetTick());
//						break;
//					}

				}
		 //TODO Add current sensor reading
		 osDelay(50);
	 }
}
