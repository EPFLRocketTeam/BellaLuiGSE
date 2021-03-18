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
#include <GSE/valve.h>
#include <GSE/ignition.h>
#include <led.h>
#include <stm32f446xx.h>
#include <stm32f4xx_hal.h>
#include <stm32f4xx_hal_gpio.h>
#include <sys/_stdint.h>

uint8_t led_GSE_valve_id;

void valve_init(void)
{
#ifdef HB3_POWER_BOARD
	//Set all S2 valves to low
	HAL_GPIO_WritePin(GPIOA, GPIO_PIN_15, GPIO_PIN_RESET);
	HAL_GPIO_WritePin(GPIOB, GPIO_PIN_9, GPIO_PIN_RESET);
	HAL_GPIO_WritePin(GPIOB, GPIO_PIN_7, GPIO_PIN_RESET);
	HAL_GPIO_WritePin(GPIOC, GPIO_PIN_13, GPIO_PIN_RESET);
#endif

	rocket_log("Plumbing system initialised.\n");
	led_GSE_valve_id = led_register_TK();
	led_set_rgb(20,170,245);

}
void TK_GSE_valve_control(void const * argument)
{
	//if xbee code = purge (D0)
	//if xbee code = purge backup (D1)
	//if xbee code = fill (D2)
	//if xbee code = fill backup (D3)

	static uint32_t valve_order = 0;
	static uint32_t old_valve_order = 0;
	led_set_TK_rgb(led_GSE_valve_id, 60, 180, 230);

	//TODO Add sensor confirmation
	 for(;;) {
		 valve_order = can_getOrder();
		 if(valve_order != old_valve_order)
		 {
			 old_valve_order = valve_order;
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
		 }
		 osDelay(50);
	 }
}

