/*
 * sensor_telemetry.c
 *
 *  Created on: 21 Feb 2021
 *      Author: Lucas Pallez
 *
 */

#include "main.h"
#include "can_transmission.h"
#include "debug/console.h"

void telemetry_init(void)
{
	//Initialise telemetry

}
void sensors_init(void)
{
	//Initialize sensors
}

void TK_telemetry_control(void const * argument)
{
	//Order status, check each state and send appropriate data
	for(;;)
	{
		osDelay(50);
	}
}
void TK_sensors_control(void const * argument)
{

	uint16_t raw_tank_temp;
	uint16_t real_tank_temp;

	for(;;)
	{
//		float tank_temp = 0;
//		float hose_pressure = 0;
//		float host_temp = 0;

		/*
		 * 10mV/°C
		 * 2°C = 0mV
		 * 102°C = 1V
		 *
		 */

		// Get ADC value
		HAL_ADC_Start(&hadc1);
		HAL_ADC_PollForConversion(&hadc1, HAL_MAX_DELAY);
		raw_tank_temp = HAL_ADC_GetValue(&hadc1);

		real_tank_temp = 3300*((float) raw_tank_temp/4095.0); //result in mV or .1°C --> WRONG, something is off


		can_setFrame(real_tank_temp, DATA_ID_TANK_TEMPERATURE, HAL_GetTick());
		rocket_boot_log("RAW Tank Temperature: %d \n", raw_tank_temp);
		rocket_boot_log("Tank Temperature: %d \n", real_tank_temp);
		osDelay(1000);
	}
}
