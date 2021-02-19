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
#include "GSE/sensor_telemetry.h"


uint16_t raw_tank_temp;
uint16_t real_tank_temp;

uint16_t raw_wind_speed;
uint16_t real_wind_speed;

uint16_t raw_hose_temp;
uint16_t real_hose_temp;

uint16_t raw_hose_pressure;
uint16_t real_hose_pressure;

uint16_t raw_rocket_weight;
uint16_t real_rocket_weight;

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

	for(;;)
	{

		//read_tank_temp();
		read_anemo();
		//read_hose_temp();
		//read_hose_pressure();
		//read_load_cell();

		osDelay(1000);
	}
}

void read_tank_temp(void)
{
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
	//Conversion from mV to 0.1°C

	can_setFrame(real_tank_temp, DATA_ID_TANK_TEMPERATURE, HAL_GetTick());
	rocket_boot_log("RAW Tank Temperature: %d \n", raw_tank_temp);
	rocket_boot_log("Tank Temperature: %d \n", real_tank_temp);
}
void read_anemo(void)
{

	/*
	 * Output: 0.4V - 2V
	 * Min: 0.2 m/s
	 * Max: 32.4 m/s
	 */

	// Get ADC value
	HAL_ADC_Start(&hadc1);
	HAL_ADC_PollForConversion(&hadc1, HAL_MAX_DELAY);
	raw_wind_speed = HAL_ADC_GetValue(&hadc1);

	real_wind_speed = 330*((float) raw_wind_speed/4095.0); //result in 10mV
	real_wind_speed = ((float) real_wind_speed - 400.0 )/ 1600.0*32.4; //result in 0.1m/s

	can_setFrame(real_wind_speed, DATA_ID_WIND_SPEED, HAL_GetTick());
	rocket_boot_log("RAW Wind Speed: %d \n", raw_wind_speed);
	rocket_boot_log("Wind Speed: %d \n", real_wind_speed);
}
void read_hose_temp(void)
{

	/*
	 * UNITS / RATE
	 * Min - Max values
	 *
	 */

	// Get ADC value
	HAL_ADC_Start(&hadc1);
	HAL_ADC_PollForConversion(&hadc1, HAL_MAX_DELAY);
	raw_hose_temp = HAL_ADC_GetValue(&hadc1);

	real_hose_temp = 3300*((float) raw_hose_temp/4095.0); //result in mV
	//Conversion from mV to 0.1°C

	can_setFrame(real_hose_temp, DATA_ID_IN_HOSE_TEMPERATURE, HAL_GetTick());
	rocket_boot_log("RAW Hose Temperature: %d \n", raw_hose_temp);
	rocket_boot_log("Hose Temperature: %d \n", real_hose_temp);
}
void read_hose_pressure(void)
{

	/*
	 * UNITS / RATE
	 * Min - Max values
	 *
	 */

	// Get ADC value
	HAL_ADC_Start(&hadc1);
	HAL_ADC_PollForConversion(&hadc1, HAL_MAX_DELAY);
	raw_hose_pressure = HAL_ADC_GetValue(&hadc1);

	real_hose_pressure = 3300*((float) raw_hose_pressure/4095.0); //result in mV
	//Conversion from mV to 0.1bar

	can_setFrame(real_hose_pressure, DATA_ID_IN_HOSE_PRESSURE, HAL_GetTick());
	rocket_boot_log("RAW Hose Pressure: %d \n", raw_hose_pressure);
	rocket_boot_log("Tank Pressure: %d \n", real_hose_pressure);
}
void read_load_cell(void)
{
	/*
	 * UNITS / RATE
	 * Min - Max values
	 *
	 */

	// Get ADC value
	HAL_ADC_Start(&hadc1);
	HAL_ADC_PollForConversion(&hadc1, HAL_MAX_DELAY);
	raw_rocket_weight = HAL_ADC_GetValue(&hadc1);

	real_rocket_weight = 3300*((float) raw_rocket_weight/4095.0); //result in mV
	//Conversion from mV to 0.1kg

	can_setFrame(real_rocket_weight, DATA_ID_ROCKET_WEIGHT, HAL_GetTick());
	rocket_boot_log("RAW Rocket Weight: %d \n", raw_rocket_weight);
	rocket_boot_log("Rocket Weight: %d \n", real_rocket_weight);
}
