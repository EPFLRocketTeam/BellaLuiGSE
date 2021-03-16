/*
 * sensor_telemetry.c
 *
 *  Created on: 21 Feb 2021
 *      Author: Lucas Pallez
 *
 */

#include <can_transmission.h>
#include <cmsis_os.h>
#include <main.h>
#include <stm32f4xx_hal.h>
#include <stm32f4xx_hal_adc.h>
#include <stm32f4xx_hal_def.h>
#include "GSE/sensor.h"


float raw_tank_temp;
float real_tank_temp;

float raw_wind_speed;
float real_wind_speed;

float raw_hose_temp;
float real_hose_temp;

float raw_hose_pressure;
float real_hose_pressure;

float raw_rocket_weight;
float real_rocket_weight;

float raw_battery_level;
float real_battery_level;
//
float raw_current = 0;
float real_current = 0;

float raw_current_ign_bckp = 0;
float real_current_ign_bckp = 0;

float raw_current_dc_bckp = 0;
float real_current_dc_bckp = 0;

float raw_current_ign_main = 0;
float real_current_ign_main = 0;

float raw_current_dc_main = 0;
float real_current_dc_main = 0;

float raw_current_fill = 0;
float real_current_fill = 0;

float raw_current_purge = 0;
float real_current_purge = 0;



void sensors_init(void)
{
	//Initialize sensors
	//TODO add sensor calibration
}


void TK_sensors_control(void const * argument)
{

	for(;;)
	{


		//read_tank_temp();
		//read_anemo();
		//read_hose_temp(); // does it work ?
		//read_hose_pressure();
		//read_load_cell();
		//read_battery_level();
		//read_current_level();

		HAL_ADC_Start(&hadc1);
#ifdef HB2_SENSOR_TELEMETRY_BOARD
		raw_tank_temp = HAL_ADC_GetValue(&hadc1);
		raw_wind_speed = HAL_ADC_GetValue(&hadc1);
		raw_hose_temp = HAL_ADC_GetValue(&hadc1);
		raw_hose_pressure = HAL_ADC_GetValue(&hadc1);
		raw_rocket_weight = HAL_ADC_GetValue(&hadc1);
		raw_battery_level = HAL_ADC_GetValue(&hadc1);
#endif
#ifdef HB1_CODE_BOARD
		raw_current_ign_bckp = HAL_ADC_GetValue(&hadc1);
		raw_current_dc_bckp = HAL_ADC_GetValue(&hadc1);
#endif
#ifdef HB3_POWER_BOARD
		raw_current_ign_main = HAL_ADC_GetValue(&hadc1);
		raw_current_dc_main = HAL_ADC_GetValue(&hadc1);
		raw_current_fill = HAL_ADC_GetValue(&hadc1);
		raw_current_purge = HAL_ADC_GetValue(&hadc1);
#endif
		HAL_ADC_PollForConversion(&hadc1, HAL_MAX_DELAY);
	    HAL_ADC_Stop(&hadc1);

		osDelay(100);
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

	real_tank_temp = (float) raw_tank_temp*(3300.0/4095.0);
	//result in mV or .1°C
	//Conversion from mV to 0.1°C
	//real_tank_temp = raw_tank_temp*(1.5*4095.0)/3.3;
	// (VALUE (mV) - min (mV))/max (mV) * max (UNIT)

	can_setFrame(real_tank_temp, DATA_ID_TANK_TEMPERATURE, HAL_GetTick());
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
	real_wind_speed = ((float) real_wind_speed - 40.0 )/ 160.0*32.4; //result in 0.1m/s
	// (VALUE (mV) - min (mV))/max (mV) * max (UNIT)

	can_setFrame(real_wind_speed, DATA_ID_WIND_SPEED, HAL_GetTick());
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
	// (VALUE (mV) - min (mV))/max (mV) * max (UNIT)

	can_setFrame(real_hose_temp, DATA_ID_IN_HOSE_TEMPERATURE, HAL_GetTick());
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
	// (VALUE (mV) - min (mV))/max (mV) * max (UNIT)

	can_setFrame(real_hose_pressure, DATA_ID_IN_HOSE_PRESSURE, HAL_GetTick());
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
	// (VALUE (mV) - min (mV))/max (mV) * max (UNIT)

	can_setFrame(real_rocket_weight, DATA_ID_ROCKET_WEIGHT, HAL_GetTick());
}

void read_battery_level(void)
{
	/*
	 * UNITS / RATE
	 * Min - Max values
	 *
	 */

	// Get ADC value
	HAL_ADC_Start(&hadc1);
	HAL_ADC_PollForConversion(&hadc1, HAL_MAX_DELAY);

	raw_battery_level = HAL_ADC_GetValue(&hadc1);

	real_battery_level = 3300*((float) raw_battery_level/4095.0); //result in mV

	can_setFrame(real_battery_level, DATA_ID_BATTERY_LEVEL, HAL_GetTick());
}

void get_current_level(void)
{
	/*
	 * UNITS / RATE
	 * Min - Max values
	 *
	 */

	// Get ADC value
	HAL_ADC_Start(&hadc1);
	HAL_ADC_PollForConversion(&hadc1, HAL_MAX_DELAY);
	raw_current = HAL_ADC_GetValue(&hadc1);

	real_current = 3300*((float) raw_current/4095.0); //result in mV

	//TODO Specify distribution method

	//#ifdef HB1_CODE_BOARD
	//		can_setFrame(current, DATA_ID_IGNITION_CURRENT_1, HAL_GetTick());
	//		can_setFrame(current, DATA_ID_DISCONNECT_CURRENT_1, HAL_GetTick());
	//#endif
	//#ifdef HB3_POWER_BOARD
	//		can_setFrame(current, DATA_ID_IGNITION_CURRENT_2, HAL_GetTick());
	//		can_setFrame(current, DATA_ID_DISCONNECT_CURRENT_2, HAL_GetTick());
	//		can_setFrame(current, DATA_ID_FILL_VALVE_CURRENT, HAL_GetTick());
	//		can_setFrame(current, DATA_ID_PURGE_VALVE_CURRENT, HAL_GetTick());
	//#endif

}
