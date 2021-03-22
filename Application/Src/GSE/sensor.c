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
#include <debug/led.h>
#include <thread_init.h>

#define VSENSE 				(3300.0/4095.0) //For conversions
#define HOSE_PRESSURE		(1)
#define HOSE_PRESSURE_CONV	(0.0649)
#define HOSE_TEMP			(2)
#define HOSE_TEMP_CONV		(1) //TODO
#define TANK_TEMP			(3)
#define TANK_TEMP_CONV		(1) //TODO
#define WIND_SPEED			(4)
#define WIND_SPEED_CONV		(1) //TODO
#define BATTERY_LEVEL		(5)
#define BATTERY_LEVEL_CONV	(490/100)
#define ROCKET_WEIGHT		(6)
#define ROCKET_WEIGHT_CONV	(1) //TODO
#define CURRENT				(7)
#define CURRENT_CONV		(1) //TODO

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
float raw_current;
float real_current;

float raw_current_ign_bckp;
float real_current_ign_bckp;

float raw_current_dc_bckp;
float real_current_dc_bckp;

float raw_current_ign_main;
float real_current_ign_main;

float raw_current_dc_main;
float real_current_dc_main;

float raw_current_fil;
float real_current_fill;

float raw_current_purge;
float real_current_purge;

uint8_t led_GSE_sensor_id;

void sensors_init(void)
{
	//Initialize sensors
	//TODO add sensor calibration
	led_GSE_sensor_id = led_register_TK();

}


void TK_sensors_control(void const * argument)
{
	led_set_TK_rgb(led_GSE_sensor_id, 255, 0, 255);
//	HAL_ADC_Start(&hadc1);

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
		HAL_ADC_PollForConversion(&hadc1, HAL_MAX_DELAY);
#ifdef HB1_SENSOR_TELEMETRY_BOARD
		raw_hose_pressure = HAL_ADC_GetValue(&hadc1);
		raw_hose_temp = HAL_ADC_GetValue(&hadc1);
		raw_tank_temp = HAL_ADC_GetValue(&hadc1);
		raw_wind_speed = HAL_ADC_GetValue(&hadc1);
		raw_battery_level = HAL_ADC_GetValue(&hadc1);
		raw_rocket_weight = HAL_ADC_GetValue(&hadc1);
#endif
#ifdef HB2_CODE_BOARD
		raw_current_ign_bckp = HAL_ADC_GetValue(&hadc1);
		raw_current_dc_bckp = HAL_ADC_GetValue(&hadc1);
#endif
#ifdef HB3_POWER_BOARD
		raw_current_ign_main = HAL_ADC_GetValue(&hadc1);
		raw_current_dc_main = HAL_ADC_GetValue(&hadc1);
		raw_current_fill = HAL_ADC_GetValue(&hadc1);
		raw_current_purge = HAL_ADC_GetValue(&hadc1);
#endif
	    HAL_ADC_Stop(&hadc1);

#ifdef HB1_SENSOR_TELEMETRY_BOARD
	    real_hose_pressure = sensor_conversion(HOSE_PRESSURE, raw_hose_pressure);
		can_setFrame(real_hose_pressure, DATA_ID_IN_HOSE_PRESSURE, HAL_GetTick());

	    real_hose_temp = sensor_conversion(HOSE_TEMP, raw_hose_temp);
		can_setFrame(real_hose_temp, DATA_ID_IN_HOSE_TEMPERATURE, HAL_GetTick());

	    real_tank_temp = sensor_conversion(TANK_TEMP, raw_tank_temp);
		can_setFrame(real_tank_temp, DATA_ID_TANK_TEMPERATURE, HAL_GetTick());

	    real_wind_speed = sensor_conversion(WIND_SPEED, raw_wind_speed);
		can_setFrame(real_wind_speed, DATA_ID_WIND_SPEED, HAL_GetTick());

	    real_battery_level = sensor_conversion(BATTERY_LEVEL, raw_battery_level);
		can_setFrame(real_battery_level, DATA_ID_BATTERY_LEVEL, HAL_GetTick());

	    real_rocket_weight = sensor_conversion(ROCKET_WEIGHT, raw_rocket_weight);
		can_setFrame(real_rocket_weight, DATA_ID_ROCKET_WEIGHT, HAL_GetTick());

#endif
#ifdef HB2_CODE_BOARD
	    real_current_ign_bckp = sensor_conversion(CURRENT, raw_current_ign_bckp);
		can_setFrame(real_current_ign_bckp, DATA_ID_IGNITION_CURRENT_2, HAL_GetTick());

	    real_current_dc_bckp = sensor_conversion(CURRENT, raw_current_dc_bckp);
		can_setFrame(real_current_dc_bckp, DATA_ID_DISCONNECT_CURRENT_2, HAL_GetTick());

#endif
#ifdef HB3_POWER_BOARD
		real_current_ign_main = sensor_conversion(CURRENT, raw_current_ign_main);
		can_setFrame(real_current_ign_main, DATA_ID_IGNITION_CURRENT_1, HAL_GetTick());

		real_current_dc_main = sensor_conversion(CURRENT, raw_current_dc_main);
		can_setFrame(real_current_dc_main, DATA_ID_DISCONNECT_CURRENT_1, HAL_GetTick());

		real_current_fill = sensor_conversion(CURRENT, raw_current_fill);
		can_setFrame(real_current_fill, DATA_ID_FILL_VALVE_CURRENT, HAL_GetTick());

		real_current_purge = sensor_conversion(CURRENT, raw_current_purge);
		can_setFrame(real_current_purge, DATA_ID_PURGE_VALVE_CURRENT, HAL_GetTick());

#endif

		osDelay(100);
	}
}

float sensor_conversion(uint8_t sensor, float raw_data)
{
	float converted_value = 0;
	converted_value = raw_data*VSENSE;
	switch(sensor)
	{
	case HOSE_PRESSURE:
		converted_value /= HOSE_PRESSURE_CONV;
		break;
	case HOSE_TEMP:
		converted_value /= HOSE_TEMP_CONV;
		break;
	case TANK_TEMP:
		converted_value /= TANK_TEMP_CONV;
		break;
	case WIND_SPEED:
		converted_value /= WIND_SPEED_CONV;
		break;
	case BATTERY_LEVEL:
		converted_value /= BATTERY_LEVEL_CONV;
		break;
	case ROCKET_WEIGHT:
		converted_value /= ROCKET_WEIGHT_CONV;
		break;
	case CURRENT:
		converted_value /= CURRENT_CONV;
		break;
	default:
		break;
	}
	return converted_value;
}

