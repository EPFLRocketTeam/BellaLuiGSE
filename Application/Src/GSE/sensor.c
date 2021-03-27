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
#define HOSE_PRESSURE		(0)
#define HOSE_PRESSURE_CONV	(0.639*22.0/55.0) //mbar
#define HOSE_TEMP			(1)
#define HOSE_TEMP_CONV		(1) //3.958R/°C
#define TANK_TEMP			(2)
#define TANK_TEMP_CONV		(0.01) //m°C
#define WIND_SPEED			(3)
#define WIND_SPEED_CONV		(1) //Not used
#define BATTERY_LEVEL		(4)
#define BATTERY_LEVEL_CONV	(0.054) //mV
#define ROCKET_WEIGHT		(5)
#define ROCKET_WEIGHT_CONV	(1) //TODO
#define CURRENT				(6)
#define CURRENT_CONV		(0.025) //mA ->value at 3.3/2 = 0A -> TODO Adjust on GST

#define NB_SAMPLE			64

#define IGN_MAIN			0
#define IGN_SEC				0
#define DISCONNECT_MAIN		1
#define DISCONNECT_SEC		1
#define FILL				2
#define PURGE				3


#ifdef HB1_SENSOR_TELEMETRY_BOARD
#define NB_SENSOR 			6
#endif

#ifdef HB2_CODE_BOARD
#define NB_SENSOR 			2
#endif

#ifdef HB3_POWER_BOARD
#define NB_SENSOR 			4
#endif

#define ADC_BUFFER_LENGTH	(NB_SENSOR*NB_SAMPLE)
#define KULITE_CALIB		(512.7008) //Calibrated

uint16_t adc_buffer[ADC_BUFFER_LENGTH];

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

float raw_current_fill;
float real_current_fill;

float raw_current_purge;
float real_current_purge;

uint8_t led_GSE_sensor_id;

void sensors_init(void)
{
	//Initialize sensors
	//TODO add sensor calibration
	HAL_ADC_Start_DMA(&hadc1, (uint32_t*)adc_buffer, ADC_BUFFER_LENGTH);
	led_GSE_sensor_id = led_register_TK();

	//kulite calibration


}


void TK_sensors_control(void const * argument)
{
	led_set_TK_rgb(led_GSE_sensor_id, 255, 0, 255);
//	HAL_ADC_Start(&hadc1);

	for(;;)
	{
//		HAL_ADC_Start(&hadc1);
//		HAL_ADC_PollForConversion(&hadc1, HAL_MAX_DELAY);
//#ifdef HB1_SENSOR_TELEMETRY_BOARD
//		raw_hose_pressure = HAL_ADC_GetValue(&hadc1);
//		raw_hose_temp = HAL_ADC_GetValue(&hadc1);
//		raw_tank_temp = HAL_ADC_GetValue(&hadc1);
//		raw_wind_speed = HAL_ADC_GetValue(&hadc1);
//		raw_battery_level = HAL_ADC_GetValue(&hadc1);
//		raw_rocket_weight = HAL_ADC_GetValue(&hadc1);
//#endif
//#ifdef HB2_CODE_BOARD
//		raw_current_ign_bckp = HAL_ADC_GetValue(&hadc1);
//		raw_current_dc_bckp = HAL_ADC_GetValue(&hadc1);
//#endif
//#ifdef HB3_POWER_BOARD
//		raw_current_ign_main = HAL_ADC_GetValue(&hadc1);
//		raw_current_dc_main = HAL_ADC_GetValue(&hadc1);
//		raw_current_fill = HAL_ADC_GetValue(&hadc1);
//		raw_current_purge = HAL_ADC_GetValue(&hadc1);
//#endif
//	    HAL_ADC_Stop(&hadc1);

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

		osDelay(10);
	}
}
void HAL_ADC_ConvCpltCallback(ADC_HandleTypeDef* hadc)
{
	for(uint16_t i = 0; i < NB_SAMPLE; i++)
	{
#ifdef HB1_SENSOR_TELEMETRY_BOARD
		raw_hose_pressure += adc_buffer[i+HOSE_PRESSURE];
		raw_hose_temp += adc_buffer[i+HOSE_TEMP];
		raw_tank_temp += adc_buffer[i+TANK_TEMP];
		raw_wind_speed += adc_buffer[i+WIND_SPEED];
		raw_battery_level += adc_buffer[i+BATTERY_LEVEL];
		raw_rocket_weight += adc_buffer[i+ROCKET_WEIGHT];
#endif
#ifdef HB2_CODE_BOARD
		raw_current_ign_bckp += adc_buffer[i+IGN_SEC];
		raw_current_dc_bckp += adc_buffer[i+DISCONNECT_SEC];
#endif
#ifdef HB3_POWER_BOARD
		raw_current_ign_main += adc_buffer[i+IGN_MAIN];
		raw_current_dc_main += adc_buffer[i+DISCONNECT_MAIN];
		raw_current_fill += adc_buffer[i+FILL];
		raw_current_purge += adc_buffer[i+PURGE];
#endif
	}

#ifdef HB1_SENSOR_TELEMETRY_BOARD
	raw_hose_pressure /= NB_SAMPLE;
	raw_hose_temp /= NB_SAMPLE;
	raw_tank_temp /= NB_SAMPLE;
	raw_wind_speed /= NB_SAMPLE;
	raw_battery_level /= NB_SAMPLE;
	raw_rocket_weight /= NB_SAMPLE;
#endif
#ifdef HB2_CODE_BOARD
	raw_current_ign_bckp /= NB_SAMPLE;
	raw_current_dc_bckp /= NB_SAMPLE;
#endif
#ifdef HB3_POWER_BOARD
	raw_current_ign_main /= NB_SAMPLE;
	raw_current_dc_main /= NB_SAMPLE;
	raw_current_fill /= NB_SAMPLE;
	raw_current_purge /= NB_SAMPLE;
#endif

}
float sensor_conversion(uint8_t sensor, float raw_data)
{
	float converted_value = 0;
	converted_value = raw_data*VSENSE; //Result in mV
	switch(sensor)
	{
	case HOSE_PRESSURE:
		converted_value = (converted_value - KULITE_CALIB)/HOSE_PRESSURE_CONV;
		break;
	case HOSE_TEMP:
		converted_value /= HOSE_TEMP_CONV;
		break;
	case TANK_TEMP:
		converted_value = converted_value/TANK_TEMP_CONV -2000; //Starting value at 2°C
		break;
	case WIND_SPEED:
		converted_value = (converted_value - 400)/1600*32.4;
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

