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

#define VSENSE(number) 		(number*3300.0/4096.0) //ADC unit -> mV
#define HOSE_PRESSURE		(0)
//#define HOSE_PRESSURE_CONV	(0.649*22.0/55.0) //mbar
//#define HOSE_PRESSURE_CONV	(3.238172*22.0/55.0) //mbar
#define HOSE_TEMP			(1)
#define HOSE_TEMP_CONV		(0.231*22.0/55.0) //3.958R/°C
#define TANK_TEMP			(2)
#define TANK_TEMP_CONV		(0.01) //m°C
#define WIND_SPEED			(3)
#define BATTERY_LEVEL		(4)
#define BATTERY_LEVEL_CONV	(4.9) //mV
#define ROCKET_WEIGHT		(5)
#define ROCKET_WEIGHT_CONV	(1.957/4.3) // Unit in mV/V -> Need to adjust to Vbat
#define ROCKET_WEIGHT_FS	(100) //100kg Full scale
#define ROCKET_WEIGHT_GAIN	(50) //Gain set in Hardware on the Opamp (R_G=2.05k)
#define CURRENT				(6)
#define CURRENT_CONV		(0.025) //mA ->value at 3.3/2 = 0A -> TODO Adjust on GST

#define NB_SAMPLE			16

#define IGN_MAIN			0
#define IGN_SEC				0
#define DISCONNECT_MAIN		1
#define DISCONNECT_SEC		1
#define FILL				2
#define PURGE				3

#define HOSE_PRESSURE_MAX	70000	//70 bar
#define HOSE_TEMP_MAX		100000	//100°C
#define TANK_TEMP_MAX		100000	//100°C
#define WIND_SPEED_MAX		400000	//40m/s
#define BATTERY_LEVEL_MAX	200000	//20V
#define ROCKET_WEIGHT_MAX	100000 	//100kg
#define CURRENT_MAX			500000	//50A

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
//#define KULITE_CALIB		(238) //TEST measured plugged in

uint16_t adc_buffer[ADC_BUFFER_LENGTH];

float raw_tank_temp;
uint32_t real_tank_temp;

float raw_wind_speed;
uint32_t real_wind_speed;

float raw_hose_temp;
uint32_t real_hose_temp;

float raw_hose_pressure;
uint32_t real_hose_pressure;

float raw_rocket_weight;
uint32_t real_rocket_weight;

float raw_battery_level;
uint32_t real_battery_level;

float raw_current_ign_bckp;
uint32_t real_current_ign_bckp;

float raw_current_dc_bckp;
uint32_t real_current_dc_bckp;

float raw_current_ign_main;
uint32_t real_current_ign_main;

float raw_current_dc_main;
uint32_t real_current_dc_main;

float raw_current_fill;
uint32_t real_current_fill;

float raw_current_purge;
uint32_t real_current_purge;

static uint8_t led_GSE_sensor_id;

void sensors_init(void)
{
	//Initialize sensors
	//TODO add sensor calibration
	HAL_ADC_Start_DMA(&hadc1, (uint32_t*)adc_buffer, ADC_BUFFER_LENGTH);
	led_GSE_sensor_id = led_register_TK();
	led_set_TK_rgb(led_GSE_sensor_id, 255, 0, 255);
	//kulite calibration

}


void TK_sensors_control(void const * argument)
{
//	HAL_ADC_Start(&hadc1);

	for(;;)
	{
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
		HAL_IWDG_Refresh(&hiwdg);
		osDelay(50);
	}
}
void HAL_ADC_ConvCpltCallback(ADC_HandleTypeDef* hadc)
{
	raw_hose_pressure = 0;
	raw_hose_temp = 0;
	raw_tank_temp = 0;
	raw_wind_speed = 0;
	raw_battery_level = 0;
	raw_rocket_weight = 0;

	for(uint16_t i = 0; i < ADC_BUFFER_LENGTH; i += NB_SENSOR)
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
	float debug_conv = 0;
	converted_value = VSENSE(raw_data); //Result in mV
	switch(sensor)
	{
	case HOSE_PRESSURE:
		debug_conv = converted_value;
//		converted_value = converted_value*HOSE_PRESSURE_CONV - KULITE_CALIB;
		converted_value = raw_data*31.64 - 9334;//TEST
		if(converted_value > HOSE_PRESSURE_MAX)
			converted_value = HOSE_PRESSURE_MAX;
		break;
	case HOSE_TEMP:
		converted_value /= HOSE_TEMP_CONV;
		if(converted_value > HOSE_TEMP_MAX)
			converted_value = HOSE_TEMP_MAX;
		break;
	case TANK_TEMP:
		converted_value = converted_value/TANK_TEMP_CONV - 2000; //Starting value at 2°C
		if(converted_value > TANK_TEMP_MAX)
			converted_value = TANK_TEMP_MAX;
		break;
	case WIND_SPEED:
		converted_value = (converted_value - 400)/1600*32400;
		if(converted_value > WIND_SPEED_MAX)
			converted_value = WIND_SPEED_MAX;
		break;
	case BATTERY_LEVEL:
		converted_value *= BATTERY_LEVEL_CONV;
		if(converted_value > BATTERY_LEVEL_MAX)
			converted_value = BATTERY_LEVEL_MAX;
		break;
	case ROCKET_WEIGHT:
		if(real_battery_level > 0)
			converted_value = (1000*converted_value*ROCKET_WEIGHT_FS)/(ROCKET_WEIGHT_CONV*real_battery_level*ROCKET_WEIGHT_GAIN);
		else
			converted_value = 999999;
		if(converted_value > ROCKET_WEIGHT_MAX)
			converted_value = ROCKET_WEIGHT_MAX;
		break;
	case CURRENT:
		converted_value = converted_value/CURRENT_CONV;
		if((converted_value < -CURRENT_MAX) || (converted_value > CURRENT_MAX))
			converted_value = CURRENT_MAX;
		break;
	default:
		break;
	}
	return converted_value;
}

