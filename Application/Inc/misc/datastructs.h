/*
 * datastructs.h
 *
 *  Created on: 5 Apr 2018
 *      Author: Cl�ment Nussbaumer
 */

#ifndef INCLUDE_DATASTRUCTS_H_
#define INCLUDE_DATASTRUCTS_H_

#ifdef __cplusplus
 extern "C" {
#endif

#include <stdint.h>

typedef float float32_t;
typedef double float64_t;

typedef struct
{
	uint8_t fill_valve_state;
	uint8_t purge_valve_state;
	uint8_t main_ignition_state;
	uint8_t sec_ignition_state;
	uint8_t hose_disconnect_state;
	uint32_t code;	//NOT BEING SENT TO GST
	float battery_level;
	float hose_pressure;
	float hose_temperature;
	float tank_temperature;
	float rocket_weight;
	float ignition1_current;
	float ignition2_current;
	float wind_speed;
}GSE_state;
typedef struct
{
  void* ptr;
  uint16_t size;
} Telemetry_Message;

typedef struct
{
  void* ptr;
  uint16_t size;
} String_Message;


#ifdef __cplusplus
 }
#endif

#endif /* INCLUDE_DATASTRUCTS_H_ */
