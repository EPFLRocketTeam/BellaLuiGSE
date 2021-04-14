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
	uint8_t main_disconnect_state;
	uint8_t sec_disconnect_state;
	uint8_t code;	//NOT BEING SENT TO GST
	uint32_t battery_level;
	uint32_t hose_pressure;
	uint32_t hose_temperature;
	uint32_t tank_temperature;
	uint32_t rocket_weight;
	uint32_t ignition1_current;
	uint32_t ignition2_current;
	uint32_t disconnect1_current;
	uint32_t disconnect2_current;
	uint32_t fill_valve_current;
	uint32_t purge_valve_current;
	uint32_t wind_speed;
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
