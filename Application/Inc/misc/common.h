/*
 * Common.h
 *
 *  Created on: 4 Apr 2018
 *      Author: Cl�ment Nussbaumer
 */

#ifndef INCLUDE_COMMON_H_
#define INCLUDE_COMMON_H_

#ifdef __cplusplus
extern "C" {
#endif

#include <misc/datastructs.h>
#include <stm32f4xx_hal.h>

#define CIRC_BUFFER_SIZE 8

#define IGNITION_CODE 9

extern TIM_HandleTypeDef htim7;

/*
 * States declaration
 */

enum GSE_State {
	empty,
	OPEN_FILL_VALVE,
	CLOSE_FILL_VALVE,
	OPEN_PURGE_VALVE,
	CLOSE_PURGE_VALVE,
	OPEN_FILL_VALVE_BACKUP,
	CLOSE_FILL_VALVE_BACKUP,
	OPEN_PURGE_VALVE_BACKUP,
	CLOSE_PURGE_VALVE_BACKUP,
	DISCONNECT_HOSE,
	//TODO Add sensor value request order
	MAIN_IGNITION_ON,
	MAIN_IGNITION_OFF,
	SECONDARY_IGNITION_ON,
	SECONDARY_IGNITION_OFF,

};

enum Warning {
	EVENT, WARNING_MOTOR_PRESSURE
};


static inline void uint8ToFloat(uint8_t *uint8Ptr, float *floatPtr) {
	uint8_t *floatAsUintPtr = (uint8_t*) floatPtr;
	floatAsUintPtr[0] = uint8Ptr[3];
	floatAsUintPtr[1] = uint8Ptr[2];
	floatAsUintPtr[2] = uint8Ptr[1];
	floatAsUintPtr[3] = uint8Ptr[0];
}

static inline int32_t mod(int32_t x, int32_t n) {
	int32_t r = x % n;
	return r < 0 ? r + n : r;
}

static inline void floatToUint8(uint8_t *uint8Ptr, float *floatPtr) {
	uint8_t *floatAsUintPtr = (uint8_t*) floatPtr;
	uint8Ptr[0] = floatAsUintPtr[3];
	uint8Ptr[1] = floatAsUintPtr[2];
	uint8Ptr[2] = floatAsUintPtr[1];
	uint8Ptr[3] = floatAsUintPtr[0];
}

static inline float32_t abs_fl32(float32_t v) {
	return (v >= 0) ? v : -v;
}

static inline float32_t array_mean(float32_t *array, uint8_t arraySize) {
	uint8_t i;
	float32_t sum = 0.0;

	for (i = 0; i < arraySize; i++) {
		sum += array[i];
	}

	return sum / arraySize;
}

#ifdef __cplusplus
}
#endif

#endif /* INCLUDE_COMMON_H_ */
