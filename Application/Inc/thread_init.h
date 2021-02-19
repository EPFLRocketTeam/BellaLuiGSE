/*
 * threads.h
 *
 *  Created on: 11 Feb 2020
 *      Author: Arion
 */

#ifndef APPLICATION_HOSTBOARD_INC_THREADS_H_
#define APPLICATION_HOSTBOARD_INC_THREADS_H_


#define OS_STKCHECK
#define LED
//#define TELEMETRY_BOARD
//#define VALVE_BOARD
//#define CODE_BOARD
#define SENSOR_TELEMETRY_BOARD

//GSE Boards
#ifdef VALVE_BOARD
#define CAN_ID CAN_ID_GSE_VALVE_BOARD
#define VALVE
#define IGNITION
#define IGNITION_1

#define BOARD_LED_R (0)
#define BOARD_LED_G (127)
#define BOARD_LED_B (127)
#endif

#ifdef CODE_BOARD
#define CAN_ID CAN_ID_GSE_CODE_BOARD
#define SECURITY_CODE
#define IGNITION
#define DISCONNECT

#define BOARD_LED_R (0)
#define BOARD_LED_G (255)
#define BOARD_LED_B (0)
#endif

#ifdef SENSOR_TELEMETRY_BOARD
#define CAN_ID CAN_ID_GSE_SENSOR_TELEMETRY_BOARD
#define SENSOR_TELEMETRY
#define XBEE

//########################################## TEST ##########################################
//#define VALVE
//########################################## TEST ##########################################

#define IGNITION
//#define IGNITION_2

#define BOARD_LED_R (0)
#define BOARD_LED_G (0)
#define BOARD_LED_B (255)
#endif

#ifdef XBEE
#include <telemetry/xbee.h>
#endif

//GSE Defs
#ifdef VALVE
#include <GSE/valve.h>
#endif

#ifdef IGNITION
#include <GSE/ignition.h>
#endif

#ifdef SECURITY_CODE
#include <GSE/code.h>
#endif

#ifdef SENSOR_TELEMETRY
#include <GSE/sensor_telemetry.h>
#endif

void create_semaphores();
void create_threads();


#endif /* APPLICATION_HOSTBOARD_INC_THREADS_H_ */
