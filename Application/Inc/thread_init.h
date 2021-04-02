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

/*
 * ================= Standard GSE Configuration =================
 *
 * HB1: Sensor_Telemetry_Board
 * 	> S1/S2: Sensor & Telemetry
 *
 * HB2: Code_Board
 * 	> S1: Power Controller -> Backup
 * 	> S2: Security Code
 *
 *
 * HB3: Power_Board
 * 	> S1: Power Controller -> Igniters / Disconnect
 * 	> S2: Power Controller -> Valves
 *
 */

//================= DEFINE THE BOARD TO FLASH =================

#define HB1_SENSOR_TELEMETRY_BOARD
//#define HB2_CODE_BOARD
//#define HB3_POWER_BOARD

//================= GSE Boards =================

//Flash on Hostboard 1 (top)
#ifdef HB1_SENSOR_TELEMETRY_BOARD
#define CAN_ID CAN_ID_GSE_SENSOR_TELEMETRY_BOARD
#define SENSORS
#define XBEE

#define BOARD_LED_R (0)
#define BOARD_LED_G (0)
#define BOARD_LED_B (255)
#endif

//Flash on Hostboard 2 (middle)
#ifdef HB2_CODE_BOARD
#define CAN_ID CAN_ID_GSE_CODE_BOARD
#define IGNITION
#define SECURITY_CODE
#define SENSORS

#define BOARD_LED_R (0)
#define BOARD_LED_G (255)
#define BOARD_LED_B (0)
#endif

//Flash on Hostboard 3 (bottom)
#ifdef HB3_POWER_BOARD
#define CAN_ID CAN_ID_GSE_POWER_BOARD
#define IGNITION
#define VALVE
#define SENSORS

#define BOARD_LED_R (255)
#define BOARD_LED_G (0)
#define BOARD_LED_B (0)
#endif

#ifdef XBEE
#include <telemetry/xbee.h>
#endif

//================= GSE Defs =================
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
