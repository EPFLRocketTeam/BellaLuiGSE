#include <debug/shell.h>
#include <stm32f4xx_hal.h>
#include <sys/_stdint.h>

/*
 * CAN_communication.c
 *
 * If you read a frame and there is a message, the led blinks blue.
 * If you write a frame and it fails, the led blinks red.
 * If you write a frame and if does not fail, the led blinks green.
 *
 *  Created on: Feb 23, 2019
 *      Author: Tim Lebailly
 */
typedef float float32_t;

#include <stdbool.h>
#include <thread_init.h>
#include <cmsis_os.h>

#include <can_transmission.h>
#include "debug/profiler.h"
#include "debug/led.h"
#include "debug/monitor.h"
#include "misc/datastructs.h"
#include "misc/common.h"
#include "debug/console.h"
#include "debug/terminal.h"
#include <telemetry/telemetry_handling.h>



#define BUFFER_SIZE 128
#define OUTPUT_SHELL_BUFFER_SIZE 256
#define SHELL_MIN_FLUSH_TIME 100

GSE_state GSE = {0,0,0,0,0,0,15,0,0,0,0,0,0,0,0,0,0,0,0};
uint8_t order = 0;
uint8_t ignition_order = 0;
uint8_t GST_code = 0;
uint8_t GST_code_result = 0;


// wrapper to avoid fatal crashes when implementing redundancy
int board2Idx(uint32_t board) {
	if (board < MAX_BOARD_ID) {
		return board;
	}  else { // invalid board ID
		// to avoid fatal crash return a default value
		return MAX_BOARD_ID;
	}
}

bool handleGSEStateData(GSE_state data) {
#ifdef XBEE
	return telemetry_sendGSEStateData(data);
#endif
	return true;
}

bool handleOrderData(uint8_t order) {
#ifdef XBEE
	return telemetry_sendOrderData(order);
#endif
	return true;
}

bool handleIgnitionData(uint8_t GSE_ignition) {
#ifdef XBEE
	return telemetry_sendIgnitionData(GSE_ignition);
#endif
	return true;
}

bool handleEchoData() {
#ifdef XBEE
	return telemetry_sendEcho();
#endif
	return true;
}

uint8_t can_getOrder() {
	return order;
}

uint8_t can_getGSTCode() {
	return GST_code;
}

uint8_t can_getIgnitionOrder() {
	return ignition_order;
}

GSE_state can_getGSEState() {
	return GSE;
}

void TK_can_reader() {
	// init
	CAN_msg msg;


	bool new_order = 0;
	bool new_GSE_state = 0;
	bool new_GST_code = 0;
	bool new_GSE_ignition_order = 0;
	bool new_echo = 0;
	int idx = 0;
	uint32_t shell_command;
	uint32_t shell_payload;

	osDelay (500); // Wait for the other threads to be ready

	while(true) {
		//start_profiler(1);

		while (can_msgPending()) { // check if new data
			msg = can_readBuffer();
			//rocket_log("Received can frame ID %d\n", msg.id);

//			if((int32_t) (HAL_GetTick() - msg.timestamp) > 100000) {
//				rocket_log("CAN RX Error %d@%d vs %d\n", msg.id, msg.timestamp, HAL_GetTick());
//				continue;
//			}

			if(is_verbose()) {
				rocket_log("----- CAN RX frame begins -----\n");
				rocket_log("Frame Source: %d\n", (uint32_t) msg.id_CAN);
				rocket_log("Frame ID: %d\n", (uint32_t) msg.id);
				rocket_log("Frame Timestamp: %d\n", (uint32_t) msg.timestamp);
				rocket_log("Frame Data: %d\n", (uint32_t) msg.data);
				rocket_log("----- CAN RX frame ends -----\n");
			}

			idx = board2Idx(msg.id_CAN);

			switch(msg.id) {

			//Shell

			case DATA_ID_SHELL_CONTROL:
				shell_command = msg.data & 0xFF000000;
				shell_payload = msg.data & 0x00FFFFFF;

				if(shell_command == SHELL_BRIDGE_CREATE) {
					shell_bridge(shell_payload & 0xF);
					can_setFrame(SHELL_ACK, DATA_ID_SHELL_CONTROL, HAL_GetTick());
					rocket_log("\n\nBellaLui Terminal for board %u\n\n", get_board_id());
				} else if(shell_command == SHELL_BRIDGE_DESTROY) {
					shell_bridge(-1);
				} else if(shell_command == SHELL_ACK) {
					rocket_direct_transmit((uint8_t*) "> Connected to remote shell\n", 28);
				} else if(shell_command == SHELL_ERR) {
					rocket_direct_transmit((uint8_t*) "> Failed to connect to remote shell\n", 36);
				}

				break;
			case DATA_ID_SHELL_INPUT: // Little-Endian
				shell_receive_byte(((char*) &msg.data)[0], -1);
				shell_receive_byte(((char*) &msg.data)[1], -1);
				shell_receive_byte(((char*) &msg.data)[2], -1);
				shell_receive_byte(((char*) &msg.data)[3], -1);
				break;
			case DATA_ID_SHELL_OUTPUT:
				rocket_direct_transmit((uint8_t*) &msg.data, 4);
				break;

			//Orders and Rx Data

			case DATA_ID_ORDER:
				order = msg.data;
				new_order = true;
				new_GSE_state = true;
				break;
			case DATA_ID_IGNITION:
				ignition_order = msg.data;
				new_GSE_ignition_order = true;
				break;
			case DATA_ID_GST_CODE:
				GST_code = msg.data;
				new_GST_code = true;
				break;

			//States

			case DATA_ID_FILL_VALVE_STATE:
				GSE.fill_valve_state = msg.data;
				new_GSE_state = true;
				break;
			case DATA_ID_PURGE_VALVE_STATE:
				GSE.purge_valve_state = msg.data;
				new_GSE_state = true;
				break;
			case DATA_ID_MAIN_DISCONNECT_STATE:
				GSE.main_disconnect_state = msg.data;
				new_GSE_state = true;
				break;
			case DATA_ID_SEC_DISCONNECT_STATE:
				GSE.sec_disconnect_state = msg.data;
				new_GSE_state = true;
				break;
			case DATA_ID_MAIN_IGNITION_STATE:
				GSE.main_ignition_state = msg.data;
				new_GSE_state = true;
				break;
			case DATA_ID_SEC_IGNITION_STATE:
				GSE.sec_ignition_state = msg.data;
				new_GSE_state = true;
				break;

			//Sensor & Input data

			case DATA_ID_GSE_CODE:
				GSE.code = msg.data;
				new_GSE_state = true;
				break;
			case DATA_ID_IGNITION_CURRENT_1:
				GSE.ignition1_current = msg.data;
				new_GSE_state = true;
				break;
			case DATA_ID_IGNITION_CURRENT_2:
				GSE.ignition2_current = msg.data;
				new_GSE_state = true;
				break;
			case DATA_ID_DISCONNECT_CURRENT_1:
				GSE.disconnect1_current = msg.data;
				new_GSE_state = true;
				break;
			case DATA_ID_DISCONNECT_CURRENT_2:
				GSE.disconnect2_current = msg.data;
				new_GSE_state = true;
				break;
			case DATA_ID_FILL_VALVE_CURRENT:
				GSE.fill_valve_current = msg.data;
				new_GSE_state = true;
				break;
			case DATA_ID_PURGE_VALVE_CURRENT:
				GSE.purge_valve_current = msg.data;
				new_GSE_state = true;
				break;
			case DATA_ID_IN_HOSE_PRESSURE:
				GSE.hose_pressure = msg.data;
				new_GSE_state = true;
				break;
			case DATA_ID_IN_HOSE_TEMPERATURE:
				GSE.hose_temperature = msg.data;
				new_GSE_state = true;
				break;
			case DATA_ID_TANK_TEMPERATURE:
				GSE.tank_temperature = msg.data;
				new_GSE_state = true;
				break;
			case DATA_ID_ROCKET_WEIGHT:
				GSE.rocket_weight = msg.data;
				new_GSE_state = true;
				break;
			case DATA_ID_BATTERY_LEVEL:
				GSE.battery_level = msg.data;
				new_GSE_state = true;
				break;
			case DATA_ID_WIND_SPEED:
				GSE.wind_speed = msg.data;
				new_GSE_state = true;
				break;

			//Ping

			case DATA_ID_ECHO:
				new_echo = true;
				break;

			default:
				rocket_log("Unhandled can frame ID %d\n", msg.id);
				break;
			}
		}

		// check if new/non-handled full packets are present
		if(new_order) {
			new_order = !handleOrderData(order);
		}
		if(new_GSE_ignition_order) {
			new_GSE_ignition_order = !handleIgnitionData(ignition_order);
		}
		if(new_GST_code) {
			new_GST_code = !handleIgnitionData(ignition_order);
		}
		if(new_echo) {
			new_echo = !handleEchoData();
		}
		if(new_GSE_state){
			new_GSE_state = !handleGSEStateData(GSE);
		}

		osDelay(10);
	}
}
