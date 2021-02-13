/*
 * data_handling.c
 *
 *  Created on: 19 Apr 2018
 *      Author: Clement Nussbaumer
 *      Alexandre Devienne
 */

#include "debug/led.h"
#include <misc/common.h>
#include <misc/data_handling.h>
#include <misc/datagram_builder.h>
#include "cmsis_os.h"

#include <stdbool.h>
#include <telemetry/simpleCRC.h>
#include <telemetry/telemetry_protocol.h>
//#include <GSE/code.h>

extern "C" {
	#include <can_transmission.h>
	#include <debug/console.h>
}

#define GSE_STATE_TIMEMIN 100
#define ORDER_TIMEMIN 10
#define GSE_IGNITION_TIMEMIN 10
#define ECHO_TIMEMIN 100

volatile static uint32_t Packet_Number = 0;

// for import in C code
extern "C" bool telemetry_sendGSEStateData(GSE_state data);
extern "C" bool telemetry_sendOrderData(uint8_t order);
extern "C" bool telemetry_sendIgnitionData(uint8_t GSE_ignition);
extern "C" bool telemetry_sendEcho();

extern "C" bool telemetry_receiveOrderPacket(uint32_t time_stamp, uint8_t* payload);
extern "C" bool telemetry_receiveIgnitionPacket(uint32_t time_stamp, uint8_t* payload);

extern osMessageQId xBeeQueueHandle;

uint32_t telemetrySeqNumber = 0;
uint8_t current_GSE_order = 0;

GSE_state GSE_states = {0,0,0,0,0,1111,0,0,0,0,0,0,0,0,0,0,0,0};
uint8_t sec_GST_code = 0;
uint32_t last_GSE_state_update = 0;
uint32_t last_order_update = 0;
uint32_t last_GSE_ignition_update = 0;
uint32_t last_echo = 0;

Telemetry_Message m1;
Telemetry_Message m2;
Telemetry_Message m3;
Telemetry_Message m4;
Telemetry_Message m5;
Telemetry_Message m6;
Telemetry_Message m7;
Telemetry_Message m8;
Telemetry_Message m9;
Telemetry_Message m10;


Telemetry_Message event;

//the createXXXDatagram-Methods create the datagrams as described in the Schema (should be correct)

Telemetry_Message createOrderDatagram(uint8_t order, uint32_t time_stamp, uint32_t seqNumber)
{
	DatagramBuilder builder = DatagramBuilder (ORDER_DATAGRAM_PAYLOAD_SIZE, ORDER_PACKET,time_stamp, seqNumber);

	builder.write32<uint32_t> (time_stamp);
	builder.write32<uint32_t> (Packet_Number++);
	builder.write8 (order);
	return builder.finalizeDatagram();

}

Telemetry_Message createIgnitionDatagram(uint8_t GSE_ignition, uint32_t time_stamp, uint32_t seqNumber)
{
	//GST_code_result = verify_security_code(GST_code);
	uint8_t GST_code_result = 0;

	DatagramBuilder builder = DatagramBuilder (GSE_IGNITION_DATAGRAM_PAYLOAD_SIZE, IGNITION_PACKET, time_stamp, seqNumber);

	builder.write32<uint32_t> (time_stamp);
	builder.write32<uint32_t> (Packet_Number++);
	builder.write8 (GSE_ignition);
	builder.write8(GST_code_result);
	return builder.finalizeDatagram();

}

Telemetry_Message createEchoDatagram(uint32_t time_stamp, uint32_t seqNumber)
{

	DatagramBuilder builder = DatagramBuilder (GSE_IGNITION_DATAGRAM_PAYLOAD_SIZE, ECHO_PACKET,time_stamp, seqNumber);

	builder.write32<uint32_t> (time_stamp);
	builder.write32<uint32_t> (Packet_Number++);
	builder.write8(0xCA);
	return builder.finalizeDatagram();

}
Telemetry_Message createGSEStateDatagram(GSE_state* GSE, uint32_t time_stamp, uint32_t seqNumber) {
	DatagramBuilder builder = DatagramBuilder(GSE_STATE_DATAGRAM_PAYLOAD_SIZE, GSE_STATE_PACKET,time_stamp, seqNumber);

	builder.write8 (GSE->fill_valve_state);
	builder.write8 (GSE->purge_valve_state);
	builder.write8 (GSE->main_ignition_state);
	builder.write8 (GSE->sec_ignition_state);
	builder.write8 (GSE->hose_disconnect_state);
	builder.write32<float32_t>(GSE->battery_level);
	builder.write32<float32_t>(GSE->hose_pressure);
	builder.write32<float32_t>(GSE->hose_temperature);
	builder.write32<float32_t>(GSE->tank_temperature);
	builder.write32<float32_t>(GSE->rocket_weight);
	builder.write32<float32_t>(GSE->ignition1_current);
	builder.write32<float32_t>(GSE->ignition2_current);
	builder.write32<float32_t>(GSE->wind_speed);

	return builder.finalizeDatagram();
}

bool telemetry_sendGSEStateData(GSE_state data)
{
	uint32_t now = HAL_GetTick();
	bool handled = false;

	if (now - last_GSE_state_update > GSE_STATE_TIMEMIN) {
		m7 = createGSEStateDatagram(&data, now, telemetrySeqNumber++);
		if (osMessagePut (xBeeQueueHandle, (uint32_t) &m7, 10) != osOK) {
			vPortFree(m7.ptr); // free the datagram if we couldn't queue it
		}
		last_GSE_state_update = now;
		handled = true;

	}
	return handled;
}

bool telemetry_sendOrderData(uint8_t order)
{
	uint32_t now = HAL_GetTick();
	bool handled = false;

	if (now - last_order_update > ORDER_TIMEMIN) {
		m8 = createOrderDatagram(order, now, telemetrySeqNumber++);
		if (osMessagePut (xBeeQueueHandle, (uint32_t) &m8, 10) != osOK) {
			vPortFree(m8.ptr); // free the datagram if we couldn't queue it
		}
		last_order_update = now;
		handled = true;

	}
	return handled;
}

bool telemetry_sendIgnitionData(uint8_t GSE_ignition)
{
	uint32_t now = HAL_GetTick();
	bool handled = false;

	if (now - last_GSE_ignition_update > GSE_IGNITION_TIMEMIN) {
		m9 = createIgnitionDatagram(GSE_ignition, now, telemetrySeqNumber++);
		if (osMessagePut (xBeeQueueHandle, (uint32_t) &m9, 10) != osOK) {
			vPortFree(m9.ptr); // free the datagram if we couldn't queue it
		}
		last_GSE_ignition_update = now;
		handled = true;

	}
	return handled;
}

bool telemetry_sendEcho()
{
	uint32_t now = HAL_GetTick();
	bool handled = false;

	if (now - last_echo > ECHO_TIMEMIN) {
		m10 = createEchoDatagram(now, telemetrySeqNumber++);
		if (osMessagePut (xBeeQueueHandle, (uint32_t) &m10, 10) != osOK) {
			vPortFree(m10.ptr); // free the datagram if we couldn't queue it
		}
		last_echo = now;
		handled = true;

	}
	return handled;
}

// Received Packet Handling

bool telemetry_receiveOrderPacket(uint32_t ts, uint8_t* payload) {

	switch (payload[0])
	{
		case OPEN_FILL_VALVE:
		{
			current_GSE_order = OPEN_FILL_VALVE;
			break;
		}
		case CLOSE_FILL_VALVE:
		{
			current_GSE_order = CLOSE_FILL_VALVE;
			break;
		}
		case OPEN_PURGE_VALVE:
		{
			current_GSE_order = OPEN_PURGE_VALVE;
			break;
		}
		case CLOSE_PURGE_VALVE:
		{
			current_GSE_order = CLOSE_PURGE_VALVE;
			break;
		}
		case OPEN_FILL_VALVE_BACKUP:
		{
			current_GSE_order = OPEN_FILL_VALVE_BACKUP;
			break;
		}
		case CLOSE_FILL_VALVE_BACKUP:
		{
			current_GSE_order = CLOSE_FILL_VALVE_BACKUP;
			break;
		}
		case OPEN_PURGE_VALVE_BACKUP:
		{
			current_GSE_order = OPEN_PURGE_VALVE_BACKUP;
			break;
		}
		case CLOSE_PURGE_VALVE_BACKUP:
		{
			current_GSE_order = CLOSE_PURGE_VALVE_BACKUP;
			break;
		}
		case DISCONNECT_HOSE:
		{
			current_GSE_order = DISCONNECT_HOSE;
			break;
		}
	}
	can_setFrame((uint32_t) current_GSE_order, DATA_ID_ORDER , ts);
	return 0;
}

bool telemetry_receiveIgnitionPacket(uint32_t ts, uint8_t* payload) {
	if(payload[0] == MAIN_IGNITION_ON) {
		can_setFrame((uint32_t) MAIN_IGNITION_ON, DATA_ID_IGNITION, ts);
	}
	else if(payload[0] == MAIN_IGNITION_OFF) {
			can_setFrame((uint32_t) MAIN_IGNITION_OFF, DATA_ID_IGNITION, ts);
	}
	else if(payload[0] == SECONDARY_IGNITION_ON) {
		can_setFrame((uint32_t) SECONDARY_IGNITION_ON, DATA_ID_IGNITION, ts);
	}
	else if(payload[0] == SECONDARY_IGNITION_OFF) {
		can_setFrame((uint32_t) SECONDARY_IGNITION_OFF, DATA_ID_IGNITION, ts);
	}
	can_setFrame((uint32_t) payload[1], DATA_ID_GST_CODE, ts);
	return 0;
}


