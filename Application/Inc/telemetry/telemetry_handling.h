/* telemetry_handling.h
 *
 *  Created on: 10 Jun 2019
 *      Author: Alexandre Devienne
 */

#ifndef TELEMETRY_HANDLING_H_
#define TELEMETRY_HANDLING_H_

#ifdef __cplusplus
 extern "C" {
#endif

#include <misc/datastructs.h>
#include <stdbool.h>
#include <sys/_stdint.h>

bool telemetry_sendCodeData();
bool telemetry_sendGSEStateData(GSE_state data);
bool telemetry_sendOrderData(uint8_t order);
bool telemetry_sendIgnitionData(uint8_t GSE_ignition);
bool telemetry_sendEcho();

bool telemetry_receiveOrderPacket(uint32_t time_stamp, uint8_t* payload);
bool telemetry_receiveIgnitionPacket(uint32_t time_stamp, uint8_t* payload);


#ifdef __cplusplus

#endif


#endif /* TELEMETRY_HANDLING_H_ */
