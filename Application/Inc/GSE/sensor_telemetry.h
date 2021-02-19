/*
 * sensor_telemetry.h
 *
 *  Created on: 21 Feb 2021
 *      Author: Lucas Pallez
 *
 */

#ifndef APPLICATION_HOSTBOARD_INC_GSE_SENSOR_TELEMETRY_H_
#define APPLICATION_HOSTBOARD_INC_GSE_SENSOR_TELEMETRY_H_

void telemetry_init(void);
void sensors_init(void);
void TK_telemetry_control(void const * argument);
void TK_sensors_control(void const * argument);

void read_tank_temp(void);
void read_anemo(void);
void read_hose_temp(void);
void read_hose_pressure(void);
void read_load_cell(void);


#endif /* APPLICATION_HOSTBOARD_INC_GSE_SENSOR_TELEMETRY_H_ */
