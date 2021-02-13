/*
 * ignition.h
 *
 *  Created on: 21 Feb 2021
 *      Author: Lucas Pallez
 *
 */

#ifndef APPLICATION_HOSTBOARD_INC_GSE_IGNITION_H_
#define APPLICATION_HOSTBOARD_INC_GSE_IGNITION_H_

void ignition_sys_init(void);
void TK_ignition_control(void const * argument);
float read_current();


#endif /* APPLICATION_HOSTBOARD_INC_GSE_IGNITION_H_ */
