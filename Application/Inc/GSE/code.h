/*
 * code.h
 *
 *  Created on: 21 Feb 2021
 *      Author: Lucas Pallez
 *
 */

#ifndef APPLICATION_HOSTBOARD_INC_GSE_CODE_H_
#define APPLICATION_HOSTBOARD_INC_GSE_CODE_H_

void code_init(void);
void TK_code_control(void const * argument);

uint8_t verify_security_code(uint8_t GST_code);

#endif /* APPLICATION_HOSTBOARD_INC_GSE_CODE_H_ */
