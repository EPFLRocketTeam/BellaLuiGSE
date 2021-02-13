/*
 * console.h
 *
 *  Created on: 11 Feb 2020
 *      Author: Arion
 */

#ifndef APPLICATION_HOSTBOARD_INC_DEBUG_CONSOLE_H_
#define APPLICATION_HOSTBOARD_INC_DEBUG_CONSOLE_H_

// Semi hosting has to be enabled in eclipse, otherwise the program will sigtrap at the instruction initialise_monitor_handler() in main.c

#include <thread_init.h>
#include <stdio.h>

#include <stm32f4xx_hal.h>

#define CONSOLE_BUFFER_SIZE 256


#ifdef __cplusplus
extern "C"
{
#endif

#ifdef DEBUG_MONITOR
#define rocket_log printf
extern void initialise_monitor_handles(void);
#endif

void rocket_log_lock();
void rocket_log_release();
void rocket_direct_transmit(uint8_t* buffer, uint32_t length);
void rocket_transmit(uint8_t* buffer, uint32_t length);
int rocket_boot_log(const char* format, ...);
int rocket_log(const char* format, ...);
void rocket_log_init(UART_HandleTypeDef* uart);
UART_HandleTypeDef* get_console_uart();

#ifdef __cplusplus
}
#endif

#endif /* APPLICATION_HOSTBOARD_INC_DEBUG_CONSOLE_H_ */
