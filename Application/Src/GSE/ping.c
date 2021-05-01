/*
 * ping.c
 *
 *  Created on: 14 Apr 2021
 *      Author: Lucas Pallez
 *
 */

#include <cmsis_os.h>
#include <ping.h>
#include <telemetry_handling.h>

void TK_WDG_pinger(void const * argument)
{
	for(;;)
	{
		telemetry_sendEcho();
		//TODO add what to do if timer runs out

		osDelay(500);
	}
}
