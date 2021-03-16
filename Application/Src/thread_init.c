/*
 * threads.c
 *
 *  Created on: 11 Feb 2021
 *      Author: Lucas
 */

#include <can_reception.h>
#include <stddef.h>
#include "main.h"
#include <thread_init.h>

#include <sync.h>
#include <debug/led.h>
#include <debug/shell.h>
#include <debug/console.h>
#include <GSE/valve.h>
#include <GSE/ignition.h>
#include <GSE/code.h>
#include <sensor.h>

#include "FreeRTOS.h"
#include "task.h"
#include "cmsis_os.h"

osThreadId task_ShellHandle;
osThreadId task_LEDHandle;
osThreadId telemetryTransmissionHandle;
osThreadId telemetryReceptionHandle;
osThreadId canReaderHandle;
osThreadId heavyIOHandle;
osThreadId GSEValveHandle;
osThreadId GSEIgnitionHandle;
osThreadId GSECodeHandle;
osThreadId GSESensorHandle;

void create_semaphores() {
	//init_heavy_scheduler();
}
void create_threads() {


	osThreadDef(task_LED, TK_led_handler, osPriorityNormal, 0, 256);
	task_LEDHandle = osThreadCreate(osThread(task_LED), NULL);
	rocket_log("LED thread started.\n");

	osThreadDef(can_reader, TK_can_reader, osPriorityNormal, 0, 1024);
	canReaderHandle = osThreadCreate(osThread(can_reader), NULL);
	rocket_log("CAN reception thread started.\n");

//	osThreadDef(heavy_io, TK_heavy_io_scheduler, osPriorityNormal, 0, 1024);
//	heavyIOHandle = osThreadCreate(osThread(heavy_io), NULL);
//	rocket_log("Heavy IO thread started.\n");

	#ifdef XBEE
	  xbee_freertos_init(&huart1);
	  osThreadDef(xBeeTransmission, TK_xBeeTransmit, osPriorityNormal, 0, 2048);
	  telemetryTransmissionHandle = osThreadCreate(osThread(xBeeTransmission), NULL);
	  rocket_log("Telemetry transmission thread started.\n");
	  osThreadDef(xBeeReception, TK_xBeeReceive, osPriorityNormal, 0, 2048);
	  telemetryReceptionHandle = osThreadCreate(osThread(xBeeReception), NULL);
	  rocket_log("Telemetry reception thread started.\n");
	#endif

	  osThreadDef(task_shell, TK_shell, osPriorityNormal, 0, 256);
	  	task_ShellHandle = osThreadCreate(osThread(task_shell), NULL);
	  	rocket_boot_log("Shell thread started.\n");

	#ifdef VALVE
	  valve_init();
	  osThreadDef(GSE_valves, TK_GSE_valve_control, osPriorityNormal, 0, 128);
	  GSEValveHandle = osThreadCreate(osThread(GSE_valves), NULL);
	  rocket_log("Valve control thread started. \n");
	#endif

	#ifdef IGNITION
	  ignition_sys_init();
	  osThreadDef(ignition, TK_ignition_control, osPriorityNormal, 0, 128);
		  GSEIgnitionHandle = osThreadCreate(osThread(ignition), NULL);
	  rocket_log("Ignition control thread started.\n");
	#endif

	#ifdef SECURITY_CODE
	  code_init();
	  osThreadDef(security_code, TK_code_control, osPriorityNormal, 0, 128);
		  GSECodeHandle = osThreadCreate(osThread(security_code), NULL);
	  rocket_log("Security Code control thread started.\n");
	#endif


#ifdef SENSORS
	  	sensors_init();
		osThreadDef(GSE_sensor, TK_sensors_control, osPriorityNormal, 0, 128);
			  GSESensorHandle = osThreadCreate(osThread(GSE_sensor), NULL);
		rocket_log("GSE Sensors thread started.\n");
#endif
}
