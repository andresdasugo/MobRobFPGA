/*
 * FreeRTOS Kernel V10.3.0
 * Copyright (C) 2020 Amazon.com, Inc. or its affiliates.  All Rights Reserved.
 */

/*
* [File Name]     Rover5_FreeRTOS.c
* [Platform]      DE0-Nano
* [Project]       MobRobFPGA/ROB3D & FreeRTOS
* [Version]       1.3
* [Author]        Andres David Suarez Gomez
* [Date]          13/04/2023
* [Language]      'C'
* [History]       1.0 - Full migration of the C Project to FreeRTOS
*				  1.1 - Implementation of PID and Lyapunov pose controller
*				  1.2 - Development of an IP core for encoder reading
*				  1.3 - Implementation of communication using XBee boards
*/

/*---------------------------------------INCLUDES---------------------------------------*/

/* Standard includes. */
#include <stddef.h>
#include <stdio.h>
#include <string.h>
#include <alt_types.h>
#include <math.h> 			// for the math functions
#include <stdint.h>			// for the uint types


/* System specific includes. */
#include "io.h"				// for reading or writing to I/O
#include "system.h"			// for the configuration and register of the QSYS system
#include "sys/alt_irq.h"
#include "altera_avalon_uart.h"
#include "altera_up_avalon_de0_nano_adc.h"
#include "altera_up_avalon_accelerometer_spi.h"
#include "altera_avalon_pio_regs.h"
#include "altera_avalon_timer_regs.h"
#include "altera_avalon_uart_regs.h"

/* Scheduler includes. */
#include "FreeRTOS.h"
#include "task.h"
#include "queue.h"

/*--- Nik START ---*/
#include "rover_comm.h"
#include "rover_task.h"
/*--- Nik END ---*/
#include "rover_sensing.h"
#include "rover_control.h"

/*---------------------------------------DEFINES----------------------------------------*/

#define TRUE 	1
#define FALSE 	0

/* Mobile robot ID */
#define ROBOT_ID 	"RV0"

/*--------------------------------------------------------------------------------------*/

/*--------------------------------------VARIABLES---------------------------------------*/



/*--------------------------------------------------------------------------------------*/

/*---------------------------------FUNCTION PROTOTYPES----------------------------------*/

void vApplicationStackOverflowHook( TaskHandle_t xTask, signed char *pcTaskName );

/*--------------------------------------------------------------------------------------*/

/*------------------------------------MAIN FUNCTION-------------------------------------*/
int main( void )
{

	// create Communication task
	//   Name:  let´s name the tasks RoverTask___
	//   Stack: let's start with 128 kWords (i.e. 512 kB, since word is 32-bit wide)
	//   Prio:  keep prio low, let's rise prio only for very hard real time tasks

	/*
	* Task: 		RoverTaskComm
	* Description: 	Handles all of the robot communication through XBee
	* Notes: 		The task stops functioning if I put 128*configMINIMAL_STACK_SIZE
	*/
	if (xTaskCreate(RoverTaskComm, "T_comm", 32*configMINIMAL_STACK_SIZE, NULL, TASK_PRIO_LOW, NULL) != pdPASS) {
		printf("[ERR]: failed to create task T_comm\n");
	}

	/*
	* Task: 		RoverTaskSensing
	* Description: 	Handles sensor inputs like ultrasonic sensors and LiDaR
	* Notes: 		The task stops functioning if I put 128*configMINIMAL_STACK_SIZE
	*/
	//if (xTaskCreate(RoverTaskSensing, "T_sensing", configMINIMAL_STACK_SIZE, NULL, TASK_PRIO_LOW, NULL) != pdPASS) {
	//		printf("[ERR]: failed to create task T_sensing\n");
	//}

	/*
	* Task: 		RoverTaskControl
	* Description: 	Controls the motion of the robot
	* Notes: 		The task stops functioning if I put 128*configMINIMAL_STACK_SIZE
	*/
	if (xTaskCreate(RoverTaskControl, "T_control", 32*configMINIMAL_STACK_SIZE, NULL, TASK_PRIO_HIGH, NULL) != pdPASS) {
		printf("[ERR]: failed to create task T_control\n");
	}

    /* Finally start the scheduler. */
	vTaskStartScheduler();

	/* Will only reach here if there is insufficient heap available to start the scheduler. */
	for( ;; );
}
/*--------------------------------------------------------------------------------------*/

/*--------------------------------------FUNCTIONS---------------------------------------*/

void vApplicationStackOverflowHook( TaskHandle_t xTask, signed char *pcTaskName )
{
	printf("Task %s STACK OVERFLOW\n", pcTaskName);
}

/*--------------------------------------------------------------------------------------*/
