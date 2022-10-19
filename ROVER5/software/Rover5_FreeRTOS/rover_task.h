/*
 * rover_task.h
 *
 *  Created on: 25/mar/2020
 *      Author: nbattezzati
 */

#ifndef ROVER_TASK_H_
#define ROVER_TASK_H_

/*---------------------------------------DEFINES----------------------------------------*/
/*Task priorities*/
#define TASK_PRIO_LOW			(tskIDLE_PRIORITY + 1)
#define TASK_PRIO_MID			(tskIDLE_PRIORITY + 2)
#define TASK_PRIO_HIGH			(tskIDLE_PRIORITY + 3)
/* Task Execution Periods */
#define RoverTaskComm_PERIOD 	1     // ms
#define RoverTaskControl_PERIOD 100  // ms
#define RoverTaskSensing_PERIOD 1     // ms

// MACRO: TASK_MS_2_TICKS
// Parameters:  - time: the amount of time in milliseconds to convert to ticks
// Return:      the number of ticks for the given amount of time in ms
#define TASK_MS_2_TICKS(time)		(time/portTICK_PERIOD_MS)
/*--------------------------------------------------------------------------------------*/

#endif /* ROVER_TASK_H_ */
