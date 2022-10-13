/*
 * rover_pid.h
 *
 *  Created on: 09/giu/2020
 *      Author: nbattezzati
 */

#ifndef ROVER_PID_H_
#define ROVER_PID_H_

#include <stdint.h>
#include <stdbool.h>

typedef struct {
	/* Controller gains */
	float Kp;
	float Ki;
	float Kd;

	/* Derivative low-pass filter time constant */
	float tau;

	/* Output limits */
	float limMin;
	float limMax;

	/* Sample time (in seconds) */
	float T;

	/******************************************************************/
	/* Controller "memory" --> private variables: do not change them! */
	float integrator;
	float prevError;			/* Required for integrator */
	float differentiator;
	float prevMeasurement;		/* Required for differentiator */
	/******************************************************************/

} rover_PID_const_t;

void PIDControl_Init(rover_PID_const_t *pid);
float PIDControl(rover_PID_const_t *pid, float setpoint, float measurement, float * error, bool is_angle);
float SimplePIDControl (rover_PID_const_t *pid, float setpoint, float measurement, bool is_angle);


#endif /* ROVER_PID_H_ */
