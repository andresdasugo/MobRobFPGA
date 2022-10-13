/*
 * rover_pid.c
 *
 *  Created on: 09/giu/2020
 *      Author: nbattezzati
 */

#include <math.h>
#include "rover_pid.h"




void PIDControl_Init(rover_PID_const_t *pid)
{
	/* Clear controller variables */
	pid->integrator = 0.0f;
	pid->prevError  = 0.0f;

	pid->differentiator  = 0.0f;
	pid->prevMeasurement = 0.0f;
}

float PIDControl(rover_PID_const_t *pid, float setpoint, float measurement, float * error, bool is_angle)
{
	float out = 0.0;

	/*
	* Error signal
	*/
    *error = setpoint - measurement;

    if(is_angle == true){
    	*error = atan2f(sinf(*error), cosf(*error));
    }

	/*
	* Proportional
	*/
    float proportional = pid->Kp * (*error);


	/*
	* Integral
	*/
    pid->integrator = pid->integrator + 0.5f * pid->Ki * pid->T * (*error + pid->prevError);


	/* Anti-wind-up via dynamic integrator clamping */
	float limMinInt, limMaxInt;

	/* Compute integrator limits */
	if (pid->limMax > proportional) {

		limMaxInt = pid->limMax - proportional;

	} else {

		limMaxInt = 0.0f;

	}

	if (pid->limMin < proportional) {

		limMinInt = pid->limMin - proportional;

	} else {

		limMinInt = 0.0f;

	}

	/* Clamp integrator */
    if (pid->integrator > limMaxInt) {

        pid->integrator = limMaxInt;

    } else if (pid->integrator < limMinInt) {

        pid->integrator = limMinInt;

    }


	/*
	* Derivative (band-limited differentiator)
	*/

    pid->differentiator = -(2.0f * pid->Kd * (measurement - pid->prevMeasurement)	/* Note: derivative on measurement, therefore minus sign in front of equation! */
                        + (2.0f * pid->tau - pid->T) * pid->differentiator)
                        / (2.0f * pid->tau + pid->T);


	/*
	* Compute output and apply limits
	*/
    out = proportional + pid->integrator + pid->differentiator;

    if (out > pid->limMax) {
        out = pid->limMax;
    } else if (out < pid->limMin) {
        out = pid->limMin;
    }

	/* Store error and measurement for later use */
    pid->prevError       = *error;
    pid->prevMeasurement = measurement;

	/* Return controller output */
    return out;

}

float SimplePIDControl (rover_PID_const_t *pid, float setpoint, float measurement, bool is_angle)
{
	float out = 0.0;

	float e			= setpoint - measurement;
	if(is_angle == true){
		e 			= atan2f(sinf(e), cosf(e));
	}
	float e_dot 	 = e - pid->prevError;
	pid->integrator	+= e;
	out 		 = (pid->Kp*e) + (pid->Kd*e_dot) + (pid->Ki*pid->integrator);
	pid->prevError   = e;

	return out;
}
