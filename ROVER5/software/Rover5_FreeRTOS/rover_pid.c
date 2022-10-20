#include <math.h>
#include "rover_pid.h"

/*----------------------------------PUBLIC FUNCTIONS------------------------------------*/
// FUNCTION: PIDControl_Init()
// Description: function to clear the PID controller parameters
void PIDControl_Init(rover_PID_const_t *pid)
{
	/* Clear controller variables */
	pid->integrator = 0.0f;
	pid->prevError  = 0.0f;
	pid->differentiator  = 0.0f;
	pid->prevMeasurement = 0.0f;
}

// FUNCTION: PIDControl()
// Description: function to apply a PID controller
float PIDControl(rover_PID_const_t *pid, float setpoint, float measurement, float * error, bool is_angle)
{
	float out = 0.0;
	
	/* Compute the error	*/
    *error = setpoint - measurement;

	/* If is an angle avoid the error to go to infinite*/
    if(is_angle == true){
    	*error = atan2f(sinf(*error), cosf(*error));
    }

	/* Compute proportional part of the controller	*/
    float proportional = pid->Kp * (*error);

	/* Compute integral part of the controller	*/
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

	/* 	Compute derivative part of the controller (band-limited differentiator)	*/

    pid->differentiator = -(2.0f * pid->Kd * (measurement - pid->prevMeasurement)	/* Note: derivative on measurement, therefore minus sign in front of equation! */
                        + (2.0f * pid->tau - pid->T) * pid->differentiator)
                        / (2.0f * pid->tau + pid->T);

	/* Compute output and apply limits */
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

// FUNCTION: SimplePIDControl()
// Description: function to send a message over the RF channel to a specific recipient
/* TO DO: Merge this function with the PIDControl function*/
float SimplePIDControl (rover_PID_const_t *pid, float setpoint, float measurement, float * error, bool is_angle)
{
	float out = 0.0;

	/* Compute the error */
	*error	= setpoint - measurement;
	
	/* If is an angle avoid the error to go to infinite*/
	if(is_angle == true){
		*error = atan2f(sinf(*error), cosf(*error));
	}

	/* Compute differential part of the controller */
	pid->differentiator	 = *error - pid->prevError;
	/* Compute integral part of the controller	*/
	pid->integrator	+= *error;
	
	/* Compute the controller output */
	out  = (pid->Kp * (*error)) + (pid->Kd*pid->differentiator) + (pid->Ki*pid->integrator);
	
	/* Store error and measurement for later use */
	pid->prevError   = *error;

	/* Return controller output */
	return out;
}
/*--------------------------------------------------------------------------------------*/
