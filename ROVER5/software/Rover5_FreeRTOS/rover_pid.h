#ifndef ROVER_PID_H_
#define ROVER_PID_H_

#include <stdint.h>
#include <stdbool.h>

/*------------------------------------PUBLIC TYPES--------------------------------------*/

// TYPE: rover_PID_const_t
// Description: structure used to store parameters used for the PID controller
// Fields:      - Kp: Proportional gain
//              - Ki: Integral gain
//              - Kd: Derivative gain
//              - tau: Derivative low-pass filter time constant
//              - limMin: Output minimum limit
//              - limMax: Output maximum limit
//              - T: Sample time (in seconds)
//              - integrator: variable used for the integrator
//              - prevError: previous error
//              - differentiator: variable used for the differentiator
//              - prevMeasurement: previous measurement
typedef struct {
	float Kp;
	float Ki;
	float Kd;
	float tau;
	float limMin;
	float limMax;
	float T;
	/* Controller "memory" --> private variables: do not change them! */
	float integrator;
	float prevError;			/* Required for integrator */
	float differentiator;
	float prevMeasurement;		/* Required for differentiator */
} rover_PID_const_t;
/*--------------------------------------------------------------------------------------*/

/*---------------------------------FUNCTION PROTOTYPES----------------------------------*/
// FUNCTION: PIDControl_Init()
// Description: function to clear the PID controller parameters
// Parameters:  - pid: Parameters of the PID controller
// Return:      N/A
void PIDControl_Init(rover_PID_const_t *pid);

// FUNCTION: PIDControl()
// Description: function to apply a PID controller
// Parameters:  - pid: Parameters of the PID controller
//              - setpoint: Set point for the PID controller
//              - measurement: Measurement taken by the PID Controller
//              - error: Difference between the Set Point and the Measurement
//				- is_angle: Defines if the measurement is an angle (if it is an angle, applies computation to avoid the error going to infinite)
// Return:      Control signal to be applied
float PIDControl(rover_PID_const_t *pid, float setpoint, float measurement, float * error, bool is_angle);

// FUNCTION: SimplePIDControl()
// Description: function to apply a simple PID controller
// Parameters:  - pid: Parameters of the PID controller
//              - setpoint: Set point for the PID controller
//              - measurement: Measurement taken by the PID Controller
//              - error: Difference between the Set Point and the Measurement
//				- is_angle: Defines if the measurement is an angle (if it is an angle, applies computation to avoid the error going to infinite)
// Return:      Control signal to be applied
float SimplePIDControl (rover_PID_const_t *pid, float setpoint, float measurement, float * error, bool is_angle);
/*--------------------------------------------------------------------------------------*/

#endif /* ROVER_PID_H_ */
