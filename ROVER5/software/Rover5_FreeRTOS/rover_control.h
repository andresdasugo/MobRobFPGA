/*
 * rover_control.h
 *
 *  Created on: 2/04/2020
 *      Author: andre
 */

#ifndef ROVER_CONTROL_H_
#define ROVER_CONTROL_H_


// FUNCTION: TaskControl()
// Description: main task of the control module.
//              It control the motion of the robot
// Parameters:  - pvParameters: pointer to task parameters (pass NULL, not used)
// Return:      -
void RoverTaskControl(void *pvParameters);



#endif /* ROVER_CONTROL_H_ */
