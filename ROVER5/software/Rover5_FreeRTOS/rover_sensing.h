/*
 * rover_sensing.h
 *
 *  Created on: 2/04/2020
 *      Author: andre
 */

#ifndef ROVER_SENSING_H_
#define ROVER_SENSING_H_

// FUNCTION: TaskSensing()
// Description: main task of the sensing module.
//              Calculates the current position of the robot x,y,theta
// Parameters:  - pvParameters: pointer to task parameters (pass NULL, not used)
// Return:      -
void RoverTaskSensing(void *pvParameters);



#endif /* ROVER_SENSING_H_ */
