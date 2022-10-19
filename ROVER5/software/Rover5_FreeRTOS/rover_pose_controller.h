/*
 * rover_controllers.h
 *
 *  Created on: 3/06/2020
 *      Author: andre
 */

#ifndef ROVER_POSE_CONTROLLER_H_
#define ROVER_POSE_CONTROLLER_H_

#include "rover_pid.h"

// TYPE: rover_pose_t
// Description: structure used to define the pose (x,y,theta)
// Fields:      - x: position in the x-axis 
//              - y: position in the y-axis
//				- theta: angle of orientation (with respect to the x-axis)
typedef struct {
	float x;
	float y;
	float theta;
} rover_pose_t;

// TYPE: rover_velocities_t
// Description: structure used to define the linear and angular velocities
// Fields:      - v: linear velocity
//              - w: angular velocity
typedef struct {
	float v;
	float w;
} rover_velocities_t;

// TYPE: rover_pose_control_t
// Description: enumeration use to define the type of motion controller
// Fields:      - RoverPoseControl_PID: PID pose controller
//              - RoverPoseControl_Lyapunov: Lyapunov pose controller
typedef enum {
	RoverPoseControl_PID,
	RoverPoseControl_Lyapunov
} rover_pose_control_t;

// TYPE: rover_lyapunov_const_t
// Description: structure used to store the gains of the Lyapunov controller
// Fields:      - K1: Gain K1
//              - K2: Gain K2
//              - Q1: Gain Q1
//              - Q2: Gain Q2
typedef struct {
	float K1;
	float K2;
	float Q1;
	float Q2;
} rover_lyapunov_const_t;

// TYPE: rover_pid_params_t
// Description: structure used to store the parameters of the PID pose controller
// Fields:      - set_linear_velocity: Linear velocity set for the robot
//              - radius_to_stop: How close from the desired pose the robot can stop moving
//				- pid_const: PID controller parameters
typedef struct {
	float set_linear_velocity;
	float radius_to_stop;
	rover_PID_const_t pid_const;
} rover_pid_params_t;

// TYPE: rover_pose_control_params_t
// Description: union used to store the parameters of the pose controller
// Fields:      - lyapunov: Parameters of the Lyapunov controller
//              - pid: Parameters of the PID controller
typedef union {
	rover_lyapunov_const_t * lyapunov;
	rover_pid_params_t * pid;
} rover_pose_control_params_t;

// FUNCTION: SubtractPose()
// Description: function to compute the difference between two poses
// Parameters:  - pose_1: pose to be subtracted
//              - pose_2: pose to be subtracted
// Return:      difference between the two poses
rover_pose_t SubtractPose(const rover_pose_t * pose_1, const rover_pose_t * pose_2);

// FUNCTION: PoseController_Init()
// Description: function to clear the pose controller parameters
// Parameters:  - type: Type of controller
//				- params: Parameters of the pose controller
// Return:      N/A
void PoseController_Init(const rover_pose_control_t type, const rover_pose_control_params_t * params);

// FUNCTION: PoseController_Run()
// Description: function apply a pose controller to the robot
// Parameters:  - type: Type of controller
//              - params: Parameters of the pose controller
//              - pose: Current robot pose
//              - desired_pose: Desired robot pose
//              - pose_error: Difference between the Current and Desired robot pose
//				- velocities: linear and angular velocities for the robot motion
// Return:      bytes sent if message is successfully queued, < 0 if failed
bool PoseController_Run(
	const rover_pose_control_t type,
	const rover_pose_control_params_t * params,
	const rover_pose_t * pose,
	const rover_pose_t * desired_pose,
	rover_pose_t * pose_error,
	rover_velocities_t * velocities);

#endif /* ROVER_POSE_CONTROLLER_H_ */