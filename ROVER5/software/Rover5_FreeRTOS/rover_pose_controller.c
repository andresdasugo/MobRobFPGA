/*
 * rover_controllers.c
 *
 *  Created on: 3/06/2020
 *      Author: andre
 */

#include <math.h>
#include "rover_pose_controller.h"

////////////////////////////
///                      ///
///       DEFINES        ///
///                      ///
////////////////////////////

////////////////////////////
///                      ///
///   PUBLIC VARIABLES   ///
///                      ///
////////////////////////////

////////////////////////////
///                      ///
///   PRIVATE VARIABLES  ///
///                      ///
////////////////////////////

////////////////////////////
///                      ///
///    PUBLIC TYPES      ///
///                      ///
////////////////////////////

////////////////////////////
///                      ///
///    PRIVATE TYPES     ///
///                      ///
////////////////////////////

////////////////////////////
///                      ///
/// FUNCTION PROTOTYPES  ///
///                      ///
////////////////////////////

// FUNCTION: PIDPoseControl()
// Description: function to apply a PID controller for a Motion Controller
// Parameters:  - params: Parameters of the PID controller
//              - pose: Current robot pose
//              - desired_pose: Desired robot pose
//              - error_pose: Difference between the Current and Desired robot pose
//				- velocities: linear and angular velocities for the robot motion
// Return:      True if the robot gets to the desired pose, false otherwise
bool PIDPoseControl(
	rover_pid_params_t * params,
	const rover_pose_t * pose,
	const rover_pose_t * desired_pose,
	rover_pose_t * error_pose,
	rover_velocities_t * velocities
);

// FUNCTION: LyapunovControl_Init()
// Description: function to clear the Lyapunov controller parameters
// Parameters:  - params: Parameters for the Lyapunov Controller
// Return:      N/A
void LyapunovControl_Init (const rover_lyapunov_const_t * params);

// FUNCTION: LyapunovControl()
// Description: function to apply a Lyapunov stability criteria for a Motion Controller
// Parameters:  - params: Parameters of the PID controller
//              - pose: Current robot pose
//              - desired_pose: Desired robot pose
//              - error_pose: Difference between the Current and Desired robot pose
//				- velocities: linear and angular velocities for the robot motion
// Return:      True if the robot gets to the desired pose, false otherwise
bool LyapunovControl (
	const rover_lyapunov_const_t * params,
	const rover_pose_t * pose,
	const rover_pose_t * desired_pose,
	rover_pose_t * error_pose,
	rover_velocities_t * velocities
);

////////////////////////////
///                      ///
///   PUBLIC FUNCTIONS   ///
///                      ///
////////////////////////////

// FUNCTION: SubtractPose()
// Description: function to compute the difference between two poses
rover_pose_t SubtractPose(const rover_pose_t * pose_1, const rover_pose_t * pose_2){

	rover_pose_t result = {0};

	if (pose_1 && pose_2){
		result.x = pose_1->x - pose_2->x;
		result.y = pose_1->y - pose_2->y;
		result.theta = pose_1->theta - pose_2->theta;
	}

	return result;
}

// FUNCTION: PoseController_Init()
// Description: function to clear the pose controller parameters
void PoseController_Init(rover_pose_control_t type, const rover_pose_control_params_t * params)
{
	switch(type) {
	/* Initialize the parameters of the PID pose controller */
	case RoverPoseControl_PID:
		PIDControl_Init(&(params->pid->pid_const));
	break;
	/* Initialize the parameters of the Lyapunov controller*/
	case RoverPoseControl_Lyapunov:
		LyapunovControl_Init(params->lyapunov);
	break;
	/* Otherwise generate an error*/
	default:
		// TODO: error
	break;
	}
}

// FUNCTION: RoverSendMsg()
// Description: function to send a message over the RF channel to a specific recipient
bool PoseController_Run(
	const rover_pose_control_t type,
	const rover_pose_control_params_t * params,
	const rover_pose_t * pose,
	const rover_pose_t * desired_pose,
	rover_pose_t * pose_error,
	rover_velocities_t * velocities
)
{
	switch(type) {
	/* Use the PID pose controller */
	case RoverPoseControl_PID:
		return PIDPoseControl(params->pid, pose, desired_pose, pose_error, velocities);
	break;
	/* Use the Lyapunov controller*/
	case RoverPoseControl_Lyapunov:
		return LyapunovControl(params->lyapunov, pose, desired_pose, pose_error, velocities);
	break;
	/* Otherwise generate an error*/
	default:
		// TODO: error
		return false;
	break;
	}

	return false;

}

////////////////////////////
///                      ///
///  PRIVATE FUNCTIONS   ///
///                      ///
////////////////////////////

// FUNCTION: PIDPoseControl()
// Description: function to apply a PID controller for a Motion Controller
bool PIDPoseControl(
	rover_pid_params_t * params,
	const rover_pose_t * pose,
	const rover_pose_t * desired_pose,
	rover_pose_t * error_pose,
	rover_velocities_t * velocities
)
{
	float angle_error = 0;
	bool is_angle = true;

	* error_pose = SubtractPose(desired_pose, pose);
	float goal_theta = atan2f(error_pose->y, error_pose->x); //Angle from robot to goal
	float distance_to_goal = sqrtf((powf((error_pose->x),2))+(powf((error_pose->y),2))); //Distance from robot to goal

	if (distance_to_goal < params->radius_to_stop){ //If the robot is close enough
		velocities->v = 0;
		velocities->w = PIDControl(&(params->pid_const), desired_pose->theta, pose->theta, &angle_error, is_angle);
		if (angle_error < 0.2){
			velocities->w = 0;
			return true;
		}
	}else{
		velocities->v = params->set_linear_velocity;
		velocities->w = PIDControl(&(params->pid_const), goal_theta, pose->theta, &angle_error, is_angle);
	}

	return false;
}

// FUNCTION: LyapunovControl_Init()
// Description: function to clear the Lyapunov controller parameters
void LyapunovControl_Init (const rover_lyapunov_const_t * params)
{
	// nothing to init
}

// FUNCTION: LyapunovControl()
// Description: function to apply a Lyapunov stability criteria for a Motion Controller
bool LyapunovControl (
	const rover_lyapunov_const_t * params,
	const rover_pose_t * pose,
	const rover_pose_t * desired_pose,
	rover_pose_t * error_pose,
	rover_velocities_t * velocities
)
{
	float l;
	float zeta;
	float psi;

	/* Compute the error in the pose */
	* error_pose = SubtractPose(desired_pose, pose);

	/* Compute the variables in the controller*/
	l = sqrtf((powf((error_pose->x),2))+(powf((error_pose->y),2)));
	zeta = atan2f((error_pose->y),(error_pose->x))- pose->theta;
	psi = atan2f((error_pose->y),(error_pose->x))- desired_pose->theta;

	/* Compute the linear and angular velocities */
	velocities->v = params->K1 * l * ((cosf(zeta))+(tanf(zeta)*tanf(zeta)*cos(zeta)));
	velocities->w = (params->K2 * zeta) + ((1 + ((params->Q2 * psi) / zeta)) * (params->K1 * sinf(zeta)) * ((cos(zeta))+(tan(zeta)*tanf(zeta)*cosf(zeta))));

	return true;
}
