#include <stdio.h>

#include "FreeRTOS.h"
#include "task.h"

#include "rover_task.h"
#include "rover_comm.h"

#include "rover_pose_controller.h"

#include <math.h> 			// for the math functions
#include <stdint.h>			// for the uint types
#include <stdbool.h>		// for the bool types

/* System specific includes. */
#include "io.h"										// for reading or writing to I/O
#include "system.h"									// for the configuration and register of the QSYS system
#include "sys/alt_irq.h"
#include "altera_up_avalon_de0_nano_adc.h"
#include "altera_up_avalon_accelerometer_spi.h"
#include "altera_avalon_pio_regs.h"
#include "altera_avalon_timer_regs.h"


/*---------------------------------------DEFINES----------------------------------------*/
/* Constants defined by the mobile robot model */
#define R_WHEEL_ONLY		30						//radius of the wheel in mm
#define R_TRACKED			117.5					//radius of the belt in mm
#define N					343						//number of ticks per revolution in the encoder 1000/3
#define L_WHEEL_SEPARATION	190						//wheel separation in mm
#define MM_PER_TICK			(2*M_PI*R_WHEEL_ONLY)/N	//mm traveled each tick
/* Print Status Options */
#define TELEMETRY_VALUES	0x00
#define ODOMETRY_VALUES		0x01
/*--------------------------------------------------------------------------------------*/

/*----------------------------------PUBLIC VARIABLES------------------------------------*/

/*--------------------------------------------------------------------------------------*/

/*---------------------------------PRIVATE VARIABLES------------------------------------*/
/* FPGA variables */
unsigned char led_val = 0; //to control de LEDs of the FPGA Board
unsigned char sw_val = 0; //to get the values of the SWITCHES from the FPGA Board
/* variable to store theta before the angle */
float theta_before = 0;
/*--------------------------------------------------------------------------------------*/

/*-----------------------------------PRIVATE TYPES--------------------------------------*/
// TYPE: rover_encoders_t
// Description: structure used to store the average values of the encoders in the robot
// Fields:      - right: value of the encoders in the right wheel
//              - left:  value of the encoders in the left wheel
typedef struct {
	int8_t right;
	int8_t left;
} rover_encoders_t;

// TYPE: rover_PWM_t
// Description: structure used to send the PWM signals to the wheels
// Fields:      - right: value of the PWM signal for the right wheel
//              - left:  value of the PWM signal for the left wheel
typedef struct {
	int8_t right;
	int8_t left;
} rover_PWM_t;

// TYPE: rover_encoders_raw_t
// Description: structure used to store the raw values of the four encoders in the robot
// Fields:      - r1: value of the encoder in the first right wheel 
//              - r2: value of the encoder in the second right wheel 
//              - l1: value of the encoder in the first left wheel 
//              - l2: value of the encoder in the second left wheel 
typedef struct {
	int8_t r1;
	int8_t r2;
	int8_t l1;
	int8_t l2;
} rover_encoders_raw_t;

// TYPE: rover_wheel_velocities_t
// Description: structure used to send the values of the velocities of each wheel to the robot
// Fields:      - right: velocity for the right wheels
//              - left:  velocity for the left wheels
typedef struct {
	float right;
	float left;
} rover_wheel_velocities_t;

// TYPE: rover_direction_t
// Description: enumeration used to store the hex values for the dH bridges tha controls the direction of movement of the robot
// Fields:      - RoverDirection_Forward:	Hex value for the robot to move forward
//              - RoverDirection_Right:		Hex value for the robot to move right
//              - RoverDirection_Left:		Hex value for the robot to move left
//              - RoverDirection_Backwards: Hex value for the robot to move backwards
typedef enum{
	RoverDirection_Forward 		= 0x35,
	RoverDirection_Right 		= 0x36,
	RoverDirection_Left 		= 0x39,
	RoverDirection_Backwards 	= 0x3A
} rover_direction_t;
/*--------------------------------------------------------------------------------------*/

/*---------------------------------FUNCTION PROTOTYPES----------------------------------*/
// FUNCTION: SetMobileRobotVelocity()
// Description: function that sends the PWM signal to the wheels according to the linear and angular velocities required by the motion controller
// Parameters:  - velocities: linear and angular velocities of the robot needed for the robot
//              - wheel_velocities: velocity of the wheels according to the needed linear and angular velocities
//              - measured_wheel_velocities: wheel velocities measured by the UpdatedOdometry() function
//              - direction: direction needed for the robot movement
//              - PWM: PWM value to send to the wheels
//              - pid_left_wheel: PID parameters for the PID controller in the left wheel
//              - pid_right_wheel: PID parameters for the PID controller in the right wheel
// Return:      N/A
void SetMobileRobotVelocity(rover_velocities_t * velocities, rover_wheel_velocities_t * wheel_velocities, rover_wheel_velocities_t * measured_wheel_velocities, rover_direction_t * direction, rover_PWM_t * PWM, rover_PID_const_t pid_left_wheel, rover_PID_const_t pid_right_wheel);

// FUNCTION: UpdateOdometry()
// Description: function to compute the pose (x, y, theta) of the robot according to the encoder data
// Parameters:  - pose: x, y, theta pose of the robot according to the encoder data
//              - delta_enc: change in the value of the encoders
//              - enc_raw: raw data from the encoders in the wheel of the robot
//              - measured_wheel_velocities: velocities of the wheels according to the encoder data
// Return:      N/A
void UpdateOdometry(rover_pose_t * pose, rover_encoders_t * delta_enc, rover_encoders_raw_t * enc_raw, rover_wheel_velocities_t * measured_wheel_velocities);

// FUNCTION: irqkey()
// Description: function to define the interruptions of the FPGA board buttons
// Parameters:  - context: context of the interruption
//              - id: id of the interruption
// Return:      N/A
static void irqkey (void * context, alt_u32 id);

// FUNCTION: PrintStatus()
// Description: function to send a message over the RF channel to a specific recipient with information of current status of the robot
// Parameters:  - opt_to_print: the kind of information to be sended
//              - pose: current pose of the robot
//              - pose_error: current error in the pose of the robot with respect of the controller
//              - enc: raw value of the encoders
//              - velocities: linear and angular velocities of the robot needed for the robot
//              - wheel_velocities: velocity of the wheels according to the needed linear and angular velocities
// Return:      N/A
void PrintStatus (uint8_t opt_to_print, rover_pose_t * pose, rover_pose_t * pose_error, rover_encoders_raw_t * enc, rover_velocities_t * velocities, rover_wheel_velocities_t * wheel_velocities);
/*--------------------------------------------------------------------------------------*/

/*----------------------------------PUBLIC FUNCTIONS------------------------------------*/
// FUNCTION: RoverTaskControl()
// Description: main task of the control module.
void RoverTaskControl(void *pvParameters)
{
	uint8_t msg_period = 0;

	TickType_t last_wakeup_time = 0;

	// Initialize the last_wakeup_time variable with the current time.
	last_wakeup_time = xTaskGetTickCount();

	rover_pose_control_params_t params;

	/* command reception variables */ 
	comm_receiver_t cmd_receiver	= NULL;
	char * command_str				= NULL;

	//printf("Robot ID = 0 - ADSG 2\n");

	/* Set the initial robot position (x, y, theta) */
	rover_pose_t pose = {0, 0, 0};

	/* Set the robot goal pose (x_d, y_d, theta_d) */
	rover_pose_t desired_pose = {1000, 1000, M_PI_2};

	/* rover pose error (output from the controller) */
	rover_pose_t pose_error;

	/* Initialise PID pose controller */
						   // Kp , Ki , Kd , Tau , Lim_min, Lim_Max, Tm
	rover_PID_const_t pid = { 20 , 0  , 0  , 0.01,      0 ,    100 , 0.1};
								// set_linear_velocity, radius_to_stop, pid
	rover_pid_params_t pid_pose = {(250*2*R_WHEEL_ONLY), 10, pid};
	params.pid = &pid_pose;
	PoseController_Init(RoverPoseControl_PID, &params);
	bool is_desired_pose = true;

	/* Initialise Lyapunov controller */
	//rover_lyapunov_const_t lyapunov = {1, 1, 1, 1};
	//params.lyapunov = &lyapunov;
	//PoseController_Init(RoverPoseControl_Lyapunov, &params);

	/* Set the constants for the Lyapunov Control function (K1, K2, Q1, Q2) */
	//rover_lyapunov_const_t lyapunov_const = {0.5, 0.5, 0, 0.5};

	/* Initialize the direction of the robots motion */
	rover_direction_t direction = RoverDirection_Forward;

	/* Initialize the linear and angular velocities */
	rover_velocities_t velocities = {100*R_WHEEL_ONLY, 0};

	/* Initialize the wheel velocities */
	rover_wheel_velocities_t wheel_velocities = {0};

	/* Initialize the measured wheel velocities */
	rover_wheel_velocities_t measured_wheel_velocities = {0};

	/* Initialize the wheel PWMs */
	rover_PWM_t PWM = {0};

	/* Initialize PID constants for wheel velocity control */
	   	   	   	   	   	   	   	   	   // Kp , Ki , Kd , Tau , Lim_min, Lim_Max, Tm
	rover_PID_const_t pid_right_wheel = { 20 , 0  , 0  , 0.01,      0 ,    255 , 0.1};
						   	   	   	   // Kp , Ki , Kd , Tau , Lim_min, Lim_Max, Tm
	rover_PID_const_t pid_left_wheel  = { 20 , 0  , 0  , 0.01,      0 ,    255 , 0.1};

	/* Create a variable that holds the cumulative variation of the encoders averaged on the two wheels */
	rover_encoders_t delta_enc = {0};
	/* ... and also the raw values for each encoder */
	rover_encoders_raw_t enc_raw = {0};

	/* Register the FPGA Key interrupt service routine */
	IOWR_ALTERA_AVALON_PIO_IRQ_MASK(KEY_BASE, 0x3);				//KEY FPGA
	IOWR_ALTERA_AVALON_PIO_EDGE_CAP(KEY_BASE, 0x0);
	alt_irq_register( KEY_IRQ, NULL,(void*)irqkey );


	/* Register listener for commands */ 
	cmd_receiver = RoverRegisterMsgReceiver();

	/* Runs continuously */
	while(1){

		//printf("T_control RUNNING\n");

		/* check if there is a new command */ 
		/*if (RoverGetMsg(cmd_receiver, &command_str) > 0) {
			printf("New COMMAND received: %s\n", command_str);
			RoverReleaseMsg(command_str);
		}*/

		/* Compute the pose of the robot according to the encoder values*/
		UpdateOdometry(&pose, &delta_enc, &enc_raw, &measured_wheel_velocities);

		/* Run the controller algorithm until the robot is in the desired pose*/
		//is_desired_pose = PoseController_Run(RoverPoseControl_PID, &params, &pose, &desired_pose, &pose_error, &velocities);
		/*if(is_desired_pose){
			// There is a new command?
			// Compute new pose
			// Start again - is_desired_pose = false;
		}else{
			// Set the desired control function
			is_desired_pose = PoseController_Run(RoverPoseControl_PID, &params, &pose, &desired_pose, &pose_error, &velocities);
		}
	 	*/

	 	/* Set the velocities of the wheels according to the motion controller*/
		SetMobileRobotVelocity(&velocities, &wheel_velocities, &measured_wheel_velocities, &direction, &PWM, pid_left_wheel, pid_right_wheel);

		/* Define an specific direction and PWM values for the robot */
		//IOWR(MOTORES_BASE,0,RoverDirection_Forward);
		//IOWR(PWM2_BASE,0,40); //PWM2 -> Left  wheel
		//IOWR(PWM1_BASE,0,100); //PWM1 -> Right wheel

		/* Wait for 10 control periods to send a message */
		if (msg_period == 5){
			PrintStatus(ODOMETRY_VALUES, &pose, &pose_error, &enc_raw, &velocities, &wheel_velocities);
			msg_period = 0;
		}
		++msg_period;

		/* Wait for the next cycle. */
		vTaskDelayUntil( &last_wakeup_time, TASK_MS_2_TICKS(RoverTaskControl_PERIOD));

	}

}
/*--------------------------------------------------------------------------------------*/

/*---------------------------------PRIVATE FUNCTIONS------------------------------------*/
// FUNCTION: SetMobileRobotVelocity()
// Description: function that sends the PWM signal to the wheels according to the linear and angular velocities required by the motion controller
void SetMobileRobotVelocity(rover_velocities_t * velocities, rover_wheel_velocities_t * wheel_velocities, rover_wheel_velocities_t * measured_wheel_velocities, rover_direction_t * direction, rover_PWM_t * PWM, rover_PID_const_t pid_left_wheel, rover_PID_const_t pid_right_wheel){

	/* Compute the wheel velocities according to the robot model and the desired linear and angular velocities*/
	wheel_velocities->left  = (((2*(velocities->v)) - (velocities->w*L_WHEEL_SEPARATION))/(2*R_WHEEL_ONLY));
	wheel_velocities->right = (((2*(velocities->v)) + (velocities->w*L_WHEEL_SEPARATION))/(2*R_WHEEL_ONLY));

	/* Compute the robot direction according to the wheel velocities */
	if (wheel_velocities->left >= 0 && wheel_velocities->right >= 0){
		*direction = RoverDirection_Forward;
	} else if (wheel_velocities->left <= 0 && wheel_velocities->right <= 0){
		wheel_velocities->left = fabs(wheel_velocities->left);
		wheel_velocities->right = fabs(wheel_velocities->right);
		*direction = RoverDirection_Backwards;
	} else if (wheel_velocities->left >= 0 && wheel_velocities->right <= 0){
		wheel_velocities->right = fabs(wheel_velocities->right);
		*direction = RoverDirection_Right;
	} else if (wheel_velocities->left <= 0 && wheel_velocities->right >= 0){
		wheel_velocities->left = fabs(wheel_velocities->left);
		*direction = RoverDirection_Left;
	} else {
		*direction = RoverDirection_Forward;
	}

	/* Use a PID controller to compute the PWM values of the wheels*/
	//velocities->w = PIDControl(&(params->pid_const), desired_pose->theta, pose->theta, &angle_error, is_angle);
	PWM->right = PIDControl(&pid_right_wheel, wheel_velocities->right, measured_wheel_velocities->right, 0, false);
	PWM->left  = PIDControl(&pid_left_wheel,  wheel_velocities->left,  measured_wheel_velocities->left,  0, false);

	/* Use an open loop controller to compute the PWM values of the wheels*/
	/* These equations take into account the differences in wheel velocities */
	/*if (wheel_velocities->right <= 170){
		PWM->right = 0;
	} else if (wheel_velocities->right > 170 && wheel_velocities->right <= 320){
		PWM->right = (0.327 * wheel_velocities->right) + 53.25;
	} else if (wheel_velocities->right > 320){
		PWM->right = 160;
	} else {
		PWM->right = 0;
	}

	if (wheel_velocities->left <= 170){
		PWM->left = 0;
	} else if (wheel_velocities->left > 170 && wheel_velocities->left <= 320){
		PWM->left = (0.4444 * wheel_velocities->left) + 5.9675;
	} else if (wheel_velocities->left > 320){
		PWM->left = 150;
	} else {
		PWM->left = 0;
	}
	*/
	/*
	 * These equations are assuming both wheels have the same velocities with the same PWM values
	 */
	/*
	if (wheel_velocities->right <= 140){
		PWM->right = 0;
	} else if (wheel_velocities->right > 140 && wheel_velocities->right <= 180){
		PWM->right = (0.7296 * wheel_velocities->right) - 21.009;
	} else if (wheel_velocities->right > 180 && wheel_velocities->right <= 350){
		PWM->right = (0.3757 * wheel_velocities->right) + 35.034;
	} else if (wheel_velocities->right > 350 && wheel_velocities->right <= 400){
		PWM->right = (1.6278 * wheel_velocities->right) - 399.32;
	} else if (wheel_velocities->right > 400){
		PWM->right = 255;
	} else {
		PWM->right = 0;
	}

	if (wheel_velocities->left <= 140){
		PWM->left = 0;
	} else if (wheel_velocities->left > 140 && wheel_velocities->left <= 180){
		PWM->left = (0.7296 * wheel_velocities->left) - 21.009;
	} else if (wheel_velocities->left > 180 && wheel_velocities->left <= 350){
		PWM->left = (0.3757 * wheel_velocities->left) + 35.034;
	} else if (wheel_velocities->left > 350 && wheel_velocities->left <= 400){
		PWM->left = (1.6278 * wheel_velocities->left) - 399.32;
	} else if (wheel_velocities->left > 400){
		PWM->left = 255;
	} else {
		PWM->left = 0;
	}
	*/

	/* Set the robot direction and send the PWM values to the wheel motors */
	IOWR(MOTORES_BASE,0,*direction);
	IOWR(PWM1_BASE,0,PWM->right);
	IOWR(PWM2_BASE,0,PWM->left);
}

// FUNCTION: UpdateOdometry()
// Description: function to compute the pose (x, y, theta) of the robot according to the encoder data
void UpdateOdometry(rover_pose_t * pose, rover_encoders_t * delta_enc, rover_encoders_raw_t * enc_raw, rover_wheel_velocities_t * measured_wheel_velocities)
{

	rover_pose_t pose_dt = {0,0,0};

	/* Odometry variables */
	float Dr=0;
	float Dl=0;
	float Dc=0;

	/* Read the values from the encoders */
	volatile uint32_t encoder_ip = IORD(ENCODER_READING_BASE,0);
	enc_raw->r2 = ((encoder_ip>>0)  & 0x000000FF);
	enc_raw->r1 = ((encoder_ip>>8)  & 0x000000FF);
	enc_raw->l2  = ((encoder_ip>>16) & 0x000000FF);
	enc_raw->l1  = ((encoder_ip>>24) & 0x000000FF);

	//printf("Encoder ip value: 0x%.8x \n", encoder_ip);
	//printf("Encoder values: l1 %d, l2 %d, r1 %d r2 %d \n", cur_left_enc1, cur_left_enc2, cur_right_enc1, cur_right_enc2);

	/* Update the cumulative encoders */
	delta_enc->right	= enc_raw->r2;
	delta_enc->left 	= - enc_raw->l2;

	/* Compute Dl, Dr and Dc */
	Dr = MM_PER_TICK * delta_enc->right;
	Dl = MM_PER_TICK * delta_enc->left;
	Dc = ((Dr+Dl)/2);

	/* Compute the wheel velocities using Dr and Dl */
	measured_wheel_velocities->right = 10 * Dr;
	measured_wheel_velocities->left  = 10 * Dl;

	/* Compute the x_dt, y_dt, theta_dt values from Dr, Dl, Dc */
	pose_dt.x = Dc*cosf(pose->theta);
	pose_dt.y = Dc*sinf(pose->theta);
	pose_dt.theta = ((Dr-Dl)/L_WHEEL_SEPARATION);

	/* Compute the new values x_new, y_new, theta_new for the robot pose */
	pose->x += pose_dt.x;
	pose->y += pose_dt.y;
	theta_before += pose_dt.theta;
	pose->theta = (0.5107 *(theta_before)) + 0.0735;
}

// FUNCTION: PrintStatus()
// Description: function to send a message over the RF channel to a specific recipient with information of current status of the robot
void PrintStatus (
	uint8_t opt_to_print,
	rover_pose_t * pose,
	rover_pose_t * pose_error,
	rover_encoders_raw_t * enc,
	rover_velocities_t * velocities,
	rover_wheel_velocities_t * wheel_velocities)
{
	/* Send information about the telemetry values */
	if (opt_to_print == TELEMETRY_VALUES){
		RoverSendMsg_MOTOR_TELEM(1,0,
			wheel_velocities->left, wheel_velocities->right,
			0, 0, /* TODO: get motor currents */
			enc->l1, enc->l2, enc->r1, enc->r2);
	}
	/* Send information about the odometry values */
	else if (opt_to_print == ODOMETRY_VALUES){
		RoverSendMsg_CONTROL_OUTPUT(1,0,
			pose->x, pose->y, pose->theta,
			velocities->v,velocities->w,
			pose_error->x, pose_error->y, pose_error->theta);
	}
	else {
		printf("[ERR] Unknown print option (%d)!\n", opt_to_print);
	}
}

/*
 * Interrupt Functions
 */

// FUNCTION: irqkey()
// Description: function to define the interruptions of the FPGA board buttons
static void irqkey (void * context, alt_u32 id){
	led_val = 0;
	IOWR_ALTERA_AVALON_PIO_EDGE_CAP(KEY_BASE, 0);
	IOWR_ALTERA_AVALON_PIO_IRQ_MASK(KEY_BASE, 0x3);
	IOWR_16DIRECT(KEY_BASE, 0, 0); // reset request
}
/*--------------------------------------------------------------------------------------*/
