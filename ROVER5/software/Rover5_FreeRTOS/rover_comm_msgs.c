#include <stdio.h>
#include <stdint.h>

/*---------------------------------------DEFINES----------------------------------------*/
#define COMM_MSG_HEADER_LEN			(64)	// minimum header len is 52 character as here below:
											// {"from":01,"to":00,"cnt":00001,"type":00,"body":{}}
#define COMM_MSG_LEN_MOTOR_TELEM	(32)
#define COMM_MSG_LEN_CONTROL_OUTPUT	(128)
/*--------------------------------------------------------------------------------------*/

/*---------------------------------PRIVATE VARIABLES------------------------------------*/
volatile uint32_t _comm_msg_cnt = 0;
/*--------------------------------------------------------------------------------------*/

/*---------------------------------FUNCTION PROTOTYPES----------------------------------*/
int8_t RoverSendMsg(uint8_t to, const char * str, uint16_t len);
/*--------------------------------------------------------------------------------------*/

/*----------------------------------PUBLIC FUNCTIONS------------------------------------*/
// TYPE: comm_msg_type_t
// Description: structure used to enumerate the types of messages to send over the RF channel
// Fields:      - CommMsgTpe_MOTOR_TELEM: Telemetry of the motors
//              - CommMsgTpe_SENSOR_TELEM: Telemetry of the sensors
//              - CommMsgTpe_LIDAR: LiDaR information
//              - CommMsgTpe_CONTROL_OUTPUT: Information of the control being applied to the mobile robot
typedef enum {
	CommMsgTpe_MOTOR_TELEM			= 0,
	CommMsgTpe_SENSOR_TELEM			= 1,
	CommMsgTpe_LIDAR				= 2,
	CommMsgTpe_CONTROL_OUTPUT		= 3,
} comm_msg_type_t;


// FUNCTION: RoverSendMsg()
// Description: function to send a message over the RF channel to a specific recipient
int8_t RoverSendMsg_MOTOR_TELEM(uint8_t from, uint8_t to, int32_t v1, int32_t v2, int32_t c1, int32_t c2, int32_t el1, int32_t el2, int32_t er1, int32_t er2)
{
	char _msg_str[COMM_MSG_HEADER_LEN+COMM_MSG_LEN_MOTOR_TELEM];
	uint16_t len = sprintf(_msg_str,
			"{\"from\":%d,\"to\":%d,\"cnt\":%d,\"type\":%d,\"body\":{\"v\":[%d,%d],\"c\":[%d,%d],\"e\":[%d,%d,%d,%d]}}",
			from, to, _comm_msg_cnt++, CommMsgTpe_MOTOR_TELEM,
			v1, v2, c1, c2, el1, el2, er1, er2);
	return RoverSendMsg(to, _msg_str, len);
}

// FUNCTION: RoverSendMsg()
// Description: function to send a message over the RF channel to a specific recipient
int8_t RoverSendMsg_CONTROL_OUTPUT(uint8_t from, uint8_t to, int32_t x, int32_t y, float t, int32_t lv, float av, int32_t xe, int32_t ye, float te)
{
	char _msg_str[COMM_MSG_HEADER_LEN+COMM_MSG_LEN_CONTROL_OUTPUT];
	uint16_t len = sprintf(_msg_str,
			"{\"from\":%d,\"to\":%d,\"cnt\":%d,\"type\":%d,\"body\":{\"x\":%d,\"y\":%d,\"t\":%.6f,\"lv\":%d,\"av\":%.6f,\"xe\":%d,\"ye\":%d,\"te\":%.6f}}",
			from, to, _comm_msg_cnt++, CommMsgTpe_MOTOR_TELEM,
			x, y, t, lv, av, xe, ye, te);
	return RoverSendMsg(to,_msg_str,len);
}
/*--------------------------------------------------------------------------------------*/