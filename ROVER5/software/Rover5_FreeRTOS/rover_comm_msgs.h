/*
 * rover_comm_msgs.h
 *
 *  Created on: 05/giu/2020
 *      Author: nbattezzati
 */

#ifndef ROVER_COMM_MSGS_H_
#define ROVER_COMM_MSGS_H_

/*---------------------------------FUNCTION PROTOTYPES----------------------------------*/
// FUNCTION: RoverSendMsg_MOTOR_TELEM()
// Description: function to send a message over the RF channel to a specific recipient
// Parameters:  - to: recipient address
//              - str: string containing the message to send
//              - len: length of the string (messages longer than COMM_MAX_MSG_SIZE will be truncated)
// Return:      bytes sent if message is successfully queued, < 0 if failed
int8_t RoverSendMsg_MOTOR_TELEM(uint8_t from, uint8_t to, int32_t v1, int32_t v2, int32_t c1, int32_t c2,  int32_t el1, int32_t el2, int32_t er1, int32_t er2);

// FUNCTION: RoverSendMsg_CONTROL_OUTPUT()
// Description: function to send a message over the RF channel to a specific recipient
// Parameters:  - to: recipient address
//              - str: string containing the message to send
//              - len: length of the string (messages longer than COMM_MAX_MSG_SIZE will be truncated)
// Return:      bytes sent if message is successfully queued, < 0 if failed
int8_t RoverSendMsg_CONTROL_OUTPUT(uint8_t from, uint8_t to, int32_t x, int32_t y, float t, int32_t lv, float av, int32_t xe, int32_t ye, float te);
/*--------------------------------------------------------------------------------------*/

#endif /* ROVER_COMM_MSGS_H_ */
