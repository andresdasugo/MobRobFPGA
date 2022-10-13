/*
 * rover_comm.h
 *
 * This file contains the software module for the Rover communication
 *
 *  Created on: 25/mar/2020
 *      Author: nbattezzati
 */

#ifndef ROVER_COMM_H_
#define ROVER_COMM_H_

#include "rover_comm_msgs.h"

#include <queue.h>

#include "rover_comm_msgs.h"


#define COMM_ERR_OUT_OF_MEMORY		(-1)
#define COMM_ERR_QUEUE_FULL			(-2)
#define COMM_ERR_NO_MORE_RECEIVERS	(-3)
#define COMM_ERR_INVALID_RECEIVER	(-4)




typedef QueueHandle_t comm_receiver_t;


// FUNCTION: RoverSendMsg()
// Description: function to send a message over the RF channel to a specific recipient
// Parameters:  - to: recipient address
//              - str: string containing the message to send
//              - len: length of the string (messages longer than COMM_MAX_MSG_SIZE will be truncated)
// Return:      bytes sent if message is successfully queued, < 0 if failed
int8_t RoverSendMsg(uint8_t to, const char * str, uint16_t len);

// FUNCTION: RoverSendBroadcastMsg()
// Description: function to send a message over the RF channel to all listening recipients
// Parameters:  - str: string containing the message to send
//              - len: length of the string (messages longer than COMM_MAX_MSG_SIZE will be truncated)
// Return:      bytes sent if message is successfully queued, < 0 if failed
int32_t RoverSendBroadcastMsg(const char * str, uint16_t len);

// FUCNTION: RegisterMsgReceiver()
// Description: function to register a receiver, an object that can be used to read messages
// Parameters:  -
// Return:      the message receiver object or NULL if the registration failed
comm_receiver_t RoverRegisterMsgReceiver();

// FUNCTION: RoverGetMsg()
// Description: function used to get a message (if any)
// Parameters:  - receiver: the receiver object
//              - str: a pointer that will be filled with a new message
//                    (to delete after usage calling the RoverReleaseMsg() function)
// Return:      the number of bytes in the message or 0 if there are no messages
uint8_t RoverGetMsg(comm_receiver_t receiver, char ** str);

// FUNCTION: RoverReleaseMsg()
// Description: function to release the memory associated to a received message
// Parameters:  - str: the pointer to the received message
// Return:      -
void RoverReleaseMsg(char * str);

// FUNCTION: RoverTaskComm()
// Description: main task of the communication module.
//              It contains the main loop that handles the communication with the XBee UART
// Parameters:  - pvParameters: pointer to task parameters (pass NULL, not used)
// Return:      -
void RoverTaskComm(void *pvParameters);



#endif /* ROVER_COMM_H_ */
