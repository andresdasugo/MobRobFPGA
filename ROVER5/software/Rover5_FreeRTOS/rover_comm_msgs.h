/*
 * rover_comm_msgs.h
 *
 *  Created on: 05/giu/2020
 *      Author: nbattezzati
 */

#ifndef ROVER_COMM_MSGS_H_
#define ROVER_COMM_MSGS_H_


int8_t RoverSendMsg_MOTOR_TELEM(uint8_t from, uint8_t to, int32_t v1, int32_t v2, int32_t c1, int32_t c2,  int32_t el1, int32_t el2, int32_t er1, int32_t er2);

int8_t RoverSendMsg_CONTROL_OUTPUT(uint8_t from, uint8_t to, int32_t x, int32_t y, float t, int32_t lv, float av, int32_t xe, int32_t ye, float te);




#endif /* ROVER_COMM_MSGS_H_ */
