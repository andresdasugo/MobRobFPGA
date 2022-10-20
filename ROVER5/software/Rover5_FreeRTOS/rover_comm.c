#include <stdlib.h>			// for NULL definition
#include <string.h>			// for memcpy()
#include <stdbool.h>		// for bool type
#include <sys/types.h>		// for open() syscall
#include <sys/stat.h>		// for open() syscall
#include <fcntl.h>			// for open() syscall
#include <unistd.h>			// for write() syscall
#include <stdio.h>			// for fprintf()

#include <FreeRTOS.h>
#include <semphr.h>
#include <queue.h>
#include <task.h>

#include "rover_task.h"
#include "rover_comm.h"

#include "altera_avalon_uart.h"

/*---------------------------------------DEFINES----------------------------------------*/
#define COMM_MSG_TRAILING_STR			"\0\r"	// every message ends with this string
#define COMM_MSG_TRAILING_STR_SIZE		(2)		// size of the string that closes a message
#define COMM_MAX_MSG_SIZE				(1024)	// max number of bytes in a message

#define RX_BUF_SIZE						(1024)	// max number of bytes of the receiver buffer

#define MAX_QUEUE_LEN					(8)		// max number of messages in a queue
/*--------------------------------------------------------------------------------------*/

/*----------------------------------PUBLIC VARIABLES------------------------------------*/

/*--------------------------------------------------------------------------------------*/

/*-----------------------------------PRIVATE TYPES--------------------------------------*/

// TYPE: comm_msg_t
// Description: structure used to send a message over a queue
// Fields:      - len: length of the message
//              - buf: pointer to the buffer containing the message (dynamically allocated)
typedef struct {
	uint16_t	len;
	uint8_t *	buf;
} comm_msg_t;
/*--------------------------------------------------------------------------------------*/

/*---------------------------------PRIVATE VARIABLES------------------------------------*/
volatile uint32_t _msg_cnt = 0;
SemaphoreHandle_t	tx_queue_mutex_	= NULL;
QueueHandle_t		tx_queue_		= NULL;
QueueHandle_t		rx_queue_		= NULL;
/*--------------------------------------------------------------------------------------*/

/*---------------------------------FUNCTION PROTOTYPES----------------------------------*/
// FUNCTION: string_2_msg()
// Description: function to create a comm_msg_t from a string of characters (allocating the memory for the message)
uint16_t string_2_msg(const char * str, uint16_t len, comm_msg_t * msg);

// FUNCTION: parse_msg()
// Description: function to parse received bytes and check if a message can be constructed
uint16_t parse_msg(char * buf, unsigned int len, comm_msg_t * msg);

// FUNCTION: dispatch_msg()
// Description: function to dispatch a message to a receiving queue
int8_t dispatch_msg(comm_receiver_t receiver, comm_msg_t * msg);
/*--------------------------------------------------------------------------------------*/

/*----------------------------------PUBLIC FUNCTIONS------------------------------------*/
// FUNCTION: RoverSendMsg()
// Description: function to send a message over the RF channel to a specific recipient
int8_t RoverSendMsg(uint8_t to, const char * str, uint16_t len)
{
	int8_t ret_val = 0;
	comm_msg_t msg = {0};
	uint16_t target_len = 0;

	// check if the queue exists
	if (tx_queue_ != NULL) {
		// check if the length is ok, otherwise truncate the message
		target_len = (len < COMM_MAX_MSG_SIZE ? len : COMM_MAX_MSG_SIZE) + COMM_MSG_TRAILING_STR_SIZE;
		// create the message structure
		if (string_2_msg(str, target_len, &msg) > 0) {
			// lock the mutex to be sure that nobody else is using the queue
			xSemaphoreTake(tx_queue_mutex_, portMAX_DELAY);
			// queue the message if the queue is not full
			printf("[DEBUG] sending msg to queue: len=%d, msg=%s\n", msg.len, msg.buf);
			if (xQueueSend(tx_queue_, &msg, 0) != pdTRUE) {
				printf("[ERR] Failed to send message to queue\n");
				free(msg.buf);
				ret_val = COMM_ERR_QUEUE_FULL;
			}
			// release the mutex
			xSemaphoreGive(tx_queue_mutex_);
		}
		else if (len > 0) {
			ret_val = COMM_ERR_OUT_OF_MEMORY;
		}
		else {
			// len=0, do nothing
		}
	}
	else {
		printf("[ERR] Cannot send message to a NULL queue\n");
		ret_val = COMM_ERR_OUT_OF_MEMORY;
	}

	return ret_val;
}

// FUNCTION: RoverSendBroadcastMsg()
// Description: function to send a message over the RF channel to all listening recipients
int32_t RoverSendBroadcastMsg(const char * str, uint16_t len)
{
	// TODO RoverSendBroadcastMsg()
	return 0;
}


// FUCNTION: RegisterMsgReceiver()
// Description: function to register a receiver, an object that can be used to read messages
comm_receiver_t RoverRegisterMsgReceiver()
{
	comm_receiver_t ret_val = NULL;

	// allocate the RX queue if not already done
	if (rx_queue_ == NULL) {
		if ((rx_queue_ = xQueueCreate(MAX_QUEUE_LEN, sizeof(comm_msg_t))) != NULL) {
			ret_val = rx_queue_;
		}
		else {
			printf("[ERR] failed to create RX queue\n");
		}
	}
	else {
		printf("[WARN] message receiver already in place\n");
	}

	return ret_val;
}

// FUNCTION: RoverGetMsg()
// Description: function used to get a message (if any)
uint8_t RoverGetMsg(comm_receiver_t receiver, char ** str)
{
	uint8_t ret_val = 0;
	comm_msg_t rx_msg = {0};

	// check if there is any message waiting
	if ((receiver != NULL) && (uxQueueMessagesWaiting(receiver) > 0)) {
		// get the message from the queue
		if (xQueueReceive(receiver, &rx_msg, 0) == pdTRUE) {
			*str = (char *)rx_msg.buf;
			ret_val = rx_msg.len;
		}
		else {
			printf("[ERR] Failed to receive msg from RX queue\n");
		}
	}
	else {
		// no messages
	}

	return ret_val;
}

// FUNCTION: RoverReleaseMsg()
// Description: function to release the memory associated to a received message
void RoverReleaseMsg(char * str)
{
	if (str != NULL) {
		free(str);
	}
}

// FUNCTION: TaskComm()
// Description: main task of the communication module.
void RoverTaskComm(void *pvParameters)
{
	TickType_t last_wakeup_time = 0;
	int uart_tx_fd = 0;
	comm_msg_t tx_msg = {0};
	bool msg_valid = true;

	int uart_rx_fd = 0;
	int bytes_read = 0;
	char rx_buf[RX_BUF_SIZE] = {0};
	unsigned int rx_offset = 0;
	comm_msg_t rx_msg = {0};

	// Initialize the last_wakeup_time variable with the current time.
	last_wakeup_time = xTaskGetTickCount();

	// initialize the internal resources
	if ((tx_queue_mutex_ = xSemaphoreCreateMutex()) == NULL) {
		printf("[ERR] failed to create TX queue Mutex\n");
	}
	else {
		if ((tx_queue_ = xQueueCreate(MAX_QUEUE_LEN, sizeof(comm_msg_t))) == NULL) {
			printf("[ERR] failed to create TX queue\n");
		}
		else {
			// open the XBEE UART device in write mode
			if ((uart_tx_fd = open("/dev/XBEE", O_WRONLY)) < 0) {
				fprintf(stderr, "[ERR] Failed to open the XBee serial port in WR mode\n");
			}
			// open the XBEE UART device in read mode
			else if ((uart_rx_fd = open("/dev/XBEE", O_RDONLY | O_NONBLOCK)) < 0) {
				fprintf(stderr, "[ERR] Failed to open the XBee serial port in RD mode\n");
				if (uart_tx_fd != 0) close(uart_tx_fd);
			}
		}
	}
	// start the task
	while(1)
	{
		// loop over the messages to be sent
		while ((tx_queue_ != NULL) && (uxQueueMessagesWaiting(tx_queue_) > 0)) {
			msg_valid = true;
			// get a message from the queue
			xSemaphoreTake(tx_queue_mutex_, portMAX_DELAY);
			if (xQueueReceive(tx_queue_, &tx_msg, 0) != pdTRUE) {
				msg_valid = false;
				printf("[ERR] Failed to receive msg from TX queue\n");
			}
			xSemaphoreGive(tx_queue_mutex_);

			// write the message to the UART
			if (msg_valid == true) {
				printf("[DEBUG] writing message to UART: len=%d, msg=%s\n", tx_msg.len, tx_msg.buf);
				write(uart_tx_fd, tx_msg.buf, tx_msg.len);
				// release the memory for the buffer holding the message
				free(tx_msg.buf);
			}
		}
		// receive bytes from the UART and push new messages to the RX queue
		if ((bytes_read = read(uart_rx_fd, rx_buf+rx_offset, RX_BUF_SIZE-rx_offset)) > 0) {
			rx_offset += bytes_read;
			// parse the buffer to search and eventually dispatch complete messages
			while(parse_msg(rx_buf, rx_offset, &rx_msg) > 0) {
				// update number of valid bytes in the buffer
				rx_offset -= rx_msg.len;
				// dispatch message to the registered queues
				dispatch_msg(rx_queue_, &rx_msg);
			}
			if (rx_offset >= RX_BUF_SIZE) {
				printf("[WARN] receiving buffer is full, previous messages will be discarded\n");
				rx_offset = 0;
			}
		}
		else if (bytes_read < 0) {
			// TODO: check if the queue was not empty by errno
			//printf("[ERR] failed to read from UART RX\n");
		}
		else {
			// nothing to read
		}
		// Wait for the next cycle.
		vTaskDelayUntil( &last_wakeup_time, TASK_MS_2_TICKS(1) );
	}
	// close the UART File descriptors
	if (uart_tx_fd != 0) close(uart_tx_fd);
	if (uart_rx_fd != 0) close(uart_rx_fd);
}
/*--------------------------------------------------------------------------------------*/

/*---------------------------------PRIVATE FUNCTIONS------------------------------------*/

// FUNCTION: string_2_msg()
// Description: function to create a comm_msg_t from a string of characters (allocating the memory for the message)
// Parameters:  - str: string containing the message
//              - len: length of the message
//              - msg: pointer to a comm_msg_t that will be filled with the message
// Return:      number of characters copied into the message. If any error occurs return 0
uint16_t string_2_msg(const char * str, uint16_t len, comm_msg_t * msg)
{
	uint16_t ret_val = 0;

	// allocate the buffer
	if ((msg != NULL) && (len > 0) && ((msg->buf = malloc(len)) != NULL)) {
		// copy bytes from the string to the message buffer
		memcpy(msg->buf, str, len-COMM_MSG_TRAILING_STR_SIZE);
		// set the last character to end the string
		memcpy(msg->buf+len-COMM_MSG_TRAILING_STR_SIZE, COMM_MSG_TRAILING_STR, COMM_MSG_TRAILING_STR_SIZE);
		// set message length
		msg->len = len;
		// set return value
		ret_val = msg->len;

		printf("[DEBUG] string_2_msg: len=%d, msg=%s\n", msg->len, msg->buf);
	}
	else {
		printf("[ERR] failed to allocate memory for msg buffer (or msg is NULL)\n");
	}

	return ret_val;
}

// FUNCTION: parse_msg()
// Description: function to parse received bytes and check if a message can be constructed
// Parameters:  - buf: buffer containing the bytes received from the UART
//              - len: number of valid bytes within the buffer
//              - msg: pointer to a message structure to be filled with a parsed message
// Return:      number of bytes in the new message or 0 if no complete messages could be parsed
uint16_t parse_msg(char * buf, unsigned int len, comm_msg_t * msg)
{
	uint16_t ret_val = 0;
	int i = 0;
	int msg_size = RX_BUF_SIZE+1;

	// search for the termination string '\0\r'
	for (i=0; (i<((int)len-1)) && (msg_size>RX_BUF_SIZE); ++i) {
		if (buf[i] == '\0' && buf[i+1] == '\r') {
			msg_size = i+COMM_MSG_TRAILING_STR_SIZE;
		}
	}

	// if we've found a complete message
	if (msg_size <= RX_BUF_SIZE) {
		// allocate the new message and copy it from the buffer
		if ((msg->buf = malloc(msg_size * sizeof(char))) != NULL) {
			msg->len = msg_size;
			memcpy(msg->buf, buf, msg->len);
		}
		else {
			printf("[ERR] failed to allocate the buffer for RX message\n");
		}

		// move the bytes following the end of message to the beginning of the buffer
		memmove(buf, buf+msg_size, len-(msg_size));

		// update return value (number of bytes in the new message
		ret_val = msg->len;
	}

	return ret_val;
}

// FUNCTION: dispatch_msg()
// Description: function to dispatch a message to a receiving queue
// Parameters:  - receiver: pointer to the receiver queue where to dispatch the message
//              - msg: pointer to the message to be transmitted
// Return:      0 if msg has been successfully transmitted, < 0 if any error occurred
int8_t dispatch_msg(comm_receiver_t receiver, comm_msg_t * msg)
{
	int8_t ret_val = 0;

	if (receiver != NULL) {
		// lock the mutex to be sure that nobody else is using the queue
//		xSemaphoreTake(rx_queue_mutex_, portMAX_DELAY);
		// queue the message if the queue is not full
		printf("[DEBUG] copying msg to RX queue: len=%d, msg=%s\n", msg->len, msg->buf);
		if (xQueueSend(receiver, msg, 0) != pdTRUE) {
			printf("[ERR] Failed to copy message to RX queue\n");
			free(msg->buf);
			msg->len = 0;
			ret_val = COMM_ERR_QUEUE_FULL;
		}
		// release the mutex
//		xSemaphoreGive(rx_queue_mutex_);
	}
	else {
		printf("[ERR] Trying to dispatch message to a NULL receiver\n");
		ret_val = COMM_ERR_INVALID_RECEIVER;
	}

	return ret_val;
}
/*--------------------------------------------------------------------------------------*/