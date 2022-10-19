/*
 * rover_sensing.c
 *
 *  Created on: 2/04/2020
 *      Author: andre
 */

#include <stdio.h>

#include "FreeRTOS.h"
#include "task.h"

#include "rover_task.h"

#include "io.h"							// for reading or writing to I/O
#include "system.h"						// for the configuration and register of the QSYS system
#include "sys/alt_irq.h"
#include "altera_avalon_uart.h"
#include "altera_avalon_uart_regs.h"

/*----------------------------------PUBLIC VARIABLES------------------------------------*/

/*--------------------------------------------------------------------------------------*/

/*-----------------------------------PRIVATE TYPES--------------------------------------*/
/* Ultrasonic sensor variables */
//unsigned char buffer1[5]={0}, buffer2[5]={0}, buffer3[5]={0}, buffer4[5]={0}, buffer5[5]={0}, buffer6[5]={0}, buffer7[5]={0}, buffer8[5]={0};
//unsigned char puntero1 = 0, puntero2 = 0, puntero3 = 0, puntero4 = 0, puntero5 = 0, puntero6 = 0, puntero7 = 0, puntero8 = 0;
unsigned int sensordist1=0, sensordist2=0, sensordist3=0, sensordist4 = 0, sensordist5=0, sensordist6=0, sensordist7=0, sensordist8 = 0;
/*--------------------------------------------------------------------------------------*/

/*-----------------------------------PRIVATE TYPES--------------------------------------*/

/*--------------------------------------------------------------------------------------*/

/*---------------------------------FUNCTION PROTOTYPES----------------------------------*/
// FUNCTION: InstallUltrasonicInterrupts()
// Description: function to define the interrupts from the ultrasonic sensors
// Parameters:  N/A
// Return:      N/A
void InstallUltrasonicInterrupts();
/*--------------------------------------------------------------------------------------*/

/*----------------------------------PUBLIC FUNCTIONS------------------------------------*/
// FUNCTION: RoverTaskSensing()
// Description: main task of the sensing module.
void RoverTaskSensing(void *pvParameters)
{
	TickType_t last_wakeup_time = 0;

	// Initialize the last_wakeup_time variable with the current time.
	last_wakeup_time = xTaskGetTickCount();

	while(1)
	{
		printf("T_sensing RUNNING\n");

		// Wait for the next cycle.
		vTaskDelayUntil( &last_wakeup_time, TASK_MS_2_TICKS(1000) );
	}
}
/*--------------------------------------------------------------------------------------*/

/*---------------------------------PRIVATE FUNCTIONS------------------------------------*/
/* Interrupt Functions */
// FUNCTION: isrdist1()
// Description: function to read the values from ultrasonic sensor 1 using UART
// Parameters:  - context
//				- id
// Return:      N/A
// Note: 		Sensor communication is RS232, output is an ASCII 'R' followed by three ASCII representing the distance value 
//				in inches and ending with an ASCII 13 (CR or Enter). Baud=9600, 8 bits, no parity with a stop bit.
static void isrdist1 (void * context, alt_u32 id){

	unsigned char buffer[5];
	unsigned char puntero;

	unsigned char test;
	test = IORD_ALTERA_AVALON_UART_RXDATA(DIST1_BASE);

	if (test == 'R'){
		puntero = 0;
		buffer[puntero1] = test;
		puntero++;
	}
	else{
		buffer[puntero] = test - 48;
		puntero++;
	}

	if (test == 13){
		sensordist1 = (buffer[1]*1000 + buffer[2]*100 + buffer[3]*10 + buffer[4]); //ASCII  a binario
		//sensordist1 = (buffer1[1]*100 + buffer1[2]*10 + buffer1[3]);
	}
}

// FUNCTION: isrdist2()
// Description: function to read the values from ultrasonic sensor 2 using UART
// Parameters:  - context
//				- id
// Return:      N/A
// Note: 		Sensor communication is RS232, output is an ASCII 'R' followed by three ASCII representing the distance value 
//				in inches and ending with an ASCII 13 (CR or Enter). Baud=9600, 8 bits, no parity with a stop bit.
static void isrdist2 (void * context, alt_u32 id){

	unsigned char buffer[5];
	unsigned char puntero;

	unsigned char test;
	test = IORD_ALTERA_AVALON_UART_RXDATA(DIST2_BASE);

	if (test == 'R'){
		puntero = 0;
		buffer[puntero] = test;
		puntero++;
	}
	else{
		buffer[puntero] = test - 48;
		puntero++;
	}

	if (test == 13){
		sensordist2 = (buffer[1]*1000 + buffer[2]*100 + buffer[3]*10 + buffer[4]); //ASCII  a binario
		//sensordist1 = (buffer1[1]*100 + buffer1[2]*10 + buffer1[3]);
	}
}

// FUNCTION: isrdist3()
// Description: function to read the values from ultrasonic sensor 3 using UART
// Parameters:  - context
//				- id
// Return:      N/A
// Note: 		Sensor communication is RS232, output is an ASCII 'R' followed by three ASCII representing the distance value 
//				in inches and ending with an ASCII 13 (CR or Enter). Baud=9600, 8 bits, no parity with a stop bit.
static void isrdist3 (void * context, alt_u32 id){

	unsigned char buffer[5];
	unsigned char puntero;

	unsigned char test;
	test = IORD_ALTERA_AVALON_UART_RXDATA(DIST3_BASE);

	if (test == 'R'){
		puntero = 0;
		buffer[puntero] = test;
		puntero++;
	}
	else{
		buffer[puntero] = test - 48;
		puntero++;
	}

	if (test == 13){
		sensordist3 = (buffer[1]*1000 + buffer[2]*100 + buffer[3]*10 + buffer[4]); //ASCII  a binario
		//sensordist1 = (buffer1[1]*100 + buffer1[2]*10 + buffer1[3]);
	}
}

// FUNCTION: isrdist4()
// Description: function to read the values from ultrasonic sensor 4 using UART
// Parameters:  - context
//				- id
// Return:      N/A
// Note: 		Sensor communication is RS232, output is an ASCII 'R' followed by three ASCII representing the distance value 
//				in inches and ending with an ASCII 13 (CR or Enter). Baud=9600, 8 bits, no parity with a stop bit.
static void isrdist4 (void * context, alt_u32 id){

	unsigned char buffer[5];
	unsigned char puntero;

	unsigned char test;
	test = IORD_ALTERA_AVALON_UART_RXDATA(DIST4_BASE);

	if (test == 'R'){
		puntero = 0;
		buffer[puntero] = test;
		puntero++;
	}
	else{
		buffer[puntero] = test - 48;
		puntero++;
	}

	if (test == 13){
		sensordist4 = (buffer[1]*1000 + buffer[2]*100 + buffer[3]*10 + buffer[4]); //ASCII  a binario
		//sensordist1 = (buffer1[1]*100 + buffer1[2]*10 + buffer1[3]);
	}
}

// FUNCTION: isrdist5()
// Description: function to read the values from ultrasonic sensor 5 using UART
// Parameters:  - context
//				- id
// Return:      N/A
// Note: 		Sensor communication is RS232, output is an ASCII 'R' followed by three ASCII representing the distance value 
//				in inches and ending with an ASCII 13 (CR or Enter). Baud=9600, 8 bits, no parity with a stop bit.
static void isrdist5 (void * context, alt_u32 id){

	unsigned char buffer[5];
	unsigned char puntero;

	unsigned char test;
	test = IORD_ALTERA_AVALON_UART_RXDATA(DIST5_BASE);

	if (test == 'R'){
		puntero = 0;
		buffer[puntero] = test;
		puntero++;
	}
	else{
		buffer[puntero] = test - 48;
		puntero++;
	}

	if (test == 13){
		sensordist5 = (buffer[1]*1000 + buffer[2]*100 + buffer[3]*10 + buffer[4]); //ASCII  a binario
		//sensordist1 = (buffer1[1]*100 + buffer1[2]*10 + buffer1[3]);
	}
}

// FUNCTION: isrdist6()
// Description: function to read the values from ultrasonic sensor 6 using UART
// Parameters:  - context
//				- id
// Return:      N/A
// Note: 		Sensor communication is RS232, output is an ASCII 'R' followed by three ASCII representing the distance value 
//				in inches and ending with an ASCII 13 (CR or Enter). Baud=9600, 8 bits, no parity with a stop bit.
static void isrdist6 (void * context, alt_u32 id){

	unsigned char buffer[5];
	unsigned char puntero;

	unsigned char test;
	test = IORD_ALTERA_AVALON_UART_RXDATA(DIST6_BASE);

	if (test == 'R'){
		puntero = 0;
		buffer[puntero] = test;
		puntero++;
	}
	else{
		buffer[puntero] = test - 48;
		puntero++;
	}

	if (test == 13){
		sensordist6 = (buffer[1]*1000 + buffer[2]*100 + buffer[3]*10 + buffer[4]); //ASCII  a binario
		//sensordist1 = (buffer1[1]*100 + buffer1[2]*10 + buffer1[3]);
	}
}

// FUNCTION: isrdist7()
// Description: function to read the values from ultrasonic sensor 7 using UART
// Parameters:  - context
//				- id
// Return:      N/A
// Note: 		Sensor communication is RS232, output is an ASCII 'R' followed by three ASCII representing the distance value 
//				in inches and ending with an ASCII 13 (CR or Enter). Baud=9600, 8 bits, no parity with a stop bit.
static void isrdist7 (void * context, alt_u32 id){

	unsigned char buffer[5];
	unsigned char puntero;

	unsigned char test;
	test = IORD_ALTERA_AVALON_UART_RXDATA(DIST7_BASE);

	if (test == 'R'){
		puntero = 0;
		buffer[puntero] = test;
		puntero++;
	}
	else{
		buffer[puntero] = test - 48;
		puntero++;
	}

	if (test == 13){
		sensordist7 = (buffer[1]*1000 + buffer[2]*100 + buffer[3]*10 + buffer[4]); //ASCII  a binario
		//sensordist1 = (buffer1[1]*100 + buffer1[2]*10 + buffer1[3]);
	}
}

// FUNCTION: isrdist8()
// Description: function to read the values from ultrasonic sensor 8 using UART
// Parameters:  - context
//				- id
// Return:      N/A
// Note: 		Sensor communication is RS232, output is an ASCII 'R' followed by three ASCII representing the distance value 
//				in inches and ending with an ASCII 13 (CR or Enter). Baud=9600, 8 bits, no parity with a stop bit.
static void isrdist8 (void * context, alt_u32 id){

	unsigned char buffer[5];
	unsigned char puntero;

	unsigned char test;
	test = IORD_ALTERA_AVALON_UART_RXDATA(DIST8_BASE);

	if (test == 'R'){
		puntero = 0;
		buffer[puntero] = test;
		puntero++;
	}
	else{
		buffer[puntero] = test - 48;
		puntero++;
	}

	if (test == 13){
		sensordist8 = (buffer[1]*1000 + buffer[2]*100 + buffer[3]*10 + buffer[4]); //ASCII  a binario
		//sensordist1 = (buffer1[1]*100 + buffer1[2]*10 + buffer1[3]);
	}
}

// FUNCTION: InstallUltrasonicInterrupts()
// Description: function to define the interrupts from the ultrasonic sensors
void InstallUltrasonicInterrupts() {

	int context_dist1;
	alt_irq_register(DIST1_IRQ,&context_dist1,isrdist1); // install DIST1 ISR
	alt_irq_enable (DIST1_IRQ);

	int context_dist2;
	alt_irq_register(DIST2_IRQ,&context_dist2,isrdist2); // install DIST2 ISR
	alt_irq_enable (DIST2_IRQ);

	int context_dist3;
	alt_irq_register(DIST3_IRQ,&context_dist3,isrdist3); // install DIST3 ISR
	alt_irq_enable (DIST3_IRQ);

	int context_dist4;
	alt_irq_register(DIST4_IRQ,&context_dist4,isrdist4); // install DIST4 ISR
	alt_irq_enable (DIST4_IRQ);

	int context_dist5;
	alt_irq_register(DIST5_IRQ,&context_dist5,isrdist5); // install DIST5 ISR
	alt_irq_enable (DIST5_IRQ);

	int context_dist6;
	alt_irq_register(DIST6_IRQ,&context_dist6,isrdist6); // install DIST6 ISR
	alt_irq_enable (DIST6_IRQ);

	int context_dist7;
	alt_irq_register(DIST7_IRQ,&context_dist7,isrdist7); // install DIST3 ISR
	alt_irq_enable (DIST7_IRQ);

	int context_dist8;
	alt_irq_register(DIST8_IRQ,&context_dist8,isrdist8); // install DIST4 ISR
	alt_irq_enable (DIST8_IRQ);

}
/*--------------------------------------------------------------------------------------*/