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

#include "io.h"										// for reading or writing to I/O
#include "system.h"									// for the configuration and register of the QSYS system
#include "sys/alt_irq.h"
#include "altera_avalon_uart.h"
#include "altera_avalon_uart_regs.h"

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

/* Ultrasonic sensor variables */
unsigned char buffer1[5]={0}, buffer2[5]={0}, buffer3[5]={0}, buffer4[5]={0}, buffer5[5]={0}, buffer6[5]={0}, buffer7[5]={0}, buffer8[5]={0};
unsigned char puntero1 = 0, puntero2 = 0, puntero3 = 0, puntero4 = 0, puntero5 = 0, puntero6 = 0, puntero7 = 0, puntero8 = 0;
unsigned int sensordist1=0, sensordist2=0, sensordist3=0, sensordist4 = 0, sensordist5=0, sensordist6=0, sensordist7=0, sensordist8 = 0;

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
void InstallUltrasonicInterrupts();

////////////////////////////
///                      ///
///   PUBLIC FUNCTIONS   ///
///                      ///
////////////////////////////


// FUNCTION: TaskSensing()
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



////////////////////////////
///                      ///
///   PRIVATE FUNCTIONS  ///
///                      ///
////////////////////////////

/*
 * Interrupt Functions
 */

/*************************************************************************
**************************************************************************
**																		**
** Function:    isrdist1()												**
**																		**
** Description: Funci�n que lee los valores de distancia del sensor 	**
** 				ultrasonico 1 usando UART								**
** 																		**
** Notes:       La comunicaci�n del sensor es RS232, la salida es un 	**
** 				ASCII 'R', seguida de tres ASCII que representan el		**
** 				valor de la distancia en pulgadas y termina con un 		**
** 				ASCII 13 (CR o Enter). Baud=9600, 8 bits, sin paridad	**
** 				con un bit de parada									**
**																		**
** Returns:     N/A														**
**																		**
**************************************************************************
*************************************************************************/
static void isrdist1 (void * context, alt_u32 id){

	unsigned char test;
	test = IORD_ALTERA_AVALON_UART_RXDATA(DIST1_BASE);

	if (test == 'R'){
		puntero1 = 0;
		buffer1[puntero1] = test;
		puntero1++;
	}
	else{
		buffer1[puntero1] = test - 48;
		puntero1++;
	}

	if (test == 13){
		sensordist1 = (buffer1[1]*1000 + buffer1[2]*100 + buffer1[3]*10 + buffer1[4]); //ASCII  a binario
		//sensordist1 = (buffer1[1]*100 + buffer1[2]*10 + buffer1[3]);
	}
}

/*************************************************************************
**************************************************************************
**																		**
** Function:    isrdist2()												**
**																		**
** Description: Funci�n que lee los valores de distancia del sensor 	**
** 				ultrasonico 2 usando UART								**
** 																		**
** Notes:       La comunicaci�n del sensor es RS232, la salida es un 	**
** 				ASCII 'R', seguida de tres ASCII que representan el		**
** 				valor de la distancia en pulgadas y termina con un 		**
** 				ASCII 13 (CR o Enter). Baud=9600, 8 bits, sin paridad	**
** 				con un bit de parada									**
**																		**
** Returns:     N/A														**
**																		**
**************************************************************************
*************************************************************************/
static void isrdist2 (void * context, alt_u32 id){

	unsigned char test;
	test = IORD_ALTERA_AVALON_UART_RXDATA(DIST2_BASE);

	if (test == 'R'){
		puntero2 = 0;
		buffer2[puntero2] = test;
		puntero2++;
	}
	else{
		buffer2[puntero2] = test - 48;
		puntero2++;
	}

	if (test == 13){
		sensordist2 = (buffer2[1]*1000 + buffer2[2]*100 + buffer2[3]*10 + buffer2[4]);
	}
}

/*************************************************************************
**************************************************************************
**																		**
** Function:    isrdist3()												**
**																		**
** Description: Funci�n que lee los valores de distancia del sensor 	**
** 				ultrasonico 3 usando UART								**
** 																		**
** Notes:       La comunicaci�n del sensor es RS232, la salida es un 	**
** 				ASCII 'R', seguida de tres ASCII que representan el		**
** 				valor de la distancia en pulgadas y termina con un 		**
** 				ASCII 13 (CR o Enter). Baud=9600, 8 bits, sin paridad	**
** 				con un bit de parada									**
**																		**
** Returns:     N/A														**
**																		**
**************************************************************************
*************************************************************************/
static void isrdist3 (void * context, alt_u32 id){

	unsigned char test;
	test = IORD_ALTERA_AVALON_UART_RXDATA(DIST3_BASE);

	if (test == 'R'){
		puntero3 = 0;
		buffer3[puntero3] = test;
		puntero3++;
	}
	else{
		buffer3[puntero3] = test - 48;
		puntero3++;
	}

	if (test == 13){
		sensordist3 = (buffer3[1]*1000 + buffer3[2]*100 + buffer3[3]*10 + buffer3[4]);
	}
}

/*************************************************************************
**************************************************************************
**																		**
** Function:    isrdist4()												**
**																		**
** Description: Funci�n que lee los valores de distancia del sensor 	**
** 				ultrasonico 4 usando UART								**
** 																		**
** Notes:       La comunicaci�n del sensor es RS232, la salida es un 	**
** 				ASCII 'R', seguida de tres ASCII que representan el		**
** 				valor de la distancia en pulgadas y termina con un 		**
** 				ASCII 13 (CR o Enter). Baud=9600, 8 bits, sin paridad	**
** 				con un bit de parada									**
**																		**
** Returns:     N/A														**
**																		**
**************************************************************************
*************************************************************************/
static void isrdist4 (void * context, alt_u32 id){

	unsigned char test;
	test = IORD_ALTERA_AVALON_UART_RXDATA(DIST4_BASE);

	if (test == 'R'){
		puntero4 = 0;
		buffer4[puntero4] = test;
		puntero4++;
	}
	else{
		buffer4[puntero4] = test - 48;
		puntero4++;
	}

	if (test == 13){
		sensordist4 = (buffer4[1]*1000 + buffer4[2]*100 + buffer4[3]*10 + buffer4[4]);
	}
}

/*************************************************************************
**************************************************************************
**																		**
** Function:    isrdist5()												**
**																		**
** Description: Funci�n que lee los valores de distancia del sensor 	**
** 				ultrasonico 5 usando UART								**
** 																		**
** Notes:       La comunicaci�n del sensor es RS232, la salida es un 	**
** 				ASCII 'R', seguida de tres ASCII que representan el		**
** 				valor de la distancia en pulgadas y termina con un 		**
** 				ASCII 13 (CR o Enter). Baud=9600, 8 bits, sin paridad	**
** 				con un bit de parada									**
**																		**
** Returns:     N/A														**
**																		**
**************************************************************************
*************************************************************************/
static void isrdist5 (void * context, alt_u32 id){

	unsigned char test;
	test = IORD_ALTERA_AVALON_UART_RXDATA(DIST5_BASE);

	if (test == 'R'){
		puntero5 = 0;
		buffer5[puntero5] = test;
		puntero5++;
	}
	else{
		buffer5[puntero5] = test - 48;
		puntero5++;
	}

	if (test == 13){
		sensordist5 = (buffer5[1]*1000 + buffer5[2]*100 + buffer5[3]*10 + buffer5[4]);
	}
}

/*************************************************************************
**************************************************************************
**																		**
** Function:    isrdist6()												**
**																		**
** Description: Funci�n que lee los valores de distancia del sensor 	**
** 				ultrasonico 6 usando UART								**
** 																		**
** Notes:       La comunicaci�n del sensor es RS232, la salida es un 	**
** 				ASCII 'R', seguida de tres ASCII que representan el		**
** 				valor de la distancia en pulgadas y termina con un 		**
** 				ASCII 13 (CR o Enter). Baud=9600, 8 bits, sin paridad	**
** 				con un bit de parada									**
**																		**
** Returns:     N/A														**
**																		**
**************************************************************************
*************************************************************************/
static void isrdist6 (void * context, alt_u32 id){

	unsigned char test;
	test = IORD_ALTERA_AVALON_UART_RXDATA(DIST6_BASE);

	if (test == 'R'){
		puntero6 = 0;
		buffer6[puntero6] = test;
		puntero6++;
	}
	else{
		buffer6[puntero6] = test - 48;
		puntero6++;
	}

	if (test == 13){
		sensordist6 = (buffer6[1]*1000 + buffer6[2]*100 + buffer6[3]*10 + buffer6[4]);
	}
}

/*************************************************************************
**************************************************************************
**																		**
** Function:    isrdist7()												**
**																		**
** Description: Funci�n que lee los valores de distancia del sensor 	**
** 				ultrasonico 7 usando UART								**
** 																		**
** Notes:       La comunicaci�n del sensor es RS232, la salida es un 	**
** 				ASCII 'R', seguida de tres ASCII que representan el		**
** 				valor de la distancia en pulgadas y termina con un 		**
** 				ASCII 13 (CR o Enter). Baud=9600, 8 bits, sin paridad	**
** 				con un bit de parada									**
**																		**
** Returns:     N/A														**
**																		**
**************************************************************************
*************************************************************************/
static void isrdist7 (void * context, alt_u32 id){

	unsigned char test;
	test = IORD_ALTERA_AVALON_UART_RXDATA(DIST7_BASE);

	if (test == 'R'){
		puntero7 = 0;
		buffer7[puntero7] = test;
		puntero7++;
	}
	else{
		buffer7[puntero7] = test - 48;
		puntero7++;
	}

	if (test == 13){
		sensordist7 = (buffer7[1]*1000 + buffer7[2]*100 + buffer7[3]*10 + buffer7[4]);
	}
}

/*************************************************************************
**************************************************************************
**																		**
** Function:    isrdist8()												**
**																		**
** Description: Funci�n que lee los valores de distancia del sensor 	**
** 				ultrasonico 8 usando UART								**
** 																		**
** Notes:       La comunicaci�n del sensor es RS232, la salida es un 	**
** 				ASCII 'R', seguida de tres ASCII que representan el		**
** 				valor de la distancia en pulgadas y termina con un 		**
** 				ASCII 13 (CR o Enter). Baud=9600, 8 bits, sin paridad	**
** 				con un bit de parada									**
**																		**
** Returns:     N/A														**
**																		**
**************************************************************************
*************************************************************************/
static void isrdist8 (void * context, alt_u32 id){

	unsigned char test;
	test = IORD_ALTERA_AVALON_UART_RXDATA(DIST8_BASE);

	if (test == 'R'){
		puntero8 = 0;
		buffer8[puntero8] = test;
		puntero8++;
	}
	else{
		buffer8[puntero8] = test - 48;
		puntero8++;
	}

	if (test == 13){
		sensordist8 = (buffer8[1]*1000 + buffer8[2]*100 + buffer8[3]*10 + buffer8[4]);
	}
}

/*************************************************************************
**************************************************************************
**																		**
** Function:    interrupciones()										**
**																		**
** Description: Funci�n donde se definen las interrupciones 			**
** 																		**
** Notes:       En esta funci�n se definen interrupciones asociadas con **
** 				los encoders, la tarjeta XBee, el GPS y los sensores de **
** 				distancia												**
**																		**
** Returns:     N/A														**
**																		**
**************************************************************************
*************************************************************************/
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



