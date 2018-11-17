
/**********************************************************************
File    : uart.h
Purpose : 
**********************************************************************/
#ifndef __UART_H__
#define __UART_H__
/****************************** Includes *****************************/
/****************************** Defines *******************************/
/***************************** Prototypes *****************************/
void USART_Config(void);
void USART_Config6(void);

int fputc ( int ch );      // Primary UART for QUAT/ACCEL/GYRO data
int fputc6( int ch );      // Secondary UART for output strings and enter COMMANDs

#endif // __UART_H__


