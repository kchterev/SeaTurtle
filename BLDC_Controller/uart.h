/*****************************************************************************
 *   uart.h:  Header file for NXP LPC1xxx Family Microprocessors
 *
 *   Copyright(C) 2008, NXP Semiconductor
 *   All rights reserved.
 *
 *   History
 *   2009.12.07  ver 1.00    Preliminary version, first Release
 *
******************************************************************************/
#ifndef __UART_H 
#define __UART_H

#define USE_UART			1
#define RS485_ENABLED		0
#define MODEM_TEST			0

#define IER_RBR         (0x01<<0)
#define IER_THRE        (0x01<<1)
#define IER_RLS         (0x01<<2)
#define IER_ABEO        (0x01<<8)
#define IER_ABTO        (0x01<<9)

#define IIR_PEND        0x01
#define IIR_RLS         0x03
#define IIR_RDA         0x02
#define IIR_CTI         0x06
#define IIR_THRE        0x01
#define IIR_ABEO        (0x01<<8)
#define IIR_ABTO        (0x01<<9)

#define LSR_RDR		0x01
#define LSR_OE		0x02
#define LSR_PE		0x04
#define LSR_FE		0x08
#define LSR_BI		0x10
#define LSR_THRE	0x20
#define LSR_TEMT	0x40
#define LSR_RXFE	0x80

#define UART_RxBUFSIZE 0x101		//Must be greater than 0xFF !!!!!
#define UART_TxBUFSIZE 0x101		// CAN BE SMALLER!!!!!!!

void ModemInit( void );
void UARTInit(uint32_t Baudrate);
void UART_IRQHandler(void);
void UARTSend(uint8_t *BufferPtr, uint32_t Length);
void UARTIRQSend (void);

#endif /* end __UART_H */
/*****************************************************************************
**                            End Of File
******************************************************************************/
