/****************************************************************************
 *   $Id:: uart.c 9372 2012-04-19 22:56:24Z nxp41306                        $
 *   Project: NXP LPC11xx UART example
 *
 *   Description:
 *     This file contains UART code example which include UART 
 *     initialization, UART interrupt handler, and related APIs for 
 *     UART access.
 *
 ****************************************************************************
 * Software that is described herein is for illustrative purposes only
 * which provides customers with programming information regarding the
 * products. This software is supplied "AS IS" without any warranties.
 * NXP Semiconductors assumes no responsibility or liability for the
 * use of the software, conveys no license or title under any patent,
 * copyright, or mask work right to the product. NXP Semiconductors
 * reserves the right to make changes in the software without
 * notification. NXP Semiconductors also make no representation or
 * warranty that such application will be suitable for the specified
 * use without further testing or modification.

 * Permission to use, copy, modify, and distribute this software and its
 * documentation is hereby granted, under NXP Semiconductors'
 * relevant copyright in the software, without fee, provided that it
 * is used in conjunction with NXP Semiconductors microcontrollers.  This
 * copyright, permission, and disclaimer notice must appear in all copies of
 * this code.
 ****************************************************************************/
#include "LPC11xx.h"			/* LPC11xx Peripheral Registers */
#include "application.h"

volatile uint8_t UARTTxEmpty = 1;// Flag used for non interrupt driven transmission

volatile uint8_t TXBUF[UART_TxBUFSIZE];		// UART transmit buffer
volatile uint8_t RXBUF[UART_RxBUFSIZE];		// UART receive buffer

volatile uint8_t TXDataLen, RXDataLen, TXDataCount, RXDataCount;
volatile uint8_t RxState = 0xFF, RxBoF, RxCHKSUM, RxFrameReceivedFlag = 0;

volatile uint8_t BreakFlag = 0;

extern uint32_t cnt_UART_Timeout;

extern uint32_t Flag_MedEvent;

//init params
extern uint32_t oFastEvent;				//Fast Event time cycle in ms
extern uint32_t oMedEvent;				//Med Event time cycle in ms
extern uint32_t oSlowEvent;				//Slow Event time cycle in ms
extern uint32_t oUARTtimeout;			//UART Timeout in ms

/*****************************************************************************
 ** Function name:		UART_IRQHandler
 **
 ** Descriptions:		UART interrupt handler
 **
 ** parameters:			None
 ** Returned value:		None
 **
 *****************************************************************************/
void UART_IRQHandler(void) {
	uint8_t IIRValue, LSRValue, tmp;

	IIRValue = LPC_UART->IIR;

	IIRValue >>= 1; /* skip pending bit in IIR */
	IIRValue &= 0x07; /* check bit 1~3, interrupt identification */
	if (IIRValue == IIR_RLS) /* Receive Line Status */
	{
		LSRValue = LPC_UART->LSR;
		/* Receive Line Status */
		if (LSRValue & (LSR_OE | LSR_PE | LSR_FE | LSR_RXFE | LSR_BI)) {
			/* There are errors or break interrupt */
			/* Read LSR will clear the interrupt */
			///////////////////////////////////////////////////////////////////////////////////
			if (LSRValue & LSR_BI) {
				RxBoF = LPC_UART->RBR; /* Dummy read*/
				BreakFlag++;// Use break flag to initiate reset of the controller
				RxState = 0;// Initialise the state machine, begin of frame transmission
				return;
			}

			return;
		}

	} else if (IIRValue == IIR_RDA) /* RxData in the receive buffer */
	{
		cnt_UART_Timeout = oUARTtimeout;

		BreakFlag = 0;// Clear the brake flag, so next brakes do not reset the system
		tmp = LPC_UART->RBR;		// Read the buffer
		RxCHKSUM = RxCHKSUM ^ tmp;	// update the checksum

		if (RxState == 0) {				// State BoF expected
			/*Start of frame detected */
			RxState = 1;		// Set state machine to 1 - expect length byte
			RxBoF = tmp;				// Read the Begin of Frame byte
			RXDataCount = 0;			// Clear the data counter
			RxFrameReceivedFlag = 0;	// Clear frame received flag
			RxCHKSUM = RxBoF;			// Initialise the check sum
			return;
		}

		if (RxState == 1) {				//State data length byte expected
			RXBUF[RXDataCount++] = tmp;
			RXDataLen = tmp;

			if (RXDataLen == 0xFF){
				RxState =3;
			}
			else if (RXDataLen)
				RxState = 2;     	// Set state to 2 - expect data bytes
			else
				RxState = 3; 		// Zero length - receive the check sum
			return;
		}

		if (RxState == 2) {					// RXDataLen data bytes expected
			if (RXDataCount == RXDataLen)  	//End of frame reached
				RxState = 3;
			RXBUF[RXDataCount++] = tmp;	// Store the next frame byte and increment the counter
			return;
		}

		if (RxState == 3) {				// Check sum expected
			RxState = 0xFF; // reset the receive state machine to invalid state
			if (!RxCHKSUM)		// CHKSUM must be = 0 if correct transmission
			{
				RxFrameReceivedFlag = 1;	// Set frame received flag if CHKSUM
				return;
			} else {
				//DEBU&G!!!!!!!!!!!!!!!!!!!!!
				RxFrameReceivedFlag = 0;	// Set frame received flag if CHKSUM
				return;
			}

		}

	} else if (IIRValue == IIR_THRE) /* THRE, transmit holding register empty */
	{
		/* THRE interrupt */
		LSRValue = LPC_UART->LSR; /* Check status in the LSR to see if
		 valid data in U0THR or not */
		if (LSRValue & LSR_THRE) {
			UARTTxEmpty = 1;			//Transmit FIFO empty
			UARTIRQSend(); 				//put a char in the transmit FIFO
		} else {
			UARTTxEmpty = 0;			//Transmit FIFO full, wait...
		}
	}
	return;
}

/*****************************************************************************
 ** Function name:		UARTInit
 **
 ** Descriptions:		Initialize UART0 port, setup pin select,
 **				clock, parity,  bits, FIFO, etc.
 **
 ** parameters:			UART baudrate
 ** Returned value:		None
 **
 *****************************************************************************/
void UARTInit(uint32_t baudrate) {

	uint32_t Fdiv;
	BreakFlag = 0;
	uint32_t regVal = regVal;

	UARTTxEmpty = 1;
	RXDataCount = 0;

	NVIC_DisableIRQ(UART_IRQn);

	LPC_IOCON->PIO1_6 &= ~0x07; /*  UART I/O config */
	LPC_IOCON->PIO1_6 |= 0x01; /* UART RXD */
	LPC_IOCON->PIO1_7 &= ~0x07;
	LPC_IOCON->PIO1_7 |= 0x01; /* UART TXD */

	/* Enable UART clock */
	LPC_SYSCON->SYSAHBCLKCTRL |= (1 << 12);
	LPC_SYSCON->UARTCLKDIV = 0x1; /* divided by 1 */

	LPC_UART->LCR = 0x83; /* 8 bits, no Parity, 1  bit */

	Fdiv = ((SystemCoreClock / LPC_SYSCON->UARTCLKDIV) / 16) / baudrate; /*baud rate */
	LPC_UART->DLM = Fdiv / 256;
	LPC_UART->DLL = Fdiv % 256;
	LPC_UART->FDR = 0x10; /* Default */

	LPC_UART->LCR = 0x03; /* DLAB = 0 */
	LPC_UART->FCR = 0x07; /* Enable and reset TX and RX FIFO. */

	/* Read to clear the line status. */
	regVal = LPC_UART->LSR;

	/* Ensure a clean start, no data in either TX or RX FIFO. */
	while (( LPC_UART->LSR & (LSR_THRE | LSR_TEMT)) != (LSR_THRE | LSR_TEMT))
		;
	while ( LPC_UART->LSR & LSR_RDR) {
		regVal = LPC_UART->RBR; /* Dump data from RX FIFO */
	}

	/* Enable the UART Interrupt */
	NVIC_EnableIRQ(UART_IRQn);
	LPC_UART->IER = IER_RBR | IER_THRE | IER_RLS; /* Enable UART interrupt */

	return;
}

/*****************************************************************************
 ** Function name:		UARTSend
 **
 ** Descriptions:		Send a block of data to the UART 0 port based
 **				on the data length
 **
 ** parameters:		buffer pointer, and data length
 ** Returned value:	None
 **
 *****************************************************************************/
void UARTSend(uint8_t *BufferPtr, uint32_t Length) {

	while (Length != 0) {
		/* THRE status, contain valid data */
		/* Below flag is set inside the interrupt handler when THRE occurs. */
		while (!(UARTTxEmpty & 0x01))
			;
		LPC_UART->THR = *BufferPtr;
		UARTTxEmpty = 0; /* not empty in the THR until it shifts out */
		BufferPtr++;
		Length--;
	}
	return;
}

/******************************************************************************
 **                            End Of File
 ******************************************************************************/

/*****************************************************************************
 ** Function name:		UARTIRQSend
 **
 ** Descriptions:		Send a block of data to the UART 0 port based
 **				        on the data length
 **
 ** parameters:		buffer pointer, and data length
 ** Returned value:	None
 **
 *****************************************************************************/
void UARTIRQSend(void)

{// While the transmit FIFO is empty and data to send - write the current byte in the transmit FIFO
// while (( LPC_UART->LSR & LSR_THRE) && TXDataCount) LPC_UART->THR = TXBUF[(TXDataLen - TXDataCount--)];
	if (( LPC_UART->LSR & LSR_THRE) && TXDataCount)
		LPC_UART->THR = TXBUF[(TXDataLen - TXDataCount--)];
}

/******************************************************************************
 **                            End Of File
 ******************************************************************************/

