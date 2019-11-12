/***********************************************************************
 * $Id:: handlers.c 3871 2010-07-16 11:50:22Z gerritdewaard            $
 *
 * Project: LPC1100 BLDC Motor Control AN  
 *
 * Description:
 *			
 *     
 *
 ***********************************************************************
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
 **********************************************************************/

#include "LPC11xx.h"			/* LPC11xx Peripheral Registers */
#include "application.h"
#include "LSM9DS1.h"

volatile uint32_t SysTick_cntr = 0;
volatile uint32_t delayCounter;

uint32_t cnt_FastEvent = 0, cnt_MedEvent = 0, cnt_SlowEvent = 0, cnt_UART_Timeout = 0;

uint32_t Flag_FastEvent = 0;
uint32_t Flag_MedEvent = 0;
uint32_t Flag_SlowEvent = 0;

//init params
extern uint32_t oFastEvent;				//Fast Event time cycle in ms
extern uint32_t oMedEvent;				//Med Event time cycle in ms
extern uint32_t oSlowEvent;				//Slow Event time cycle in ms
extern uint32_t oUARTtimeout;			//UART Timeout in ms

extern uint8_t SensorlessState;
extern uint32_t RPM_Const, comm_Time, Min_Comm_Time, pwmPeriod;
uint32_t filtered_ZC_Time = 0;	// comm_Time/8;;

extern uint32_t mRPM, mFilteredRPM;
extern int32_t BEMF_pin_mask;
extern uint8_t mCMTStep;

extern int8_t mDirection,  B_ADCc;
extern int32_t stl_tmp;
extern int16_t mSP, tmp_mSP;

extern int32_t pidIntError;			//PID integral error, clear on init
extern int32_t pidLastError;			//PID last error, clear on init

extern uint32_t mMinPeriod;		//Motor minimum allowed PWM match register value
extern uint32_t pwmMaxPeriod;	//PWM max allowed pulse length register value

volatile uint32_t regVal, zero_crossing_time, corrected_zero_crossing_time,
		ZC_stall_count = 0, ZC_count = 0;
volatile int32_t ZC_error, pidOutput;

extern volatile uint8_t RxFrameReceivedFlag;

extern volatile uint8_t RxState;

extern config_t config;

extern uint8_t  i2c_enabled;

extern volatile uint8_t I2CSlaveBuffer[BUFSIZE];

extern IMU_REPORT IMU;

uint8_t led_flag=0, led_value=0;

/*****************************************************************************
 ** Function name:		PIOINT0_IRQHandler
 **
 ** Descriptions:		Use one GPIO pin(port0 pin1) as interrupt source
 **
 ** parameters:			None
 ** Returned value:		None
 **
 *****************************************************************************/
#ifndef GPIO_GENERIC_INTS

void PIOINT2_IRQHandler(void) {

	regVal = LPC_GPIO2->MIS;
	/* Clear all interrupt sources */
	LPC_GPIO2->IC = 0xFFF;

	if (i2c_enabled) {
		if (regVal & 0x10000000000)
		readMagAll();
		IMU.mx = (I2CSlaveBuffer[1] << 8) | (I2CSlaveBuffer[0]);
		IMU.my = (I2CSlaveBuffer[3] << 8) | (I2CSlaveBuffer[2]);
		IMU.mz = (I2CSlaveBuffer[5] << 8) | (I2CSlaveBuffer[4]);
	}

	/*

	if (i2c_enabled) {
		readGyroAll();
		IMU.gx = (I2CSlaveBuffer[1] << 8) | (I2CSlaveBuffer[0]);
		IMU.gy = (I2CSlaveBuffer[3] << 8) | (I2CSlaveBuffer[2]);
		IMU.gz = (I2CSlaveBuffer[5] << 8) | (I2CSlaveBuffer[4]);

		readAccelAll();
		IMU.ax = (I2CSlaveBuffer[1] << 8) | (I2CSlaveBuffer[0]);
		IMU.ay = (I2CSlaveBuffer[3] << 8) | (I2CSlaveBuffer[2]);
		IMU.az = (I2CSlaveBuffer[5] << 8) | (I2CSlaveBuffer[4]);

		readMagAll();
		IMU.mx = (I2CSlaveBuffer[1] << 8) | (I2CSlaveBuffer[0]);
		IMU.my = (I2CSlaveBuffer[3] << 8) | (I2CSlaveBuffer[2]);
		IMU.mz = (I2CSlaveBuffer[5] << 8) | (I2CSlaveBuffer[4]);

		readTemp();
	}


	if (i2c_enabled) {
		readGyroAll();
		IMU.gx = (I2CSlaveBuffer[1] << 8) | (I2CSlaveBuffer[0]);
		IMU.gy = (I2CSlaveBuffer[3] << 8) | (I2CSlaveBuffer[2]);
		IMU.gz = (I2CSlaveBuffer[5] << 8) | (I2CSlaveBuffer[4]);

		readAccelAll();
		IMU.ax = (I2CSlaveBuffer[1] << 8) | (I2CSlaveBuffer[0]);
		IMU.ay = (I2CSlaveBuffer[3] << 8) | (I2CSlaveBuffer[2]);
		IMU.az = (I2CSlaveBuffer[5] << 8) | (I2CSlaveBuffer[4]);

		readMagAll();
		IMU.mx = (I2CSlaveBuffer[1] << 8) | (I2CSlaveBuffer[0]);
		IMU.my = (I2CSlaveBuffer[3] << 8) | (I2CSlaveBuffer[2]);
		IMU.mz = (I2CSlaveBuffer[5] << 8) | (I2CSlaveBuffer[4]);

		readTemp();
		IMU.temperature = (I2CSlaveBuffer[1] << 8) | (I2CSlaveBuffer[0]);
	}


*/




	return;
}
#endif

/*****************************************************************************
 ** Function name:		PIOINT1_IRQHandler
 **
 ** Descriptions:		Use GPIO pins as interrupt source - Sensorless operation
 **
 ** parameters:			None
 ** Returned value:		None
 **
 *****************************************************************************/
#ifndef GPIO_GENERIC_INTS

void PIOINT1_IRQHandler(void) {

	regVal = LPC_GPIO1->MIS;


	if (regVal & BEMF_pin_mask) {
	GPIOSetValue(DEBUG_PORT, DEBUG_BIT, 1); // DEBUG !!!!!!!!!!!!!!!!!!!!!!

		//Disable BEMF pins interrupt
		LPC_GPIO1->IE = 0;

		//Clear all the interrupts
		LPC_GPIO1->IC = 0xFFFFFFFF;

		zero_crossing_time = LPC_TMR16B1->TC;
		corrected_zero_crossing_time = zero_crossing_time - config.zc_correction;

		//CALCULATE NEW COMMUTATION TIME !!!!!!!!!!!!!!!!!!!!
		filtered_ZC_Time = (config.comm_IIR_A * corrected_zero_crossing_time
				+ config.comm_IIR_B * filtered_ZC_Time)
				/ (config.comm_IIR_A + config.comm_IIR_B);


		//Load the commutation timer
		LPC_TMR16B1->MR1 = comm_Time / 2 + zero_crossing_time;

		if ((zero_crossing_time >= comm_Time / 2)){; // && (comm_Time > START_COMM_TIME/10)) {
			//increment the run timer
			//in case we have the PLL locked
			ZC_count++;

		}
		GPIOSetValue(DEBUG_PORT, DEBUG_BIT, 0); // DEBUG !!!!!!!!!!!!!!!!!!!!!!
	}
}
#endif //GPIO_GENERIC_INTS

/*****************************************************************************
 ** Function name:		SysTick_Handler
 **
 ** Descriptions:		scheduler
 **
 ** parameters:			None
 ** Returned value:		None
 **
 *****************************************************************************/
void SysTick_Handler(void) {

	if (!cnt_FastEvent) {
		Flag_FastEvent = 1;
		cnt_FastEvent = oFastEvent;
	}
	if (!cnt_MedEvent) {
		Flag_MedEvent = 1;
		cnt_MedEvent = oMedEvent;
	}
	if (!cnt_SlowEvent) {
		Flag_SlowEvent = 1;
		cnt_SlowEvent = oSlowEvent;
		led_flag++;
		if(led_flag >3)
		{
			GPIOSetValue(LED_PORT, LED_BIT, led_value);
			led_flag=0;
			led_value=!led_value;
		}
	}

	if (!cnt_UART_Timeout) {
		RxState = 0;
		cnt_UART_Timeout = oUARTtimeout;
	}


	SysTick_cntr++;
	cnt_FastEvent--;
	cnt_MedEvent--;
	cnt_SlowEvent--;
	delayCounter--;
	cnt_UART_Timeout--;

}

/*****************************************************************************
 ** Function name:		Delay_ms(int16_t delay_ms)
 **
 ** Descriptions:		delays "delay_ms" number of milliseconds
 **
 ** parameters:			None
 ** Returned value:		None
 **
 *****************************************************************************/
void Delay_ms(uint32_t delay_ms) {

	delayCounter = delay_ms;

	//GPIOSetValue(DEBUG_PORT, DEBUG_BIT, 1); // DEBUG !!!!!!!!!!!!!!!!!!!!!!
	while (delayCounter) {

		if (RxFrameReceivedFlag) { 		//if valid frame received -

			RxFrameParser();			// parse and execute
			RxFrameReceivedFlag = 0; 	// clear flag and wait for another frame
		}

	}
	//GPIOSetValue(DEBUG_PORT, DEBUG_BIT, 0); // DEBUG !!!!!!!!!!!!!!!!!!!!!!
}

/******************************************************************************
 * Blink - blinks number of times with a set interval
 *****************************************************************************/
void Blink(uint8_t count, uint32_t interval) { // blink the LED 3 times
	uint8_t i;
	for (i = 1; i <= count; i++) {
		GPIOSetValue(LED_PORT, LED_BIT, 1);

		SysTick_cntr = 0;
		while (SysTick_cntr < interval)
			;
		SysTick_cntr = 0;

		GPIOSetValue(LED_PORT, LED_BIT, 0);

		SysTick_cntr = 0;
		while (SysTick_cntr < interval)
			;
		SysTick_cntr = 0;
	}
}
