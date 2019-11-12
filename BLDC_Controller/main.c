/***********************************************************************
 * $Id:: BLDC.c 3871 2010-07-16 11:50:22Z gerritdewaard                $
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

/******************************************************************************
 * Includes
 *****************************************************************************/
#include "LPC11xx.h"			/* LPC11xx Peripheral Registers */
#include "application.h"

/******************************************************************************
 * External global variables
 *****************************************************************************/
extern volatile uint32_t SysTick_cntr;
extern uint8_t mEnable;
extern uint32_t mRPM, mFilteredRPM;
extern int16_t mSP, tmp_mSP;
//extern int16_t mSP_OpenLoop;
extern uint8_t mDirection;
extern uint8_t mRealDirection;
extern int32_t pidOutput;
extern uint32_t oBaudRate;
extern int32_t mCurrent, mFilteredCurrent, stl_tmp;
extern int32_t M_MaxRPM;
extern uint32_t B_ADCRefv;				//ADC reference voltage in millivolts
extern uint8_t B_ADCc;				//ADC used to monitor the motor current
extern uint8_t B_ADCt;				//ADC used to monitor the board temperature
extern uint32_t B_ADCr;				//ADC nubmer of discrettes
extern uint16_t B_ADCs;				//Shunt resistance in milliohms
extern uint16_t B_ADCa;				//ADC ampliffication
extern uint16_t B_ADCo; 			//ADC voltage offset

extern volatile uint8_t RxBoF, RxFrameReceivedFlag;
extern volatile uint8_t BreakFlag;
extern uint32_t pwmPeriod, oFastEvent, oMedEvent, oSlowEvent;
extern uint32_t pwmLoadVal;			//PWM match registers load value
extern uint32_t pwmMaxPeriod;	//PWM max allowed pulse length register value
extern uint32_t pwmMinPeriod;	//PWM min allowed pulse length register value

extern uint32_t RPM_Const, comm_Time, ZC_count, free_run_counter;
extern uint32_t filtered_ZC_Time, zero_crossing_time, pwmPIDVal;
extern uint32_t cnt_FastEvent, cnt_MedEvent, cnt_SlowEvent;

uint8_t SensorlessState = STOP;

/******************************************************************************
 * Local variables
 *****************************************************************************/

extern volatile uint8_t TXBUF[UART_TxBUFSIZE];
extern volatile uint8_t RXBUF[UART_RxBUFSIZE];

extern volatile uint8_t TXDataLen, RXDataLen, TXDataCount, RXDataCount;

extern uint32_t cnt_FastEvent, cnt_MedEvent, cnt_SlowEvent;

extern uint32_t Flag_FastEvent;
extern uint32_t Flag_MedEvent;
extern uint32_t Flag_SlowEvent;

extern uint32_t mHALstate;

extern int32_t loadval;
extern int32_t Kp;
extern int32_t Ki;
extern int32_t Kd;

extern int8_t mCMTStep;		//Motor commutation step

extern config_t config;
extern char* ptr_flash_config;
extern char* ptr_ram_config;

uint32_t k = 0;

uint8_t OverCurrentFlag = 0; // 0 - current below limit. 1- overload!!!
uint8_t OverTempFlag = 0; // 0 - temperature OK, 1- overheat!!!

int16_t bTemperature;
uint32_t bThermPullup = 10000; // the value of the resistor from thermistor to Vref

/*uint32_t THERM[40] = { 182928, 139839, 107902, 83986, 65904, 52111, 41501,
 33276, 26851, 21799, 17798, 14612, 12058, 10000, 8332, 6974, 5863, 4949,
 4194, 3568, 3047, 2611, 2245, 1937, 1676, 1455, 1267, 1106, 968, 850,
 748, 660, 584, 518, 460, 410, 366, 327, 293, 0 };
 */

uint32_t THERM[40] = { 347116.4, 250089.1, 182023.2, 133803.7, 99312.8, 74407.5,
		56257.1, 42910.2, 33009.3, 25602.2, 20015.1, 15767.1, 12512.3, 10000.0,
		8046.8, 6517.6, 5312.5, 4356.6, 3593.6, 2981.0, 2486.2, 2084.3, 1756.2,
		1486.9, 1264.7, 1080.6, 927.2, 798.9, 691.2, 600.2, 523.2, 457.7, 401.8,
		353.8, 312.65, 277.10, 246.34, 219.62, 196.35, 0 };

int32_t tFast;			//Fast Event time cycle = 2 ms
int32_t tMed;			//Med Event time cycle  = 200 ms
int32_t tSlow;			//Slow Event time cycle = 1s

int32_t ADC_current, ADC_temp;

/******************************************************************************
 * RxFrameParser - parses the incoming frame
 *****************************************************************************/
void RxFrameParser(void) {

	uint16_t checksum, i;
	int res;

	//CONTROL FRAME. Header MSb is cleared. Execute the SP and return status when
	//Header byte equals board ID
	//5 bytes returned - uint RPM, uint Current and char Temperature
	//If Length equals 0xFF - reset the controller. Applies to all controllers
	//If length equals zero - do not execute anything, just return the status
	if (!(RxBoF & 0x80)) {		//Usual control header - MSb = 0
								//execute the SP and return status
		if (RXDataLen) { // If frame length = 0  - do nothing, just return the status
			//Otherwise parse here the control sequence

			if (RXDataLen == 0xFF)				// 0xFF length is a reset signal
				NVIC_SystemReset();					// reset the controller

			tmp_mSP = (RXBUF[(config.board_id << 1) - 1] << 8)
					| RXBUF[config.board_id << 1]; // get the set point RPM

			////Cap the setpoint if >  M_MaxRPM
			if (tmp_mSP > M_MaxRPM)
				tmp_mSP = M_MaxRPM;

			if (tmp_mSP < -M_MaxRPM)
				tmp_mSP = -M_MaxRPM;

			//if setpoint less the minimum RPM - set it to 0
			if (abs(tmp_mSP) < config.min_rpm) {
				tmp_mSP = 0;
			}

		}

		TxFrameBuilder(); 			// transmit a status response

		//CONFIG FRAME. MSb = 1. Accept control message only if
		//the motor is in STOP_CLEAR_SP state and the Header byte matches the Board ID
		//If length equals Zero, return the configuration data to the host
		//If length equals 0xFF, reset the controller
	} else if (((RxBoF & 0x7F) == config.board_id)
			&& ((SensorlessState == STANDBY)||(SensorlessState == STOP))) {

		if (RXDataLen == 0xFF)					// 0xFF length is a reset signal
			NVIC_SystemReset();					// reset the controller

		//Check config data length
		//if greater than zero - we have a config data
		//Try to store in the FLASH
		if (RXDataLen > 0) {
			//Validate message checksum
			checksum = 0;
			for (i = 0; i < (sizeof(config)); i++) {
				ptr_ram_config[i] = RXBUF[i + 1];
				checksum = checksum ^ ptr_ram_config[i];
			}

			if (!checksum) {
				//Checksum OK - write the config to FLASH
				res = write_config_to_flash();
				if (!res) {
					Blink(3, 500);			// Blink the LED 10 times 500ms each
					NVIC_SystemReset();		// reset the controller
				}
				// 3 long and 3 short flashes - all OK

			}
			//Otherwise, restore the config from FLASH
			else {
				read_flash_config();
				Blink(10, 50);				// Blink the LED 10 times 50ms each
											// 5 long flashes - config checksum wrong
											// back to original config
			}
		} else {
			//
			//Read and return the configuration data to the host

			TxConfigFrameBuilder();
		}

	}
}

/******************************************************************************
 * TxFrameBuilder - builds and sends the reply
 *****************************************************************************/
void TxFrameBuilder(void) {									// Build the reply
	uint8_t * pntRPM;
	uint8_t * pntCurrent;

	//uint8_t * pntPWM;

	if ((RxBoF == config.board_id) && (!(RxBoF & 0x80))) {// Reply only if the motor ID match

		TXDataLen = 6;							// Send 6 bytes, 2 bytes RPM, 2 bytes current, 1 byte temperature, 1 byte Board ID
		TXDataCount = TXDataLen;

		pntCurrent = (uint8_t*) &mCurrent;

		pntRPM = (uint8_t*) &mFilteredRPM;

		TXBUF[0] = pntRPM[1];
		TXBUF[1] = pntRPM[0];

		TXBUF[2] = pntCurrent[1];
		TXBUF[3] = pntCurrent[0];

		TXBUF[4] = bTemperature;
		TXBUF[5] = config.board_id;

		UARTIRQSend();
	}
}

/******************************************************************************
 * TxFrameBuilder - builds and sends the reply
 *****************************************************************************/
void TxConfigFrameBuilder(void) {							// Build the reply
	uint16_t i;

	for (i = 0; i < (sizeof(config)); i++) {
		TXBUF[i] = ptr_ram_config[i];
	}

	TXDataLen = 56;							// Send 56 bytes config
	TXDataCount = TXDataLen;

	UARTIRQSend();

}

/******************************************************************************
 ** Function name: Call_FastEvent
 **
 ** Descriptions:
 **
 ** parameters:
 ** Returned value:
 **
 ******************************************************************************/
void Call_FastEvent(void) {

	// Do the integration. We have to filter the input as the motor cannot respond to sharp changes
	// of the Set Point

	if (SensorlessState == RUN){
		if (mSP > tmp_mSP)
			mSP = mSP - config.delta_sp;
		else
			mSP = mSP + config.delta_sp;

	}

	if (SensorlessState == STANDBY) {
		if (mSP > tmp_mSP)
			mSP = mSP - config.min_rpm;
		else
			mSP = mSP + config.min_rpm;

	}

	//if setpoint reaches the target setpoint +/- 2 increments
	//ramping ends - set setpoint to the requested one

	if (abs(mSP - tmp_mSP) < 2 * config.delta_sp)
		mSP = tmp_mSP;

	if (SensorlessState == RUN) {
		vPID_RPM();
	}

	//if setpoint less the minimum RPM - set it to 0
	if (abs(mSP) < config.min_rpm) {
		SensorlessState = STOP;
	}
}

/******************************************************************************
 ** Function name:
 **
 ** Descriptions:
 **
 ** parameters:
 ** Returned value:
 **
 ******************************************************************************/
void Call_MedEvent(void) {
	/*
	 extern uint32_t	B_ADCv;				//ADC reference voltage in millivolts
	 extern uint8_t		B_ADCc;				//ADC used to monitor the motor current
	 extern uint8_t		B_ADCt;				//ADC used to monitor the board temperature
	 extern uint32_t	B_ADCr;				//ADC nubmer of discrettes
	 extern uint16_t	B_ADCs;				//Shunt resistance in milliohms
	 extern uint16_t	B_ADCa;				//ADC ampliffication
	 extern uint16_t	B_ADCo; 			//ADC voltage offset
	 */
	uint32_t VoltageReadout, CurrentReadout;

	CurrentReadout = ADCRead(B_ADCc); // read the current ADC

	//mCurrent = (B_ADCv*CurrentReadout*(B_ADCr1+B_ADCr2)/(B_ADCr*B_ADCr2) - B_ADCo) * 1000/ (B_ADCs*B_ADCa);
	//* 1000 to be in milliamps
	// and now something that works....

	//CurrentReadout = 10;// DEBUG!!!!!!!!!!!!!! REMOVE AFTER PROPER CURRENT READOUT !!!!!!!!!!!!!!!!!!!!!

	VoltageReadout = B_ADCRefv * CurrentReadout * 1000 / B_ADCr;

	if (VoltageReadout < B_ADCo)
		VoltageReadout = B_ADCo;

	mCurrent = (VoltageReadout - B_ADCo) / (B_ADCs * B_ADCa); //in mA

	// Filter the current ZC detection with earlier measurements through an IIR filter.
	//mFilteredCurrent = (CURRENT_IIR_COEFF_A * mCurrent
	//		+ CURRENT_IIR_COEFF_B * mFilteredCurrent)
	//		/ (CURRENT_IIR_COEFF_A + CURRENT_IIR_COEFF_B);

	if (mCurrent > config.max_current) {

		OverCurrentFlag = 1; // if the motor current exceeds the max allowed - disable the motor
	}

	//if ((abs(mFilteredRPM) > M_MaxRPM) && (SensorlessState == RUN)) //detect stalling
	//	SensorlessState = STOP;

	//if (ZC_stall_count > config.zc_stall_count) { // execute start sequence only if stalled
	//if(SensorlessState == RUN)
	//SensorlessState = STOP;
	//}

	//ZC_stall_count = 0;

}
/******************************************************************************
 ** Function name:
 **
 ** Descriptions:
 **
 ** parameters:
 ** Returned value:
 **
 ******************************************************************************/
void Call_SlowEvent(void) {
	uint8_t i;
	uint32_t res = 0;

	res = ADCRead(B_ADCt); // read the temperature ADC

	res = res * bThermPullup / (1024 - res); //actual resistance of the thermistor

	for (i = 0; res < THERM[i]; i++)
		;		//find rough entry in the resistance table

	bTemperature = -40 + i * 5
			- (res - THERM[i]) * 5 / (THERM[i - 1] - THERM[i]); // linear interpolation in between 2 entries in the table

	if (bTemperature > config.max_temp) {
		OverTempFlag = 1; 				// 0 - temperature OK, 1- overheat!!!
	} else
		OverTempFlag = 0;			// Enable the motor if T < 100 deg Celsius

	if (OverTempFlag || OverCurrentFlag)
		// Disable the Motor
		EnableMotor(FALSE);		// if overheat or overload - disable the motor

	if (!OverTempFlag && !OverCurrentFlag)
		//Enable the motor
		EnableMotor(TRUE);					// if OK - enable the motor;

	if (BreakFlag > 2)						// 3 break signals received
		NVIC_SystemReset();					// reset the controller

}

/******************************************************************************
 ** Function name:
 **
 ** Descriptions:
 **
 ** parameters:
 ** Returned value:
 **
 ******************************************************************************/
void EnableMotor(uint8_t bEnable) {

	mEnable = bEnable;		// Set mEnabl flag to let BLDC stop switching phases

	if (bEnable)
		GPIOSetValue(PORT2, CPLD_ENABLE_BIT, 0);	//Enable CPLD
	else
		GPIOSetValue(PORT2, CPLD_ENABLE_BIT, 1);	//Disable CPLD
}

/******************************************************************************
 ** Function name:
 **
 ** Descriptions:
 **
 ** parameters:
 ** Returned value:
 **
 ******************************************************************************/
void Appl_Init(void) {

	EnableMotor(FALSE);

	NVIC_SetPriority(TIMER_16_1_IRQn, 0);
	NVIC_SetPriority(EINT1_IRQn, 1);
	NVIC_SetPriority(ADC_IRQn, 2);
	NVIC_SetPriority(UART_IRQn, 2);
	NVIC_SetPriority(SysTick_IRQn, 3);

	/****************************************************/
	/* SYSTICK 											*/
	/****************************************************/
	SysTick_Config(SysTick_VALUE);

	/****************************************************/
	/* FREERUNNING COUNTER								*/
	/****************************************************/
	// Setup Timer16 1 as free running counter for e.g. RPM calc..
	// Enable the clock to Timer16 1
	LPC_SYSCON->SYSAHBCLKCTRL |= (1 << 8);
	// Set the prescaler to 48 to get 1 uS resolution
	LPC_TMR16B1->PR = SystemCoreClock / 1000000;
	//RESET timer16 1
	LPC_TMR16B1->TCR = 1 << 1;
	//ENABLE timer16 1
	LPC_TMR16B1->TCR = 1;
	// reset on MR1
	LPC_TMR16B1->MCR = 1 << 4;

	/****************************************************/
	/* GPIO	INIT										*/
	/****************************************************/
	GPIOInit();											// Init GPIOs

	GPIOSetDir(LED_PORT, LED_BIT, 1); 					//Set LED pin
	GPIOSetValue(LED_PORT, LED_BIT, 0);					//LED off

	/****************************************************/
	/* BLDC MOTOR INIT									*/
	/****************************************************/
	vBLDC_Init();

	Delay_ms(1000);										//wait for 1 sec;

	/****************************************************/
	/* UART INIT      									*/
	/****************************************************/
	/* Initialize UART */

	SystemCoreClockUpdate();
	UARTInit(oBaudRate);

	/* Initialize ADC  */
	ADCInit(ADC_CLK);

	/*Initialize I2C */

	if (I2CInit((uint32_t) I2CMASTER) == FALSE) /* initialize I2c */
	{
		while (1)
			; /* Fatal error */
	}

	//Clear Current Trip Bit
	//Only necessary for NXP Evaluation Board
	//May delete in real life controller

	//GPIOSetDir(PORT3, CURRENT_TRIP_BIT, 1);
	//GPIOSetValue(PORT3, CURRENT_TRIP_BIT, 0);

	//GPIOSetDir(PORT1, CURRENT_SELECT_BIT, 1);
	//GPIOSetValue(PORT1, CURRENT_SELECT_BIT, 0);

	GPIOSetDir(DEBUG_PORT, DEBUG_BIT, 1);   	//debug pin !!!!!!!!!!!!!!!
	GPIOSetValue(DEBUG_PORT, DEBUG_BIT, 0);

	GPIOSetDir(PORT2, CPLD_ENABLE_BIT, 1);   	//CPLD enable pin
	GPIOSetValue(PORT2, CPLD_ENABLE_BIT, 1);	//Disable CPLD

	GPIOSetDir(PORT3, LIN_ENABLE_BIT, 1);   	//LIN enable pin
	GPIOSetValue(PORT3, LIN_ENABLE_BIT, 1);		//Enable LIN
	//GPIOSetValue(PORT3, LIN_ENABLE_BIT, 0);		//Disable LIN

	LPC_IOCON->PIO0_1 &= ~0x07;
	LPC_IOCON->PIO0_1 |= 0x01;		//Set CPLD clock PIO

	LPC_SYSCON->CLKOUTCLKSEL = 3;			//Select CPLD main clock source
	LPC_SYSCON->CLKOUTDIV = 1;				//Select CPLD clock divider
	LPC_SYSCON->CLKOUTUEN = 0;				//
	LPC_SYSCON->CLKOUTUEN = 1;				//Update CPLD clock

	cnt_FastEvent = oFastEvent;			//Fast Event time cycle = 5 ms
	cnt_MedEvent = oMedEvent;			//Med Event time cycle  = 100 ms
	cnt_SlowEvent = oSlowEvent;			//Slow Event time cycle = 500s

	//////////////////////////////////////////

}

/******************************************************************************
 * MAIN
 *****************************************************************************/

int main(void) {

	Appl_Init();

	//Enable communication time (timer16_1) and BEMF interrupts
	NVIC_EnableIRQ(TIMER_16_1_IRQn);
	NVIC_EnableIRQ(EINT1_IRQn);

	// Enable the Motor
	EnableMotor(TRUE);


	Blink(3, 100);							// Blink the LED 3 times 100ms each

	//Init the receiver
	RxFrameReceivedFlag = 0; //Clear frame received flag, wait for frame

	//Loop forever
	while (1) {

		if (RxFrameReceivedFlag) { 		//if valid frame received -

			RxFrameParser();			// parse and execute
			RxFrameReceivedFlag = 0; 	// clear flag and wait for another frame
		}

		// Event scheduler
		if (Flag_FastEvent) {
			Flag_FastEvent = 0;
			Call_FastEvent();
		}
		if (Flag_MedEvent) {
			Flag_MedEvent = 0;
			Call_MedEvent();
		}
		if (Flag_SlowEvent) {
			Flag_SlowEvent = 0;
			Call_SlowEvent();
		}

		switch (SensorlessState) {
		case STOP: {

			//disable comm TIMER IRQ HERE!!!!!
			LPC_TMR16B1->MCR &= ~(1 << 3) & ~(1 << 4);

			//Set free run state
			mCMTStep = 'C';
			vBLDC_LoadMatch();

			free_run_counter = 0; //reset the freе run counter and check if the motor is still running

			Delay_ms(FREE_RUNNING_DELAY);

			//If stopped - go to STANBY
			if (free_run_counter < FREE_RUNNING_COUNTER)
				SensorlessState = STANDBY;

			mFilteredRPM = 0;
		}
			break;

		case STANDBY: {

			//Disable BEMF pins interrupts
			LPC_GPIO1->IE = 0x00;

			//Delay_ms(config.stop_delay);

			//Init the stall, stabilise and run counters
			//as well as the delta ramping
			ZC_count = 0;

			//Try to start if the received set point not 0 and direction + SP are aligned
			if (abs(mSP) > config.min_rpm) {

				//calculate direction
				if (mSP < 0)
					mDirection = 1;
				else
					mDirection = 0;

				SensorlessState = ALIGN;
			}

		}
			break;

		case ALIGN: {

			//Calculate the align duty
			pwmLoadVal = pwmPeriod - pwmPeriod * config.allign_duty / 100;

			//Start from step 0
			mCMTStep = 0;
			vBLDC_LoadMatch();
			Delay_ms(config.align_delay);

			//Init the commutation timer with a preset value - START_COMM_TIME must be chosen carefully !!!
			//When START_COMM_TIME, ALIGN_DUTY and ALIGN_DELAY are selected properly, the motor will start immediately
			comm_Time = config.start_comm_time;
			//Init the Zero Crossing Time variables
			filtered_ZC_Time = comm_Time / 2;
			zero_crossing_time = comm_Time / 2;
			//Reset the commutation timer
			LPC_TMR16B1->TCR = 1 << 1;
			//Enable commutation timer
			LPC_TMR16B1->TCR = 1;
			//Clear all timer interrupts
			LPC_TMR16B1->IR = 0xFFFFFFFF;
			//Enable comm timer IRQ and comm timer reset
			LPC_TMR16B1->MCR = (1 << 3) | (1 << 4);

			//Go to RAMP state
			//The comm timer will expire and will commutate the phases
			//As the motor is practically stalled,the Zero Crossing interrupt will happen fairly quickly
			//The next commutation time ( as calculated in handlers.c->PIOINT1_IRQHandler
			//will be comm_Time / 2 + zero_crossing_time. A little bit longer than comm_Time / 2
			//In the next commutation the zero_crossing_time will be a little bit larger
			//so in each step of the ramping the comm_Time time gets larger and closer to
			//the needed commutation time @ the selected RAMP_DUTY
			//At certain point the zero_crossing_time gets = comm_Time / 2
			//and the motor is synchronised.

			pwmLoadVal = pwmPeriod - pwmPeriod * config.ramp_duty / 100;
			//Init the comm timer in order to commutate quicly
			LPC_TMR16B1->MR1 = 10;  //Commutate immediately!!

			SensorlessState = RAMP;

		}
			break;
		case RAMP: {
			//When we reach a stable PLL lock
			if (ZC_count > config.zc_count) {
				//The motor is running here and the PLL is locked
				//change state to RUN

				mSP = (pwmMaxPeriod - pwmLoadVal) * M_MaxRPM / pwmMaxPeriod;

				//take into account the direction
				if (mDirection)
					mSP = 0 - mSP;

				SensorlessState = RUN;

			}
		}
			break;

		case RUN: {

			if ((mDirection && (mSP > 0)) || (!mDirection && (mSP < 0))
					|| (abs(mSP) < config.min_rpm)) {
				//Stop if direction changes or set point close to 0
				SensorlessState = STOP;
			}
		}
			break;
		}

	}
}

