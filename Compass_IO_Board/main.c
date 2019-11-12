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
#include "LSM9DS1.h"
#include "CMPS11.h"
#include "sys_config.h"

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
extern uint32_t pwmPeriodPinger, pwmPeriodServo, pwmPeriodLights, oFastEvent, oMedEvent, oSlowEvent;
extern uint32_t pwmLoadVal;			//PWM match registers load value
extern uint32_t pwmMaxPeriod;	//PWM max allowed pulse length register value
extern uint32_t pwmMinPeriod;	//PWM min allowed pulse length register value

extern uint32_t RPM_Const, comm_Time, ZC_count;
extern uint32_t filtered_ZC_Time, zero_crossing_time, pwmPIDVal;
extern uint32_t cnt_FastEvent, cnt_MedEvent, cnt_SlowEvent;

extern volatile uint8_t I2CSlaveBuffer[BUFSIZE];
extern uint16_t PingerCounter;

uint8_t SensorlessState = STOP, i2c_enabled = FALSE;

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

extern int32_t pwmSP_SERVO[6];
extern uint32_t pwmZeroSP;

IMU_REPORT IMU;

uint8_t * ptrIMU = (uint8_t*) &IMU;

CMPS11_REPORT CMPS;

uint8_t * ptrCMPS = (uint8_t*) &CMPS;



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

			if (RxBoF == 7){
				PingerCounter = 0;
				LPC_TMR16B0->TCR &= ~0x02;
			}

			if (RXDataLen == 0xFF)				// 0xFF length is a reset signal
				NVIC_SystemReset();					// reset the controller

			uint8_t i;
			int16_t tmpSPPWM;

			i=0; //Not used

				tmpSPPWM = (int32_t) ((RXBUF[(config.board_id << 1) - 1 + i * 2]
						<< 8) | RXBUF[(config.board_id << 1) + i * 2]); // get the set point RPM

				pwmSP_SERVO[i] = pwmZeroSP
						- (int32_t) ((tmpSPPWM * (pwmPeriodServo * 2.5 / 100))
								/ 32768); // get the set point servo pwm

			i=3; //Camera Servo

					tmpSPPWM = (int32_t) ((RXBUF[(config.board_id << 1) - 1 + i * 2]
							<< 8) | RXBUF[(config.board_id << 1) + i * 2]); // get the set point RPM

					pwmSP_SERVO[i] = pwmZeroSP
							- (int32_t) ((tmpSPPWM * (pwmPeriodServo * 2.5 / 100))
									/ 32768); // get the set point servo pwm
			for (i = 1; i < 3; i++) {

				//Pinger - i=2
				tmpSPPWM = (int32_t) ((RXBUF[(config.board_id << 1) - 1 + i * 2]
										<< 8) | RXBUF[(config.board_id << 1) + i * 2]); // get the set point RPM

				if (tmpSPPWM > 64)
					tmpSPPWM = 64;
				pwmSP_SERVO[i] = tmpSPPWM;

			}


			i=4; //Lights 1

			tmpSPPWM = (int32_t) ((RXBUF[(config.board_id << 1) - 1 + i * 2]
					<< 8) | RXBUF[(config.board_id << 1) + i * 2]); // get the set point RPM

			pwmSP_SERVO[i] =(int32_t) (pwmPeriodLights - ((uint16_t)tmpSPPWM * pwmPeriodLights)/ 65536); // get the set point to the power PWM1

			if (abs(pwmSP_SERVO[i] - pwmPeriodLights) > 0xFFF0)
					pwmSP_SERVO[i] = 0;

			i=5; //Lights 2

			tmpSPPWM = (uint32_t) ((RXBUF[(config.board_id << 1) - 1 + i * 2]
					<< 8) | RXBUF[(config.board_id << 1) + i * 2]); // get the set point RPM

			pwmSP_SERVO[i] =(int32_t) (pwmPeriodLights - ((uint16_t)tmpSPPWM * pwmPeriodLights)/ 65536); // get the set point to the power PWM1

			if (abs(pwmSP_SERVO[i] - pwmPeriodLights) > 0xFFF0)
					pwmSP_SERVO[i] = 0;


			vBLDC_LoadMatch();

		}

		TxFrameBuilder(); 			// transmit a status response

		//CONFIG FRAME. MSb = 1. Accept control message only if
		//the motor is in STOP_CLEAR_SP state and the Header byte matches the Board ID
		//If length equals Zero, return the configuration data to the host
		//If length equals 0xFF, reset the controller
	} else if (((RxBoF & 0x7F) == config.board_id)) {

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
				Blink(10, 50);				// Blink the LED 20 times 50ms each
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

	uint8_t i;
	if (((RxBoF & 0x7F) == config.board_id)) {// Reply only if the motor ID match

		IMU.UID = config.board_id;

		for (i = 0; i < (sizeof(IMU)); i++) {
			TXBUF[i] = ptrIMU[i];
		}

		TXDataLen = i;							// Send 56 bytes config
		TXDataCount = TXDataLen;

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

	TXDataLen = i;							// Send 56 bytes config
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

	uint32_t i, res;


	if (i2c_enabled) {

#if LSM9DS1
		readTemp();
		while(!tempAvailable()){
			i++;
			i++;
			i++;
			i++;
		}
		readTemp();

		IMU.temperature =  (I2CSlaveBuffer[1] << 8) | (I2CSlaveBuffer[0]);


		readMagAll();
		while(!magAvailable(ALL_AXIS)){
			i++;
			i++;
			i++;
			i++;
		}
		readMagAll();

		IMU.mx = (I2CSlaveBuffer[1] << 8) | (I2CSlaveBuffer[0]);
		IMU.my = (I2CSlaveBuffer[3] << 8) | (I2CSlaveBuffer[2]);
		IMU.mz = (I2CSlaveBuffer[5] << 8) | (I2CSlaveBuffer[4]);

		readGyroAll();
		while(!gyroAvailable(ALL_AXIS)){
			i++;
			i++;
			i++;
			i++;
		}

		readGyroAll();
		IMU.gx = (I2CSlaveBuffer[1] << 8) | (I2CSlaveBuffer[0]);
		IMU.gy = (I2CSlaveBuffer[3] << 8) | (I2CSlaveBuffer[2]);
		IMU.gz = (I2CSlaveBuffer[5] << 8) | (I2CSlaveBuffer[4]);

		readAccelAll();
		while(!accelAvailable(ALL_AXIS)){
			i++;
			i++;

		}

		readAccelAll();
		IMU.ax = (I2CSlaveBuffer[1] << 8) | (I2CSlaveBuffer[0]);
		IMU.ay = (I2CSlaveBuffer[3] << 8) | (I2CSlaveBuffer[2]);
		IMU.az = (I2CSlaveBuffer[5] << 8) | (I2CSlaveBuffer[4]);
#endif


#if CMPS11
		memset(ptrCMPS,0, sizeof(CMPS));

		res = CMPS11_read(ptrCMPS);
		if (res == I2C_OK){
			IMU.mx = CMPS.cmps_word;
			IMU.ax = CMPS.pitch;
			IMU.gx = CMPS.roll;
			IMU.temperature = CMPS.temp;
			IMU.gz = config.board_id <<8;
		}
#endif


	}

	IMU.AIN1 = ADCRead(7);
	IMU.AIN2 = ADCRead(1);
	IMU.AIN3 = ADCRead(6);
	res = ADCRead(5);

	res = res * bThermPullup / (1024 - res); //actual resistance of the thermistor

	for (i = 0; res < THERM[i]; i++)
		;		//find rough entry in the resistance table

	IMU.AIN_temp = -40 + i * 5
			- (res - THERM[i]) * 5 / (THERM[i - 1] - THERM[i]); // linear interpolation in between 2 entries in the table

			//LPC_GPIO0->DATA =  (1<<3);
			//LPC_GPIO2->DATA =  (1<<1);

	IMU.DIN1 = ((LPC_GPIO0->DATA & (1 << 3)) >> 2)
			| ((LPC_GPIO2->DATA & (1 << 1)) >> 1);
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

	//pwmSP_SERVO[1] = 64; //!!!! ДЕБУГ!!!!!!!!!!!!!!!!!!!!!!!!!!!
	//PingerCounter = 0;//!!!! ДЕБУГ!!!!!!!!!!!!!!!!!!!!!!!!!!!
	//LPC_TMR16B0->TCR &= ~0x02;//!!!! ДЕБУГ!!!!!!!!!!!!!!!!!!!!!!!!!!!


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
	NVIC_SetPriority(EINT2_IRQn, 1);
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


	reset_timer16(TIMER0);


	/****************************************************/
	/* GPIO	INIT										*/
	/****************************************************/
	GPIOInit();											// Init GPIOs

	GPIOSetDir(LED_PORT, LED_BIT, 1); 					//Set LED pin
	GPIOSetValue(LED_PORT, LED_BIT, 0);					//LED off

	//GPIOSetDir(PORT2, D0_BIT, 0); 					//Set digital input 0 pin
	//GPIOSetDir(PORT0, D1_BIT, 0); 					//Set digital input 1 pin

	/****************************************************/
	/* BLDC MOTOR INIT									*/
	/****************************************************/
	vBLDC_Init();

	Blink(3, 100);							// Blink the LED 3 times 100ms each
											// 3 short blinks - application start
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
		//while (1)
		/* Not a Fatal error. We can live with it... */
		Blink(3, 500); // 3 long blinks - I2C fault
	}

#if LSM9DS1
		LSM9DS1(); // Init IMU
		uint16_t res = LSM9DS1_begin();

	if (!res) {
		Blink(5, 500);  //5 long blinks - NO COMPASS!!!!
		i2c_enabled = FALSE;
	} else {
		i2c_enabled = TRUE;
		magOffset(X_AXIS, config.p1);

		magOffset(Y_AXIS, config.p2);
	}
#endif

#if CMPS11
		uint16_t res = CMPS11_read(ptrCMPS);
	if ((res!=12) || (CMPS.fw_version !=FW_VERSION)) {
		Blink(5, 500);  //5 long blinks - NO COMPASS!!!!
		i2c_enabled = FALSE;
	} else {
		i2c_enabled = TRUE;
	}
#endif

	GPIOSetDir(PORT3, LIN_ENABLE_BIT, 1);   		//LIN enable pin
	GPIOSetValue(PORT3, LIN_ENABLE_BIT, 1);			//Enable LIN
	//GPIOSetValue(PORT3, LIN_ENABLE_BIT, 0);		//Disable LIN

	//LPC_IOCON->PIO0_1 &= ~0x07;
	//LPC_IOCON->PIO0_1 |= 0x01;		//Set CPLD clock PIO

	//LPC_SYSCON->CLKOUTCLKSEL = 3;			//Select CPLD main clock source
	//LPC_SYSCON->CLKOUTDIV = 1;				//Select CPLD clock divider
	//LPC_SYSCON->CLKOUTUEN = 0;				//
	//LPC_SYSCON->CLKOUTUEN = 1;				//Update CPLD clock

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

	// Enable the Motor
	EnableMotor(TRUE);

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

	}
}

