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

#include "LPC11xx.h"			/* LPC11xx Peripheral Registers */
#include "application.h"

//Freerunning timer/counter
uint32_t comm_Time;

/****************************************************/
/* Motor commutation table							*/
/****************************************************/
uint8_t CMT_tbl[2][8] = { { 0xF, 2, 0, 1, 4, 3, 5, 0xF }, { 0xF, 5, 3, 4, 1, 0,
		2, 0xF } };

uint16_t RPMBuffer[64] = { 0 };

/*PARAMETERS BLOCK*/
/****************************************************/
/* PWM parameterts									*/
/****************************************************/
//init params
uint32_t PWM_FS;			//PWM frequency in Hz

//Variable params
uint32_t pwmPeriod;			//PWM total pulse length register value
uint32_t pwmMaxPeriod;		//PWM max allowed pulse length register value
uint32_t pwmMinPeriod;		//PWM min allowed pulse length register value
uint32_t pwmLoadVal = 0xFFFF;		//PWM match registers load value
uint32_t pwmPIDVal;			//PWM output from the PID regulator

/****************************************************/
/* PID parameterts									*/
/****************************************************/
//init params
int32_t PID_MaxError;		//PID maximum allowed error
int32_t PID_MinError;		//PID minimum allowed error

//Variable params
int32_t pidIntError;			//PID integral error
int32_t pidLastError;			//PID last error
int32_t pidOutput = 0xFFFF;	//PID output to PWM match registers, init to minimum duty cycle

/****************************************************/
/* Motor parameterts								*/
/****************************************************/
//init params
uint16_t M_Kv;			//Motor Kv
uint16_t M_MaxV;			//Motor maximum voltage
uint16_t M_PolePairs;	//Motor pole pairs
int32_t M_MaxRPM;		//Motor max allowed RPM

//Variable params
uint32_t mRPM, mFilteredRPM;
int16_t mSP, tmp_mSP;			//Motor set point RPM
//int16_t mSP_OpenLoop;	//Motor set point in duty units
uint32_t mMinPeriod;		//Motor minimum allowed PWM match register value
uint8_t mBrake;			//0 - no brake, 1 - brake
uint8_t mEnable;		//0 - motor disabled, 1 - motor enabled
uint8_t mDirection, mRampDirection;		// Set point direction - 0 - CCW, 1 - CW
uint8_t mRealDirection;	// Real rotor direction - 0 - CCW, 1 - CW
int32_t mCurrent = 0, mFilteredCurrent = 0, stl_tmp = 0;//Motor current in milliamps
int8_t mCMTStep = 0;		//Motor commutation step

/****************************************************/
/* Board parameterts								*/
/****************************************************/
//init params
uint32_t device_SN[4];//Processor unique serial number. used to generate pseudo unique B_ID
uint16_t B_Voltage;			//Power suply voltage for MOSFET drivers

uint32_t B_ADCRefv;				//ADC reference voltage in millivolts
uint8_t B_ADCv;					//TBD - monitor the supplied voltage
uint8_t B_ADCc = 1;				//ADC 1 is used to monitor the motor current
uint8_t B_ADCt = 5;				//ADC 5 is used to monitor the board temperature
uint8_t B_ADCph = 7;			//ADC 7 is used to monitor the C phase voltage
uint32_t B_ADCr;				//ADC nubmer of discrettes
uint16_t B_ADCs;				//Shunt resistance in milliohms
uint16_t B_ADCa;				//ADC ampliffication
uint16_t B_ADCo; 			//ADC voltage offset in 0 input current
uint16_t B_ADCr1; 			//ADC R1 scale divider
uint16_t B_ADCr2; 			//ADC R2 scale divider

/****************************************************/
/* OS parameterts								*/
/****************************************************/
//init params
uint32_t oFastEvent;				//Fast Event time cycle in ms
uint32_t oMedEvent;					//Med Event time cycle in ms
uint32_t oSlowEvent;				//Slow Event time cycle in ms
uint32_t oUARTtimeout = 25;			//UART Timeout in ms

//Variable params
uint32_t oBaudRate;	//RS232baudrate

//RPM calculation constant

uint32_t RPM_Const, Min_Comm_Time, free_run_duty, free_run_counter;

//Timers prescaler

uint32_t TMRPrescaler;

extern uint32_t timer16_0_counter;
extern uint8_t SensorlessState;

extern uint32_t cnt_UART_Timeout;

//uint32_t BEMF_pin_mask = (1 << PHASE_A_PIN) | (1 << PHASE_B_PIN)
//		| (1 << PHASE_C_PIN);

uint32_t BEMF_pin_mask = (1 << PHASE_A_PIN);

uint32_t BEMF_level_mask = 0xFFFFFFFF;

/*PARAMETERS BLOCK END*/

config_t config;

char* ptr_flash_config = (char*)USER_FLASH_AREA_START;
char* ptr_ram_config = (char*)&config;

/******************************************************************************
 ** Function name:
 **
 ** Descriptions:
 **
 ** parameters:
 ** Returned value:
 **
 ******************************************************************************/
void vBLDC_Init(void) {

	//uint8_t res,i;
	//uint8_t* bid;

	/****************************************************/
	/* Set INIT parameterts or read from EEPROM			*/
	/****************************************************/
	PWM_FS = 20000;			//ESC PWM frequency 20000 Hz
	TMRPrescaler = 0;

	set_config();

	cnt_UART_Timeout = oUARTtimeout;

	mSP = 0;				//Initial RPM setpoint = 0
	tmp_mSP = 0;			//Initial RPM setpoint = 0
	mBrake = 0;  			//Disable brake

	/****************************************************/
	/* Calculate RPM Calcualtion Constant						*/
	/****************************************************/
	// RPM constant using HALL sensors. Constant is valid if CalculateRPM is called every A HALL sensor interrupt
	//RPM_Const = ((uint32_t)SystemCoreClock*60)/M_PolePairs ; // this is when the RPM is calculated in the Read HALL Handler
	//RPM constant using phase switching. Constant is valid if CalculateRPM is called every A Phase switching
	RPM_Const = 10000000 / M_PolePairs; //RPM_Const = SystemCoreClock / (5 * M_PolePairs); // this is when the RPM is calculated in the Read HALL Handler
	Min_Comm_Time = RPM_Const / M_MaxRPM;

	/****************************************************/
	/* Calculate Chip PWM parameterts						*/
	/****************************************************/

	pwmPeriod = SystemCoreClock / (TMRPrescaler + 1) / PWM_FS; //PWM total pulse length register value = 0% PWM (4800/12=4000)

	pwmMaxPeriod = pwmPeriod * 0.96; //PWM max allowed pulse length register value = 10%PWM (3600)
	pwmMinPeriod = 0; //pwmPeriod * 0.05;//PWM min allowed pulse length register value = 94% PWM (240)

	/**************************************************************/
	/* Calculate motor minimum allowed PWM match register value   */
	/**************************************************************/

	//First see if the board voltage exceeds the max allowed motor voltage
	if (B_Voltage > M_MaxV)
		//Normalise PWM so that max PWM does not produce voltage higher than motor max voltage
		mMinPeriod = pwmMaxPeriod
				- (pwmMaxPeriod - pwmMinPeriod) * M_MaxV / B_Voltage;
	else
		//Otherwise assign the max PWM
		mMinPeriod = pwmMinPeriod;

	/****************************************************/
	/* PWM (FET drivers)								*/
	/****************************************************/
	/* PIO0_1	PHASEA_H  CT32B0_MAT2					*/
	/* CHANGED TO -    									*/
	/* PIO2_7	PHASEA_H  CT32B0_MAT2					*/
	/* PIO0_11	PHASEA_L  CT32B0_MAT3					*/
	/* PIO0_8	PHASEB_H  CT16B0_MAT0					*/
	/* PIO0_9	PHASEB_L  CT16B0_MAT1					*/
	/* PIO1_1	PHASEC_H  CT32B1_MAT0					*/
	/* PIO1_2	PHASEC_L  CT32B1_MAT1					*/
	/****************************************************/
	/* Setup Phase A high and low side PWM signals */
	init_timer32PWM(TIMER0, pwmPeriod, (MATCH0 | MATCH2 | MATCH3));
	/* Setup Phase B high and low side PWM signals */
	init_timer16PWM(TIMER0, pwmPeriod, (MATCH0 | MATCH1), CAP_DISABLE);
	/* Setup Phase C high and low side PWM signals */
	init_timer32PWM(TIMER1, pwmPeriod, (MATCH0 | MATCH1));

	pwmLoadVal = pwmMaxPeriod; // make the PWM PID output = minimum

	//pre-scale all PWM timers

	LPC_TMR32B0->PR = TMRPrescaler;
	LPC_TMR16B0->PR = TMRPrescaler;
	LPC_TMR32B1->PR = TMRPrescaler;

	LPC_TMR32B0->PC = 0;
	LPC_TMR16B0->PC = 0;
	LPC_TMR32B1->PC = 0;

	/* Make sure no match occurs */
	vBLDC_();

	//Load correcting start values so that the edges of the PWM match... blipping NXP.....
	LPC_TMR16B0->TC = 12;
	LPC_TMR32B1->TC = 24;

	// enable all timers for motor driving

	LPC_TMR32B0->TCR = 1;
	LPC_TMR16B0->TCR = 1;
	LPC_TMR32B1->TCR = 1;

	//Init BEMF digital inputs
	GPIOSetDir(PHASE_A_PORT, PHASE_A_PIN, 0);	// PHASE A
	GPIOSetDir(PHASE_B_PORT, PHASE_B_PIN, 0);	// PHASE B
	GPIOSetDir(PHASE_C_PORT, PHASE_C_PIN, 0);	// PHASE C

	// Setup the interrupt, seq: portNum, bitPosi, sense = level, single = NA, event = hi=1/lo=0 defined in LoadMatch()
	GPIOSetInterrupt(PHASE_A_PORT, PHASE_A_PIN, 1, 0, 1); //0 for inverted, 1 for non-inverted comparator input
	GPIOSetInterrupt(PHASE_B_PORT, PHASE_B_PIN, 1, 0, 1);
	GPIOSetInterrupt(PHASE_C_PORT, PHASE_C_PIN, 1, 0, 1);

}

/******************************************************************************
 ** Function name:		blcd_Commutate
 **
 ** Descriptions:
 **
 ** parameters:
 ** Returned value:
 **
 ******************************************************************************/
void vBLDC_LoadMatch(void) {


	// calculate direction
	//if (mSP < 0)
	//	mDirection = 1;
	//else
	//	mDirection = 0;

	switch (mCMTStep) {
	case 0: {

		//PHA_HI = pwmLoadVal;	// H1
		PHA_LO = OFF;			// L1
		PHB_HI = OFF;			// H2
		//PHB_LO = pwmLoadVal;	// L2
		PHC_HI = OFF;			// H3
		PHC_LO = OFF;			// L3

		PHA_HI = ON; 		// ON; 		//
		PHB_LO = pwmLoadVal; 	// L2

		BEMF_pin_mask = (1 << PHASE_C_PIN);
		BEMF_level_mask = (mDirection << PHASE_C_PIN);

	}
		break;
	case 1: {

		PHA_HI = OFF;
		PHA_LO = OFF;
		PHB_HI = OFF;
		//PHB_LO = pwmLoadVal;
		//PHC_HI = pwmLoadVal;
		PHC_LO = OFF;

		PHB_LO = pwmLoadVal;
		PHC_HI = ON; 		////ON; //

		BEMF_pin_mask = (1 << PHASE_A_PIN);
		BEMF_level_mask = (!mDirection << PHASE_A_PIN);

	}
		break;
	case 2: {

		PHA_HI = OFF;
		//PHA_LO = pwmLoadVal;
		PHB_HI = OFF;
		PHB_LO = OFF;
		//PHC_HI = pwmLoadVal;
		PHC_LO = OFF;

		PHA_LO = pwmLoadVal;
		PHC_HI = ON; //pwmLoadVal; 		////ON; //

		BEMF_pin_mask = (1 << PHASE_B_PIN);
		BEMF_level_mask = (mDirection << PHASE_B_PIN);

	}
		break;
	case 3: {

		PHA_HI = OFF;
		//PHA_LO = pwmLoadVal;
		//PHB_HI = pwmLoadVal;
		PHB_LO = OFF;
		PHC_HI = OFF;
		PHC_LO = OFF;

		PHA_LO = pwmLoadVal;
		PHB_HI = ON; 		////ON; //

		BEMF_pin_mask = (1 << PHASE_C_PIN);
		BEMF_level_mask = (!mDirection << PHASE_C_PIN);

	}
		break;
	case 4: {

		PHA_HI = OFF;
		PHA_LO = OFF;
		//PHB_HI = pwmLoadVal;
		PHB_LO = OFF;
		PHC_HI = OFF;
		//PHC_LO = pwmLoadVal;

		PHB_HI = ON; 		////ON; //
		PHC_LO = pwmLoadVal;

		BEMF_pin_mask = (1 << PHASE_A_PIN);
		BEMF_level_mask = (mDirection << PHASE_A_PIN);

	}
		break;
	case 5: {

		//PHA_HI = pwmLoadVal;
		PHA_LO = OFF;
		PHB_HI = OFF;
		PHB_LO = OFF;
		PHC_HI = OFF;
		//PHC_LO = pwmLoadVal;

		PHA_HI = ON; 		////ON; //
		PHC_LO = pwmLoadVal;

		BEMF_pin_mask = (1 << PHASE_B_PIN);
		BEMF_level_mask = (!mDirection << PHASE_B_PIN);

	}
		break;
	case 'B': // break
	{
		PHA_LO = ON;
		PHA_HI = OFF;
		PHB_LO = ON;
		PHB_HI = OFF;
		PHC_LO = ON;
		PHC_HI = OFF;
	}
		break;
	case 'C': // coach / free run - stop with 2% of the full duty
	{
		free_run_duty = FREE_RUN_DUTY; // pwmPeriod - pwmPeriod * 2 / 100;
		PHA_HI = OFF;
		PHA_LO = pwmPeriod - free_run_duty;//OFF;
		PHB_HI = OFF;
		PHB_LO = pwmPeriod - free_run_duty;//OFF;
		PHC_HI = OFF;
		PHC_LO = pwmPeriod - free_run_duty;//OFF;
	}
		break;
	default: // should not happen, to be save don't do a thing until the next interrupt
	{   // disable all matches
		PHA_HI = OFF;
		PHA_LO = OFF;
		PHB_HI = OFF;
		PHB_LO = OFF;
		PHC_HI = OFF;
		PHC_LO = OFF;
	}
		break;
	}

	//vBLDC_GetCommStep();
	//Get next step

}

/******************************************************************************
 ** Function name:		bldc_PWM
 **
 ** Descriptions:
 **
 ** parameters:
 ** Returned value:
 **
 ******************************************************************************/
void vBLDC_(void) {
	/* set all timers to MAX for NO match! */
	PHA_HI = 0xFFFF;
	PHA_LO = 0xFFFF;
	PHB_HI = 0xFFFF;
	PHB_LO = 0xFFFF;
	PHC_HI = 0xFFFF;
	PHC_LO = 0xFFFF;
}

/******************************************************************************
 ** Function name:		vBLDC_GetCommStep
 **
 ** Descriptions:
 **
 ** parameters:
 ** Returned value:
 **
 ******************************************************************************/

void vBLDC_GetCommStep(void) {

	if (mDirection) {

		mCMTStep += 1;
		if (mCMTStep > 5)
			mCMTStep = 0;

	} else {
		mCMTStep -= 1;

		if (mCMTStep < 0)
			mCMTStep = 5;
	}

	/* Check whether there is a setpoint and an enable */
	if (!tmp_mSP || !mEnable) {
		/* If no setpoint and brake is enable BREAK! */
		if (mBrake) {
			mCMTStep = 'B';
		}
		/* Else let it run free */
		else {
			mCMTStep = 'C';
		}
	}

}

/******************************************************************************
 ** Function name:		PID_calc_cntr
 **
 ** Descriptions:
 **
 ** parameters:
 ** Returned value:
 **
 ******************************************************************************/
void vPID_RPM(void) {
	int32_t Pout, Iout, Dout, output, error;

	/* Calculate the error from setpoint and actual RPM  */
	error = abs(mSP) - mFilteredRPM;

	/* Calculate and update the Integrated Error */
	if (abs(error) > MIN_PID_ERROR)
		pidIntError += error * oFastEvent; //integral error = integral error + error * dt

	/* Calculate the integral term  */
	Iout = (pidIntError * config.pid_ki) >> 12;

	/* Calculate the derivative term */
	Dout = ((error - pidLastError) * config.pid_kd / oFastEvent) >> 16;

	/* Calculate the proportional term  */
	Pout = (config.pid_kp * error) >> 4;

	//Xout = Xout >> 10; // Normalise the output i.e. all the members are actually / 1000

	/* Calculate the output from the Pout, Iout, Dout  */
	output = (Pout + Iout + Dout);

	/* Save the error */
	pidLastError = error;

	if (config.pid_kp)
		pidOutput = pwmMaxPeriod - output;
	else
		//pidOutput = mSP_OpenLoop; // debug - if PID_Kp = 0 - open loop duty
		pidOutput = pwmMaxPeriod - pwmMaxPeriod * abs(mSP) / M_MaxRPM;

	// Cap the output if exceeding the max allowed parameters
	if (pidOutput < mMinPeriod)
		pidOutput = mMinPeriod;
	if (pidOutput > pwmMaxPeriod)
		pidOutput = pwmMaxPeriod;

	pwmLoadVal = pidOutput;
}

//sets hard coded and configurable parameters
void set_config() {

	oBaudRate = 19200;

	B_ADCRefv = 3300;		//Board ADC reference voltage	3300mV

	B_ADCr = 1024;			//ADC nubmer of discrettes = 1024 for LPC111x
	B_ADCs = 10;			//Shunt resistance in milliohms - 10 resistors * 0.1 ohm = 0.01 ohm
	B_ADCa = 10;			//ADC ampliffication - op amp with K=10
	B_ADCo = 0; 			//ADC offset voltage

	PID_MaxError = 800;	    //PID maximum allowed error
	PID_MinError = -800;	//PID minimum allowed error

	B_Voltage = 15;			//Board voltage = 15V

	M_Kv = 280;				//Motor Kv = 150 (Turnigy 280kV)
	M_MaxV = 24;			//Motor maximum voltage = 24V (Turnigy 280kV)
	M_PolePairs = 7;		//Motor pole pairs = 7 (Turnigy 280kV)

	M_MaxRPM = B_Voltage * M_Kv; 	//Motor max RPM

	oFastEvent = 5;				//Fast Event time cycle = 2 ms
	oMedEvent = 100;			//Med Event time cycle  = 200 ms
	oSlowEvent = 500;			//Slow Event time cycle = 1s
	oUARTtimeout = 25;			//UART Timeout in ms

	int res = read_flash_config();
	if(config.min_rpm<99)
		config.min_rpm=99;

	if (!res) {
		set_default_config();
		res=write_config_to_flash();
		Blink(3, 500);										// Blink the LED 10 times 500ms each
		Delay_ms(1000);
		if(res){
			Blink(10, 500);										// Blink the LED 10 times 500ms each
		}
	}
}

int read_flash_config(void) {
	uint16_t j, chksm;

	chksm = 0;
	//memcpy(ptr_ram_config, ptr_flash_config, sizeof(config));
	for(j=0;j<(sizeof(config));j++)
	{
		ptr_ram_config[j]=ptr_flash_config[j];
		chksm = chksm ^ ptr_ram_config[j];
	}
	if(chksm || (ptr_ram_config[0]==0xFF))
		return FALSE;
	else
		return TRUE;
}

void set_default_config()
{
uint16_t i, checksum;
	config.board_id = 1;			//Board ID - 0 and 0xFF - reserved, 1 onwards - OK.
									// MUST BE UNIQUE FOR EACH BOARD !!!!!!!!

	config.allign_duty = 10; 		//This duty cycle defines the momentum the motor will apply when started
									//(from STOP to ALLIGN state)
									//Too low and the motor wil not align,
									//Too high will cause overloading tthe coils/power supply)
									//In %  !!!!!!!!!!!! IMPORTANT TO START !!!!!!!!!!!!!!!!!

	config.ramp_duty = 15;			//The RAMP_DUTY defines the momentum of the ramping stage.
									//Must be close to ALIGN_DUTY
									//In %	!!!!!!!!!!!! IMPORTANT TO START !!!!!!!!!!!!!!!!!

	config.delta_sp = 40;			//defines the set point increment for soft start
									//At the end of the ramping the duty is different than the start
									//smaller if no load, larger if heavier load is applied

	config.stop_delay = 100;		//Defines the time in STOP state. in mS;
	config.align_delay = 100;		//Defines the time in ALLIGN state. in mS;
	config.start_comm_time = 8000;	//Initial commutatation time. in uS;

	config.blanking_time = 130;		//Zero Crossing check Blanking Time
									//in uS 70 - 	around 10000 RPM max (7 pole pairs motor)
									//		90 - 	around  7500 RPM max (7 pole pairs motor)
									//		150 -   around  4500 RPM max (7 pole pairs motor);

	config.zc_correction = 0;		//Zero Crossing Correction.
									//Due to lag etc, the ZC moment may have to be corrected. in uS;

	config.acc_zc_gain = 32;		//Zero Crossing error multiplier.If the error is positive.
									//numeric value, real gain is ACC_ZC_GAIN/32;

	config.dec_zc_gain = 32;		//Zero Crossing error multiplier.If the error is negative.
									//numeric value, real gain is DEC_ZC_GAIN/32;

	config.zc_count = 10;			//Counts the Zero Crossings close to 1/2 comm_time.
									//After the ramping has completed.
									//If we have larger number of crossings - we have PLL lock.

	config.zc_stall_count = 0;	//Counts the low delta Phase Voltage events.
									//If phase voltage is not changing for greater number of cycles - we have a stall

									//or if RPM >> Max_RPM is used to detect stall
									//stall RPM is zc_stall_count*10_Max_RPM

	config.stall_phase_voltage = 0;	//Delta Phase Voltage threshold. If lower - stall condition
									//Only if ADC is used to measure high and low values of the phase waveform

	config.rpm_IIR_A = 4;			//filter params for RPM calculation
	config.rpm_IIR_B = 4;
	config.current_IIR_A = 8;		//filter params for current calculation
	config.current_IIR_B = 8;
	config.comm_IIR_A = 1;			//filter params for communication time calculation
	config.comm_IIR_B = 63;


	config.max_current = 30000;		//Motor max allowed current - 30A(Turnigy 280kV)
	config.max_temp = 90;			//Motor max allowed temperature +90 deg Celsius
	config.pid_kp = 0;				//PID proportional default = 50, if 0 - the PID regulator is disabled
	config.pid_ki = 0;				//PID integral default = 2
	config.pid_kd = 0;				//PID differential default = 0
	config.min_rpm = 250;			//Motor min RPM = 180 (Turnigy 280kV)

	config.p1 = 0;					//Some spare configuration parameters.
	config.p2 = 0;
	config.p3 = 0;
	config.p4 = 0;
	config.p5 = 0;
	config.p6 = 0;
	config.p7 = 0;
	config.p8 = 0;
	config.checksum = 0;
	checksum = 0; //reset the checksum

	for(i=0;i<(sizeof(config));i++)
	{
		checksum = checksum ^ ptr_ram_config[i];
	}

	config.checksum = checksum;

}

int write_config_to_flash(){
	int res;

	//int     *serial_number;
    //serial_number = read_serial();

	prepare( TARGET_SECTOR, TARGET_SECTOR );
	res=erase( TARGET_SECTOR, TARGET_SECTOR );

	if(!res){
			prepare(TARGET_SECTOR, TARGET_SECTOR );
			res  = write( ptr_ram_config, ptr_flash_config, 1024 );
	}

return res;
}
