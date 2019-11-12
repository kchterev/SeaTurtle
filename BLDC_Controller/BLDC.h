/***********************************************************************
 * $Id:: BLDC.h 3871 2010-07-16 11:50:22Z gerritdewaard                $
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

#ifndef __BLDC_MOTOR_H 
#define __BLDC_MOTOR_H

/****************************************************/
/* PWM (FET drivers) connections					*/
/****************************************************/
/* PIO2_7	PHASE1_H  CT32B0_MAT2					*/
/* PIO0_11	PHASE1_L  CT32B0_MAT3					*/
/* PIO0_8	PHASE2_H  CT16B0_MAT0					*/
/* PIO0_9	PHASE2_L  CT16B0_MAT1					*/
/* PIO1_1	PHASE3_H  CT32B1_MAT0					*/
/* PIO1_2	PHASE3_L  CT32B1_MAT1					*/
/****************************************************/
/*
 #define H1REG			LPC_TMR32B0->MR2 // Phase 1 Hi
 #define L1REG			LPC_TMR32B0->MR3 // Phase 1 Lo
 #define H2REG			LPC_TMR16B0->MR0 // Phase 2 Hi
 #define L2REG			LPC_TMR16B0->MR1 // Phase 2 Lo
 #define H3REG			LPC_TMR32B1->MR0 // Phase 3 Hi
 #define L3REG			LPC_TMR32B1->MR1 // Phase 3 Lo
 */

#define PHA_HI			LPC_TMR32B0->MR2 // Phase 1 Hi
#define PHA_LO			LPC_TMR32B0->MR3 // Phase 1 Lo
#define PHB_HI			LPC_TMR16B0->MR0 // Phase 2 Hi
#define PHB_LO			LPC_TMR16B0->MR1 // Phase 2 Lo
#define PHC_HI			LPC_TMR32B1->MR0 // Phase 3 Hi
#define PHC_LO			LPC_TMR32B1->MR1 // Phase 3 Lo

#define OFF			0xFFFF
#define ON			0x0000


//CONFIG PARAMS ----------------------------------------------------------------------------------


typedef struct motor_config {
	uint8_t	board_id;	//Board identifier. // MUST BE UNIQUE FOR EACH BOARD !!!!!!!!
	uint8_t allign_duty;//This duty cycle defines the momentum the motor will apply when started.
	uint8_t ramp_duty;//The RAMP_DUTY defines the momentum of the ramping stage.
	uint8_t delta_sp;//Defines the acceleration. The setpoint is increase by delta_sp every fast event.
	uint16_t stop_delay;			//Defines the time in STOP state.
	uint16_t align_delay;			//Defines the time in ALLIGN state.
	uint16_t start_comm_time;		//Initial commutatation time.
	uint16_t blanking_time;		//Zero Crossing check Blanking Time.
	uint8_t zc_correction;	//Zero Crossing Correction. Due to lag etc, the ZC moment may have to be corrected.
	uint8_t acc_zc_gain;//Zero Crossing error multiplier.If the error is positive.
	uint8_t dec_zc_gain;//Zero Crossing error multiplier.If the error is negative.
	uint8_t zc_count;		//Counts the Zero Crossings close to 1/2 comm_time.
	uint8_t zc_stall_count;		//Counts the low delta Phase Voltage events.
	uint8_t stall_phase_voltage;	//Delta Phase Voltage threshold.
	uint8_t rpm_IIR_A;				//Filter params for RPM calculation.
	uint8_t rpm_IIR_B;
	uint8_t current_IIR_A;			//Filter params for current calculation.
	uint8_t current_IIR_B;
	uint8_t comm_IIR_A;		//Filter params for communication time calculation.
	uint8_t comm_IIR_B;
	uint32_t max_current;			//Motor max allowed current in milliamps.
	uint16_t max_temp;				//Motor max allowed temperature.
	uint16_t pid_kp;//PID proportional param. If 0 - the PID regulator is disabled. Open loop control.
	uint16_t pid_ki;				//PID integral param.
	uint16_t pid_kd;				//PID differential param.
	uint16_t min_rpm;//Minimum allowed RPM. Bellow this RPM the SP is set to 0.
	uint16_t p1;					//Some spare configuration parameters. RFU
	uint16_t p2;					//RFU
	uint16_t p3;					//RFU
	uint16_t p4;					//RFU
	uint16_t p5;					//RFU
	uint16_t p6;					//RFU
	uint16_t p7;					//RFU
	uint16_t p8;					//RFU
	uint8_t	checksum;				//XOR of all the params

} config_t;

//End of CONFIG PARAMS ----------------------------------------------------------------------------------


//if mSP - RPM too small - do not do integration in the PID
#define MIN_PID_ERROR 		10

#define PHASE_A_PORT		1
#define PHASE_A_PIN			5
#define PHASE_A_GPIODATA	LPC_GPIO1->DATA

#define PHASE_B_PORT		1
#define PHASE_B_PIN			8
#define PHASE_B_GPIODATA	LPC_GPIO1->DATA

#define PHASE_C_PORT		1
#define PHASE_C_PIN			9
#define PHASE_C_GPIODATA	LPC_GPIO1->DATA

#define CW				1
#define CCW				0

//Define sensorless states

#define	STOP 			0
#define STANDBY		 	1
#define	ALIGN			2
#define RAMP			3
#define	RUN				4


#define FREE_RUNNING_COUNTER	1
#define FREE_RUNNING_DELAY		80
#define FREE_RUN_DUTY			1000

void vBLDC_Init(void);
void vBLDC_(void);
void vBLDC_LoadMatch(void);
void vBLDC_GetCommStep(void);
void EnableMotor(uint8_t bEnable);

void set_config();
void set_default_config();
int write_config_to_flash();
int read_flash_config();

void vPID_RPM(void);
signed char get_device_SN(unsigned int * SN);

void Delay_ms(uint32_t delay_ms);
void Blink(uint8_t count, uint32_t interval) ;

void RxFrameParser(void);

void TxFrameBuilder(void);

void TxConfigFrameBuilder(void);

#endif
