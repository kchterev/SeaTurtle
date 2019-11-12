/***********************************************************************
 * $Id:: application.h 3871 2010-07-16 11:50:22Z gerritdewaard         $
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

#ifndef __APPLICATION_H 
#define __APPLICATION_H

#include <stdio.h>
#include <string.h>
#include "stdlib.h"
#include "type.h"
/* Chip level includes */
#include <LPC11xx.h>
#include <System_LPC11xx.h>
#include "driver_config.h"
#include "target_config.h"
#include "core_cm0.h"
#include "gpio.h"
#include "timer16.h"
#include "timer32.h"
#include "uart.h"
#include "adc.h"
#include "i2c.h"

/* Application level includes */
#include "IAP.h"
#include "BLDC.h"


/****************************************************
	SOFTWARE VERSION STRING (max 16 chars/row, max 1 rows)
*****************************************************/
#define SW_VERSION_STRING "BLDC Controller.\n20131216 rev A"

/****************************************************  
	SYSTEM PINOUT 
*****************************************************/

/****************************************************/
/* System setup										*/
/* Please see also driver_config.h for driver inits */
/****************************************************/


/****************************************************/
/* Systick definitions								*/
/****************************************************/
#define SysTick_VALUE		SystemCoreClock / 1000

/****************************************************/
/* Generic definitions 								*/
/****************************************************/

#ifndef NULL
#define NULL    ((void *)0)
#endif

#ifndef FALSE
#define FALSE   (0)
#endif

#ifndef TRUE
#define TRUE    (1)
#endif

/****************************************************/
/* TIMER related macros								*/
/****************************************************/
#define EMC0	4
#define EMC1	6
#define EMC2	8
#define EMC3	10

#define TIMER1	1
#define TIMER0	0

#define CAP_ENABLE 1
#define CAP_DISABLE 0
#define CAP_RISING 1
#define CAP_FALLING 2
#define CAP_EVENT 4

#define MATCH0	(1<<0)
#define MATCH1	(1<<1)
#define MATCH2	(1<<2)
#define MATCH3	(1<<3)

#define TEST_TIMER_NUM		1		/* 0 or 1 for 16-bit timers only */

#endif /* end __APPLICATION_H */


/*****************************************************************************
**                            End Of File
******************************************************************************/
