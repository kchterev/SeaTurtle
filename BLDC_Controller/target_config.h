/****************************************************************************
 *   $Id:: target_config.h 4832 2010-09-07 23:04:21Z nxp21346                 $
 *   Project: LED flashing / ISP test program
 *
 *   Description:
 *     This file contains the config settings for the blinky code sample
 *     which flashes an LED on the LPCXpresso board and also increments an
 *     LED display on the Embedded Artists base board. This project
 *     implements CRP and is useful for testing bootloaders.
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
****************************************************************************/

#define LED_PORT 0		// Port for led
#define LED_BIT 7		// Bit on port for led
#define DEBUG_BIT 9		// pin 9 on port 2 for debug
#define DEBUG_PORT 2		// pin 9 on port 2 for debug
#define PORT3	3				// Current Trip Port
#define PORT1   1				// Current Select Input Port
//#define CURRENT_TRIP_BIT	5	// Current Trip control line bit
#define PORT2   2				// CPLD enable port
#define CPLD_ENABLE_BIT	6		// CPLD enable bit
#define LIN_ENABLE_BIT 2		// enables MC2003
//#define CURRENT_SELECT_BIT  8   //selects the current input to AD1 on NXP evaluation board
#define LED_ON 1		// Level to set port to turn on led
#define LED_OFF 0		// Level to set port to turn off led
#define LED_TOGGLE_TICKS 200 // 100 ticks = 1 Hz flash rate
#define FAST_LED_TOGGLE_TICKS 50 // 100 ticks = 1 Hz flash rate
#define COUNT_MAX		3 // how high to count on the LED display

/*********************************************************************************
**                            End Of File
*********************************************************************************/
