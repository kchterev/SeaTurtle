/******************************************************************************
SFE_LSM9DS1.cpp
SFE_LSM9DS1 Library Source File
Jim Lindblom @ SparkFun Electronics
Original Creation Date: February 27, 2015
https://github.com/sparkfun/LSM9DS1_Breakout

This file implements all functions of the LSM9DS1 class. Functions here range
from higher level stuff, like reading/writing LSM9DS1 registers to low-level,
hardware reads and writes. Both SPI and I2C handler functions can be found
towards the bottom of this file.

Development environment specifics:
    IDE: Arduino 1.6
    Hardware Platform: Arduino Uno
    LSM9DS1 Breakout Version: 1.0

This code is beerware; if you see me (or any other SparkFun employee) at the
local, and you've found our code helpful, please buy us a round!

Distributed as-is; no warranty is given.
******************************************************************************/

#include "CMPS11.h"
#include "i2c.h"
#include "BLDC.h"

extern uint8_t * ptrCMPS; 	//pointer to the compass data structure
extern CMPS11_REPORT CMPS;

uint32_t CMPS11_read(uint8_t * data)
{
	uint8_t Tx_Address = CMPS11_ADDRESS;
	uint8_t Tx_SubAddress=0;
	uint8_t Tx_len=0;
	uint8_t Rx_len = sizeof(CMPS);

	return i2c_write_read(Tx_Address,Tx_SubAddress,Tx_len,ptrCMPS, Rx_len);
}
