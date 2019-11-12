
#include "type.h"

#define CMPS11_ADDRESS 0xC0  	// Address of CMPS11 shifted right one bit for arduino wire library
#define FW_VERSION	0x03		// FW Version

uint32_t  CMPS11_read(uint8_t * data);

typedef struct CMPS11_STRUCT {
	uint8_t		fw_version;		//0- FW version
	uint8_t		cmps_byte;		//1- Compass Bearing 8 bit,
								//i.e. 0-255 for a full circle
	uint16_t	cmps_word;		//2,3- Compass Bearing 16 bit, i.e. 0-3599,
								//representing 0-359.9 degrees. register 2 being the high byte
	int8_t		pitch;			//4- Pitch angle - signed byte giving angle in degrees from the horizontal plane,
								//Kalman filtered with Gyro
	int8_t		roll;			//5- Roll angle - signed byte giving angle in degrees from the horizontal plane,
								//Kalman filtered with Gyro
	int16_t		magX;			//6,7- Magnetometer X axis raw output,
								//16 bit signed integer with register 6 being the upper 8 bits
	int16_t		magY;			//8,9- 	Magnetometer Y axis raw output,
								//16 bit signed integer with register 8 being the upper 8 bits
	int16_t		magZ;			//10,11- Magnetometer Z axis raw output,
								//16 bit signed integer with register 10 being the upper 8 bits
	int16_t		accX;			//12,13- Accelerometer  X axis raw output,
								//16 bit signed integer with register 12 being the upper 8 bits
	int16_t		accY;			//14,15-Accelerometer  Y axis raw output,
								//16 bit signed integer with register 14 being the upper 8 bits
	int16_t		accZ;			//16,17- Accelerometer  Z axis raw output,
								//16 bit signed integer with register 16 being the upper 8 bits
	int16_t		gyrX;			//18,19- Gyro X axis raw output,
								//16 bit signed integer with register 18 being the upper 8 bits
	int16_t		gyrY;			//20,21- Gyro  Y axis raw output, 16 bit signed integer with register 20 being the upper 8 bits
	int16_t		gyrZ;			//22,23- Gyro Z axis raw output, 16 bit signed integer with register 22 being the upper 8 bits
	int16_t		temp;			//24,25- Temperature raw output, 16 bit signed integer with register 24 being the upper 8 bits
	int8_t		pitch_nof;		//26- Pitch angle - signed byte giving angle in degrees from the horizontal plane (no Kalman filter)
	int8_t		roll_nof;		//27- Roll angle - signed byte giving angle in degrees from the horizontal plane (no Kalman filter)

} CMPS11_REPORT;
