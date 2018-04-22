/*
	Bosch BMP085 pressure sensor library for the Arduino microcontroller
	Mike Grusin, SparkFun Electronics

	Uses floating-point equations from the Weather Station Data Logger project
	http://wmrx00.sourceforge.net/
	http://wmrx00.sourceforge.net/Arduino/BMP085-Calcs.pdf

	Forked from BMP085 library by M.Grusin

	version 1.0 2013/09/20 initial version
	Verison 1.1.2 - Updated for Arduino 1.6.4 5/2015
	
	Our example code uses the "beerware" license. You can do anything
	you like with this code. No really, anything. If you find it useful,
	buy me a (root) beer someday.
*/

#ifndef __BMP085_H__
#define __BMP085_H__

#include "noduino.h"

char bmp085_begin();
	// call pressure.begin() to initialize BMP085 before use
	// returns 1 if success, 0 if failure (bad component or I2C bus shorted?)

char bmp085_startTemperature(void);
	// command BMP085 to start a temperature measurement
	// returns (number of ms to wait) for success, 0 for fail

char bmp085_getTemperature(double *T);
	// return temperature measurement from previous startTemperature command
	// places returned value in T variable (deg C)
	// returns 1 for success, 0 for fail

char bmp085_startPressure(char oversampling);
	// command BMP085 to start a pressure measurement
	// oversampling: 0 - 3 for oversampling value
	// returns (number of ms to wait) for success, 0 for fail

char bmp085_getPressure(double *P, double *T);
	// return absolute pressure measurement from previous startPressure command
	// note: requires previous temperature measurement in variable T
	// places returned value in P variable (mbar)
	// returns 1 for success, 0 for fail

double bmp085_sealevel(double P, double A);
	// convert absolute pressure to sea-level pressure (as used in weather data)
	// P: absolute pressure (mbar)
	// A: current altitude (meters)
	// returns sealevel pressure in mbar

double bmp085_altitude(double P, double P0);
	// convert absolute pressure to altitude (given baseline pressure; sea-level, runway, etc.)
	// P: absolute pressure (mbar)
	// P0: fixed baseline pressure (mbar)
	// returns signed altitude in meters

char bmp085_getError(void);
	// If any library command fails, you can retrieve an extended
	// error code using this command. Errors are from the wire library: 
	// 0 = Success
	// 1 = Data too long to fit in transmit buffer
	// 2 = Received NACK on transmit of address
	// 3 = Received NACK on transmit of data
	// 4 = Other error

char bmp085_readInt(char address, int16_t * value);
	// read an signed int (16 bits) from a BMP085 register
	// address: BMP085 register address
	// value: external signed int for returned value (16 bits)
	// returns 1 for success, 0 for fail, with result in value

char bmp085_readUInt(char address, uint16_t * value);
	// read an unsigned int (16 bits) from a BMP085 register
	// address: BMP085 register address
	// value: external unsigned int for returned value (16 bits)
	// returns 1 for success, 0 for fail, with result in value

char bmp085_readBytes(unsigned char *values, char length);
	// read a number of bytes from a BMP085 register
	// values: array of char with register address in first location [0]
	// length: number of bytes to read back
	// returns 1 for success, 0 for fail, with read bytes in values[] array

char bmp085_writeBytes(unsigned char *values, char length);
	// write a number of bytes to a BMP085 register (and consecutive subsequent registers)
	// values: array of char with register address in first location [0]
	// length: number of bytes to write
	// returns 1 for success, 0 for fail

#define BMP085_ADDR			0x77	// 7-bit address
#define	BMP085_REG_CONTROL	0xF4
#define	BMP085_REG_RESULT	0xF6
#define	BMP085_COMMAND_TEMPERATURE 0x2E
#define	BMP085_COMMAND_PRESSURE0 0x34
#define	BMP085_COMMAND_PRESSURE1 0x74
#define	BMP085_COMMAND_PRESSURE2 0xB4
#define	BMP085_COMMAND_PRESSURE3 0xF4

#endif
