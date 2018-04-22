/*
	SFE_BMP085.cpp
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

#include "bmp085.h"

int16_t AC1, AC2, AC3, VB1, VB2, MB, MC, MD;
uint16_t AC4, AC5, AC6;
double c5, c6, mc, md, x0, x1, x2, Y0, Y1, y2, p0, p1, p2;
char bmp085_error;

irom char bmp085_begin()
// Initialize library for subsequent pressure measurements
{
	double c3, c4, b1;

	// Start up the Arduino's "wire" (I2C) library:

	wire_begin();

	// The BMP085 includes factory calibration data stored on the device.
	// Each device has different numbers, these must be retrieved and
	// used in the calculations when taking pressure measurements.

	// Retrieve calibration data from device:

	if (bmp085_readInt(0xAA, &AC1) &&
	    bmp085_readInt(0xAC, &AC2) &&
	    bmp085_readInt(0xAE, &AC3) &&
	    bmp085_readUInt(0xB0, &AC4) &&
	    bmp085_readUInt(0xB2, &AC5) &&
	    bmp085_readUInt(0xB4, &AC6) &&
	    bmp085_readInt(0xB6, &VB1) &&
	    bmp085_readInt(0xB8, &VB2) &&
	    bmp085_readInt(0xBA, &MB) &&
	    bmp085_readInt(0xBC, &MC) && bmp085_readInt(0xBE, &MD)) {

		// All reads completed successfully!

		// If you need to check your math using known numbers,
		// you can uncomment one of these examples.
		// (The correct results are commented in the below functions.)

		// Example from Bosch datasheet
		// AC1 = 408; AC2 = -72; AC3 = -14383; AC4 = 32741; AC5 = 32757; AC6 = 23153;
		// B1 = 6190; B2 = 4; MB = -32768; MC = -8711; MD = 2868;

		// Example from http://wmrx00.sourceforge.net/Arduino/BMP085-Calcs.pdf
		// AC1 = 7911; AC2 = -934; AC3 = -14306; AC4 = 31567; AC5 = 25671; AC6 = 18974;
		// VB1 = 5498; VB2 = 46; MB = -32768; MC = -11075; MD = 2432;

		/*
		   serial_printf("AC1: "); serial_printf("%d\r\n", AC1);
		   serial_printf("AC2: "); serial_printf("%d\r\n", AC2);
		   serial_printf("AC3: "); serial_printf("%d\r\n", AC3);
		   serial_printf("AC4: "); serial_printf("%d\r\n", AC4);
		   serial_printf("AC5: "); serial_printf("%d\r\n", AC5);
		   serial_printf("AC6: "); serial_printf("%d\r\n", AC6);
		   serial_printf("VB1: "); serial_printf("%d\r\n", VB1);
		   serial_printf("VB2: "); serial_printf("%d\r\n", VB2);
		   serial_printf("MB: "); serial_printf("%d\r\n", MB);
		   serial_printf("MC: "); serial_printf("%d\r\n", MC);
		   serial_printf("MD: "); serial_printf("%d\r\n", MD);
		 */

		// Compute floating-point polynominals:

		c3 = 160.0 * pow(2, -15) * AC3;
		c4 = pow(10, -3) * pow(2, -15) * AC4;
		b1 = pow(160, 2) * pow(2, -30) * VB1;
		c5 = (pow(2, -15) / 160) * AC5;
		c6 = AC6;
		mc = (pow(2, 11) / pow(160, 2)) * MC;
		md = MD / 160.0;
		x0 = AC1;
		x1 = 160.0 * pow(2, -13) * AC2;
		x2 = pow(160, 2) * pow(2, -25) * VB2;
		Y0 = c4 * pow(2, 15);
		Y1 = c4 * c3;
		y2 = c4 * b1;
		p0 = (3791.0 - 8.0) / 1600.0;
		p1 = 1.0 - 7357.0 * pow(2, -20);
		p2 = 3038.0 * 100.0 * pow(2, -36);

		/*
		   serial_printf("\r\n", );
		   serial_printf("c3: "); serial_printf("%d\r\n", c3);
		   serial_printf("c4: "); serial_printf("%d\r\n", c4);
		   serial_printf("c5: "); serial_printf("%d\r\n", c5);
		   serial_printf("c6: "); serial_printf("%d\r\n", c6);
		   serial_printf("b1: "); serial_printf("%d\r\n", b1);
		   serial_printf("mc: "); serial_printf("%d\r\n", mc);
		   serial_printf("md: "); serial_printf("%d\r\n", md);
		   serial_printf("x0: "); serial_printf("%d\r\n", x0);
		   serial_printf("x1: "); serial_printf("%d\r\n", x1);
		   serial_printf("x2: "); serial_printf("%d\r\n", x2);
		   serial_printf("Y0: "); serial_printf("%d\r\n", Y0);
		   serial_printf("Y1: "); serial_printf("%d\r\n", Y1);
		   serial_printf("y2: "); serial_printf("%d\r\n", y2);
		   serial_printf("p0: "); serial_printf("%d\r\n", p0);
		   serial_printf("p1: "); serial_printf("%d\r\n", p1);
		   serial_printf("p2: "); serial_printf("%d\r\n", p2);
		 */

		// Success!
		return (1);
	} else {
		// Error reading calibration data; bad component or connection?
		return (0);
	}
}

irom char bmp085_readInt(char address, int16_t * value)
// Read a signed integer (two bytes) from device
// address: register to start reading (plus subsequent register)
// value: external variable to store data (function modifies value)
{
	unsigned char data[2];

	data[0] = address;
	if (bmp085_readBytes(data, 2)) {
		*value = (int16_t) ((data[0] << 8) | data[1]);
		//if (*value & 0x8000) *value |= 0xFFFF0000; // sign extend if negative
		return (1);
	}
	value = 0;
	return (0);
}

irom char bmp085_readUInt(char address, uint16_t * value)
// Read an unsigned integer (two bytes) from device
// address: register to start reading (plus subsequent register)
// value: external variable to store data (function modifies value)
{
	unsigned char data[2];

	data[0] = address;
	if (bmp085_readBytes(data, 2)) {
		*value = (((uint16_t) data[0] << 8) | (uint16_t) data[1]);
		return (1);
	}
	value = 0;
	return (0);
}

irom char bmp085_readBytes(unsigned char *values, char length)
// Read an array of bytes from device
// values: external array to hold data. Put starting register in values[0].
// length: number of bytes to read
{
	char x;

	wire_beginTransmission(BMP085_ADDR);
	wire_write(values[0]);
	bmp085_error = wire_endTransmission();
	if (bmp085_error == 0) {
		wire_requestFrom(BMP085_ADDR, length);
		while (wire_available() != length) ;	// wait until bytes are ready
		for (x = 0; x < length; x++) {
			values[x] = wire_read();
		}
		return (1);
	}
	return (0);
}

irom char bmp085_writeBytes(unsigned char *values, char length)
// Write an array of bytes to device
// values: external array of data to write. Put starting register in values[0].
// length: number of bytes to write
{
	char x;
	int i;

	wire_beginTransmission(BMP085_ADDR);
	for (i = 0; i < length; i++) 
		wire_write(values[i]);
	bmp085_error = wire_endTransmission();
	if (bmp085_error == 0)
		return (1);
	else
		return (0);
}

irom char bmp085_startTemperature(void)
// Begin a temperature reading.
// Will return delay in ms to wait, or 0 if I2C error
{
	unsigned char data[2], result;

	data[0] = BMP085_REG_CONTROL;
	data[1] = BMP085_COMMAND_TEMPERATURE;
	result = bmp085_writeBytes(data, 2);
	if (result)		// good write?
		return (5);	// return the delay in ms (rounded up) to wait before retrieving data
	else
		return (0);	// or return 0 if there was a problem communicating with the BMP
}

irom char bmp085_getTemperature(double *T)
// Retrieve a previously-started temperature reading.
// Requires begin() to be called once prior to retrieve calibration parameters.
// Requires startTemperature() to have been called prior and sufficient time elapsed.
// T: external variable to hold result.
// Returns 1 if successful, 0 if I2C error.
{
	unsigned char data[2];
	char result;
	double tu, a;

	data[0] = BMP085_REG_RESULT;

	result = bmp085_readBytes(data, 2);
	if (result)		// good read, calculate temperature
	{
		tu = (data[0] * 256.0) + data[1];

		//example from Bosch datasheet
		//tu = 27898;

		//example from http://wmrx00.sourceforge.net/Arduino/BMP085-Calcs.pdf
		//tu = 0x69EC;

		a = c5 * (tu - c6);
		*T = a + (mc / (a + md));

		/*              
		   serial_printf("\r\n", );
		   serial_printf("tu: "); serial_printf("%d\r\n", tu);
		   serial_printf("a: "); serial_printf("%d\r\n", a);
		   serial_printf("T: "); serial_printf("%d\r\n", *T);
		 */
	}
	return (result);
}

// Begin a pressure reading.
// Oversampling: 0 to 3, higher numbers are slower, higher-res outputs.
// Will return delay in ms to wait, or 0 if I2C error.
irom char bmp085_startPressure(char oversampling)
{
	unsigned char data[2], result, delay;

	data[0] = BMP085_REG_CONTROL;

	switch (oversampling) {
	case 0:
		data[1] = BMP085_COMMAND_PRESSURE0;
		delay = 5;
		break;
	case 1:
		data[1] = BMP085_COMMAND_PRESSURE1;
		delay = 8;
		break;
	case 2:
		data[1] = BMP085_COMMAND_PRESSURE2;
		delay = 14;
		break;
	case 3:
		data[1] = BMP085_COMMAND_PRESSURE3;
		delay = 26;
		break;
	default:
		data[1] = BMP085_COMMAND_PRESSURE0;
		delay = 5;
		break;
	}
	result = bmp085_writeBytes(data, 2);
	if (result)		// good write?
		return (delay);	// return the delay in ms (rounded up) to wait before retrieving data
	else
		return (0);	// or return 0 if there was a problem communicating with the BMP
}

// Retrieve a previously started pressure reading, calculate abolute pressure in mbars.
// Requires begin() to be called once prior to retrieve calibration parameters.
// Requires startPressure() to have been called prior and sufficient time elapsed.
// Requires recent temperature reading to accurately calculate pressure.
// P: external variable to hold pressure.
// T: previously-calculated temperature.
// Returns 1 for success, 0 for I2C error.
// Note that calculated pressure value is absolute mbars, to compensate for altitude call sealevel().
irom char bmp085_getPressure(double *P, double *T)
{
	unsigned char data[3];
	char result;
	double pu, s, x, y, z;

	data[0] = BMP085_REG_RESULT;

	result = bmp085_readBytes(data, 3);
	if (result)		// good read, calculate pressure
	{
		pu = (data[0] * 256.0) + data[1] + (data[2] / 256.0);

		//example from Bosch datasheet
		//pu = 23843;

		//example from http://wmrx00.sourceforge.net/Arduino/BMP085-Calcs.pdf, pu = 0x982FC0;   
		//pu = (0x98 * 256.0) + 0x2F + (0xC0/256.0);

		s = *T - 25.0;
		x = (x2 * pow(s, 2)) + (x1 * s) + x0;
		y = (y2 * pow(s, 2)) + (Y1 * s) + Y0;
		z = (pu - x) / y;
		*P = (p2 * pow(z, 2)) + (p1 * z) + p0;

		/*
		   serial_printf("\r\n");
		   serial_printf("pu: "); serial_printf("%d\r\n", pu);
		   serial_printf("T: "); serial_printf("%d\r\n", *T);
		   serial_printf("s: "); serial_printf("%d\r\n", s);
		   serial_printf("x: "); serial_printf("%d\r\n", x);
		   serial_printf("y: "); serial_printf("%d\r\n", y);
		   serial_printf("z: "); serial_printf("%d\r\n", z);
		   serial_printf("P: "); serial_printf("%d\r\n", *P);
		 */
	}
	return (result);
}

// Given a pressure P (mb) taken at a specific altitude (meters),
// return the equivalent pressure (mb) at sea level.
// This produces pressure readings that can be used for weather measurements.
irom double bmp085_sealevel(double P, double A)
{
	return (P / pow(1 - (A / 44330.0), 5.255));
}

// Given a pressure measurement P (mb) and the pressure at a baseline P0 (mb),
// return altitude (meters) above baseline.
irom double bmp085_altitude(double P, double P0)
{
	return (44330.0 * (1 - pow(P / P0, 1 / 5.255)));
}

// If any library command fails, you can retrieve an extended
// error code using this command. Errors are from the wire library: 
// 0 = Success
// 1 = Data too long to fit in transmit buffer
// 2 = Received NACK on transmit of address
// 3 = Received NACK on transmit of data
// 4 = Other error
irom char bmp085_getError(void)
{
	return (bmp085_error);
}
