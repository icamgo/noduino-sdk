/*	
 *	Copyright (c) 2017, Jan Stegenga, EVAbits BV, <jan@evabits.com>
 *	Adapted from Sensirion's SHT library - see statement below.
 *	Only tested on STS-30
 *
 *	Copyright (c) 2016, Sensirion AG <andreas.brauchli@sensirion.com>
 *	Copyright (c) 2015-2016, Johannes Winkelmann <jw@smts.ch>
 *	All rights reserved.
 *
 *	Redistribution and use in source and binary forms, with or without
 *	modification, are permitted provided that the following conditions are met:
 *			* Redistributions of source code must retain the above copyright
 *				notice, this list of conditions and the following disclaimer.
 *			* Redistributions in binary form must reproduce the above copyright
 *				notice, this list of conditions and the following disclaimer in the
 *				documentation and/or other materials provided with the distribution.
 *			* Neither the name of the <organization> nor the
 *				names of its contributors may be used to endorse or promote products
 *				derived from this software without specific prior written permission.
 *
 *	THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
 *	AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
 *	IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
 *	ARE DISCLAIMED. IN NO EVENT SHALL <COPYRIGHT HOLDER> BE LIABLE FOR ANY
 *	DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES
 *	(INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
 *	LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND
 *	ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT
 *	(INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF
 *	THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE. 
 *
*/

#ifndef __STSSENSOR_H__
#define __STSSENSOR_H__

#include <inttypes.h>

class STSSensorDriver;

class STSSensor {
 public:
	/*
	 * Enum of the supported Digital Sensirion STS Sensors.
	 * For analog sensors, see STS3xAnalogSensor.
	 * Using the special AUTO_DETECT sensor causes all i2c sensors to be
	 * probed. The first matching sensor will then be used.
	 */
	enum STSSensorType {
		/* Automatically detect the sensor type (only i2c sensors listed above) */
		AUTO_DETECT,
		// i2c Sensors:
		/* STS3x-DIS with ADDR (sensor pin 2) connected to VSS (default) */
		STS3X,
		/* STS3x-DIS with ADDR (sensor pin 2) connected to VDD */
		STS3X_ALT
	};

	/*
	 * Accuracy setting of measurement.
	 * Not all sensors support changing the sampling accuracy.
	 */
	enum STSAccuracy {
		/* Highest repeatability at the cost of slower measurement */
		STS_ACCURACY_HIGH,
		/* Balanced repeatability and speed of measurement */
		STS_ACCURACY_MEDIUM,
		/* Fastest measurement but lowest repeatability */
		STS_ACCURACY_LOW
	};

	/* Value reported by getTemperature() when the sensor is not initialized */
	static const float TEMPERATURE_INVALID;

	/*
	 * Auto-detectable sensor types.
	 * Note that the STSW1 and STSW2 share exactly the same driver as the STSC1
	 * and are thus not listed individually.
	 */
	static const STSSensorType AUTO_DETECT_SENSORS[];

	/*
	 * Instantiate a new STSSensor
	 * By default, the i2c bus is queried for known STS Sensors. To address
	 * a specific sensor, set the `sensorType'.
	 */
	 STSSensor(TwoWire i2cWire, STSSensorType sensorType = AUTO_DETECT):
		mI2cWire(i2cWire),
		mSensorType(sensorType),
		mSensor(NULL), mTemperature(STSSensor::TEMPERATURE_INVALID) {
	}
	
	virtual ~ STSSensor() {
		cleanup();
	}

	/*
	 * Initialize the sensor driver
	 * To read out the sensor use readSample(), followed by getTemperature() 
	 */
	bool init(TwoWire i2cWire);

	/*
	 * Read new values from the sensor
	 * After the call, use getTemperature() to retrieve the values
	 * Returns true if the sample was read and the values are cached
	 */
	bool readSample();

	/*
	 * Get the temperature in percent read from the last sample
	 * Use readSample() to trigger a new sensor reading
	 */
	float getTemperature() const {
		return mTemperature;
	}

	/*
	 * Change the sensor accurancy, if supported by the sensor
	 * Returns true if the accuracy was changed
	 */
	bool setAccuracy(STSAccuracy newAccuracy);

 private:
	void cleanup();
	STSSensorType mSensorType;
	STSSensorDriver *mSensor;
	float mTemperature;
	TwoWire mI2cWire;
};

/* Abstract class for a digital STS Sensor driver */
class STSSensorDriver {
 public:
	virtual ~ STSSensorDriver() = 0;

	/*
	 * Set the sensor accuracy.
	 * Returns false if the sensor does not support changing the accuracy
	 */
	virtual bool setAccuracy(STSSensor::STSAccuracy newAccuracy) {
		return false;
	}

	/* Returns true if the next sample was read and the values are cached */
	virtual bool readSample();

	/*
	 * Get the temperature in percent read from the last sample
	 * Use readSample() to trigger a new sensor reading
	 */
	float getTemperature() const {
		return mTemperature;
	}
	
	float mTemperature;
};

/* Base class for i2c STS Sensor drivers */
class STSI2cSensor:public STSSensorDriver {
 public:
	/* Size of i2c commands to send */
	static const uint8_t CMD_SIZE;

	/* Size of i2c replies to expect */
	static const uint8_t EXPECTED_DATA_SIZE;

	/*
	 * Constructor for i2c STS Sensors
	 * Takes the `i2cAddress' to read, the `i2cCommand' issues when sampling
	 * the sensor and the values `a', `b', `c' to convert the fixed-point
	 * temperature value received by the sensor to a floating point value using
	 * the formula: temperature = a + b * (rawTemperature / c)
	 */
	 STSI2cSensor(TwoWire i2cWire, uint8_t i2cAddress, uint16_t i2cCommand,
					float a, float b, float c)
				: mI2cWire(i2cWire), mI2cAddress(i2cAddress), mI2cCommand(i2cCommand),
					mA(a), mB(b), mC(c) {
	}
	
	virtual ~ STSI2cSensor() {

	}

	virtual bool readSample();

	uint8_t mI2cAddress;
	uint16_t mI2cCommand;
	float mA;
	float mB;
	float mC;
	TwoWire mI2cWire;

 private:
	static const uint8_t MAX_I2C_READ_TRIES;
	static uint8_t crc8(const uint8_t * data, uint8_t len);
	static bool readFromI2c(TwoWire localWire,
				uint8_t i2cAddress,
				const uint8_t * i2cCommand,
				uint8_t commandLength, uint8_t * data,
				uint8_t dataLength);
};

#endif
