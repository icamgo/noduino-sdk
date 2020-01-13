/*  
 *  Copyright (c) 2017, Jan Stegenga, EVAbits BV, <jan@evabits.com>
 *  Adapted from Sensirion's SHT library - see statement below.
 *  Only tested on STS-30
 *
 *  Copyright (c) 2016, Sensirion AG <andreas.brauchli@sensirion.com>
 *  Copyright (c) 2015-2016, Johannes Winkelmann <jw@smts.ch>
 *  All rights reserved.
 *
 *  Redistribution and use in source and binary forms, with or without
 *  modification, are permitted provided that the following conditions are met:
 *      * Redistributions of source code must retain the above copyright
 *        notice, this list of conditions and the following disclaimer.
 *      * Redistributions in binary form must reproduce the above copyright
 *        notice, this list of conditions and the following disclaimer in the
 *        documentation and/or other materials provided with the distribution.
 *      * Neither the name of the <organization> nor the
 *        names of its contributors may be used to endorse or promote products
 *        derived from this software without specific prior written permission.
 *
 *  THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
 *  AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
 *  IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
 *  ARE DISCLAIMED. IN NO EVENT SHALL <COPYRIGHT HOLDER> BE LIABLE FOR ANY
 *  DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES
 *  (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
 *  LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND
 *  ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT
 *  (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF
 *  THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE. 
 *
*/

#include <inttypes.h>
#include <Arduino.h>
#include "softi2c.h"
#include "sts.h"

STSSensorDriver::~STSSensorDriver()
{

}

bool STSSensorDriver::readSample()
{
	return false;
}

const uint8_t STSI2cSensor::CMD_SIZE = 2;
const uint8_t STSI2cSensor::EXPECTED_DATA_SIZE = 3;	//two bytes + CRC
const uint8_t STSI2cSensor::MAX_I2C_READ_TRIES = 5;

bool STSI2cSensor::readFromI2c(uint8_t i2cAddress,
			       const uint8_t * i2cCommand,
			       uint8_t commandLength, uint8_t * data,
			       uint8_t dataLength)
{
	wire_beginTransmission(i2cAddress);

	for (int i = 0; i < commandLength; ++i) {
		wire_write(i2cCommand[i]);
	}

	wire_endTransmission();

	wire_requestFrom(i2cAddress, dataLength);

	// there should be no reason for this to not be ready, since we're using clock
	// stretching mode, but just in case we'll try a few times
	uint8_t tries = 1;
	while (wire_available() < dataLength) {
		delay(1);
		if (tries++ >= MAX_I2C_READ_TRIES) {
			return false;
		}
	}

	for (int i = 0; i < dataLength; ++i) {
		data[i] = wire_read();
	}
	return true;
}

uint8_t STSI2cSensor::crc8(const uint8_t * data, uint8_t len)
{
	// adapted from STS21 sample code from
	// http://www.sensirion.com/en/products/humidity-temperature/download-center/

	uint8_t crc = 0xff;
	uint8_t byteCtr;
	for (byteCtr = 0; byteCtr < len; ++byteCtr) {
		crc ^= data[byteCtr];
		for (uint8_t bit = 8; bit > 0; --bit) {
			if (crc & 0x80) {
				crc = (crc << 1) ^ 0x31;
			} else {
				crc = (crc << 1);
			}
		}
	}
	return crc;
}

bool STSI2cSensor::readSample()
{
	uint8_t data[EXPECTED_DATA_SIZE];
	uint8_t cmd[CMD_SIZE];

	cmd[0] = mI2cCommand >> 8;
	cmd[1] = mI2cCommand & 0xff;

	if (!readFromI2c(mI2cAddress, cmd, CMD_SIZE, data,
			 EXPECTED_DATA_SIZE)) {
		return false;
	}
	// -- Important: assuming each 2 byte of data is followed by 1 byte of CRC

	// check CRC for T
	if (crc8(&data[0], 2) != data[2]) {
		return false;
	}
	// convert to Temperature
	uint16_t val;
	val = (data[0] << 8) + data[1];
	mTemperature = mA + mB * (val / mC);

	return true;
}

class STS3xSensor:public STSI2cSensor {
 private:
	static const uint16_t STS3X_ACCURACY_HIGH = 0x2c06;
	static const uint16_t STS3X_ACCURACY_MEDIUM = 0x2c0d;
	static const uint16_t STS3X_ACCURACY_LOW = 0x2c10;

 public:
	static const uint8_t STS3X_I2C_ADDRESS_44 = 0x4A;
	static const uint8_t STS3X_I2C_ADDRESS_45 = 0x4B;

	STS3xSensor(uint8_t i2cAddress = STS3X_I2C_ADDRESS_44)
		: STSI2cSensor(i2cAddress, STS3X_ACCURACY_HIGH, -45, 175, 65535)	//100, 65535
	{

	}
	
	virtual bool setAccuracy(STSSensor::STSAccuracy newAccuracy) {
		switch (newAccuracy) {
		case STSSensor::STS_ACCURACY_HIGH:
			mI2cCommand = STS3X_ACCURACY_HIGH;
			break;
		case STSSensor::STS_ACCURACY_MEDIUM:
			mI2cCommand = STS3X_ACCURACY_MEDIUM;
			break;
		case STSSensor::STS_ACCURACY_LOW:
			mI2cCommand = STS3X_ACCURACY_LOW;
			break;
		default:
			return false;
		}
		return true;
	}
};

const STSSensor::STSSensorType STSSensor::AUTO_DETECT_SENSORS[] = {
	STS3X,
	STS3X_ALT
};

const float STSSensor::TEMPERATURE_INVALID = NAN;

bool STSSensor::init(int scl, int sda)
{
	if (mSensor != NULL) {
		cleanup();
	}

	wire_begin(scl, sda);

	switch (mSensorType) {
	case STS3X:
		mSensor = new STS3xSensor();
		break;

	case STS3X_ALT:
		mSensor =
		    new STS3xSensor(STS3xSensor::STS3X_I2C_ADDRESS_45);
		break;

	case AUTO_DETECT:
		{
			bool detected = false;
			for (unsigned int i = 0;
			     i <
			     sizeof(AUTO_DETECT_SENSORS) /
			     sizeof(AUTO_DETECT_SENSORS[0]); ++i) {
				mSensorType = AUTO_DETECT_SENSORS[i];

				if (init(scl, sda) && readSample()) {
					detected = true;
					break;
				}
			}
			if (!detected) {
				cleanup();
			}
			break;
		}
	}
	return (mSensor != NULL);
}

bool STSSensor::readSample()
{
	if (!mSensor || !mSensor->readSample())
		return false;
	mTemperature = mSensor->mTemperature;
	return true;
}

bool STSSensor::setAccuracy(STSAccuracy newAccuracy)
{
	if (!mSensor)
		return false;
	return mSensor->setAccuracy(newAccuracy);
}

void STSSensor::cleanup()
{
	if (mSensor) {
		delete mSensor;
		mSensor = NULL;
	}
}
