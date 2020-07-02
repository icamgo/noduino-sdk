
#include "Arduino.h"

//#define	DEBUG				1

#ifdef DEBUG
#define	INFO(x)				Serial.println(x)
#else
#define	INFO(x)
#endif

#define TIME_OUTT 100

static uint8_t I2C_ADDRESS = 0x78;

/*
 * BMI160: I2C_ADDR = 0x68 / 0x69, don't need to set i2c_addr = 0xd0 (0x68 << 1)
 *
*/

void i2c_init(uint8_t addr)
{
	I2C_Init_TypeDef init = I2C_INIT_DEFAULT;

	init.enable = true;
	init.master = true;
	init.refFreq = 0;
#if 0
	init.clhr = i2cClockHLRStandard;
	init.freq = I2C_FREQ_STANDARD_MAX;
#else
	init.clhr = i2cClockHLRFast;
	init.freq = I2C_FREQ_FASTPLUS_MAX;
#endif

	I2C_ADDRESS = addr;

	/* Enabling clock to the I2C, GPIO*/
	CMU_ClockEnable(cmuClock_I2C0, true);

	/* PE12 - SDA, PE13 - SCL */
#if 1
	GPIO_PinModeSet(gpioPortE, GPIO_PIN_12, gpioModeWiredAndPullUpFilter, 1);
	GPIO_PinModeSet(gpioPortE, GPIO_PIN_13, gpioModeWiredAndPullUpFilter, 1);
#else
	GPIO_PinModeSet(gpioPortE, GPIO_PIN_12, gpioModePushPull, 1);
	GPIO_PinModeSet(gpioPortE, GPIO_PIN_13, gpioModePushPull, 1);
#endif

	I2C0->ROUTE = I2C_ROUTE_SDAPEN | I2C_ROUTE_SCLPEN;
	I2C0->ROUTE = (I2C0->ROUTE & (~_I2C_ROUTE_LOCATION_MASK)) | I2C_ROUTE_LOCATION_LOC6;

	I2C_Init(I2C0, &init);

	//I2C0->CTRL |= I2C_CTRL_AUTOSN;
	//I2C0->CTRL |= I2C_CTRL_AUTOACK | I2C_CTRL_AUTOSN;
}

/*
 * I2C_FLAG_READ - data read into buf[0].data
 * I2C_FLAG_WRITE - data written from buf[0].data
 * I2C_FLAG_WRITE_READ - data written from buf[0].data and read into buf[1].data
 * I2C_FLAG_WRITE_WRITE - data written from buf[0].data and buf[1].data
 *
 */
I2C_TransferReturn_TypeDef i2c_write(uint8_t d1, uint8_t d2)
{
	uint32_t loop = TIME_OUTT;

	I2C_TransferReturn_TypeDef st;

	I2C_TransferSeq_TypeDef seq;

	seq.addr = I2C_ADDRESS;
	seq.flags = I2C_FLAG_WRITE_WRITE;

	seq.buf[0].data = &d1;
	seq.buf[0].len = 1;
	seq.buf[1].data = &d2;
	seq.buf[1].len = 1;

	st = I2C_TransferInit(I2C0, &seq);

	while ((st == i2cTransferInProgress) && loop > 0)
	{
		st = I2C_Transfer(I2C0);
		loop--;
	}	

	if (loop <= 0) {
		//Serial.println("i2c_wirte loop faild...");
		//st = -5;
	}

	return st;
}
