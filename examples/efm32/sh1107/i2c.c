
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
 * BMI160: I2C_ADDR = 0x68 / 0x69, set i2c_addr = 0xd0 (0x68 << 1)
 *
*/

void i2c_init(uint8_t addr)
{
	I2C_Init_TypeDef init_I2C = 
	{
		.enable = true,
		.clhr = i2cClockHLRStandard,
		.freq = I2C_FREQ_STANDARD_MAX,
		.master = true,
		.refFreq = 0
	};

	I2C_ADDRESS = addr;
	
	/* Enabling clock to the I2C, GPIO*/
	CMU_ClockEnable(cmuClock_HFPER, true);
	CMU_ClockEnable(cmuClock_I2C0, true);
	CMU_ClockEnable(cmuClock_GPIO, true);
	
	/* Starting LFXO and waiting until it is stable */
	CMU_OscillatorEnable(cmuOsc_LFRCO, true, true);

	/* Routing the LFXO clock to the RTC */
	CMU_ClockSelectSet(cmuClock_LFA, cmuSelect_LFRCO);
	
#if 0
	//GPIO_DriveModeSet(gpioPortA, gpioDriveModeHigh);
	
	/* Using PA0 (SDA) and PA1 (SCL) */
	GPIO_PinModeSet(gpioPortA, 0, gpioModeWiredAndPullUpFilter, 1);
	GPIO_PinModeSet(gpioPortA, 1, gpioModeWiredAndPullUpFilter, 1);
#endif

	/* PE12 - SDA, PE13 - SCL */
	GPIO_PinModeSet(gpioPortE, GPIO_PIN_12, gpioModeWiredAndPullUpFilter, 1);
	GPIO_PinModeSet(gpioPortE, GPIO_PIN_13, gpioModeWiredAndPullUpFilter, 1);

	/* Configure interrupt pin*/
	//GPIO_PinModeSet(gpioPortC, 4, gpioModeInput, 0);
	
	//I2C0->ROUTE |= I2C_ROUTE_LOCATION_LOC2 + I2C_ROUTE_SCLPEN + I2C_ROUTE_SDAPEN;
	I2C0->ROUTE = I2C_ROUTE_SDAPEN
	| I2C_ROUTE_SCLPEN
	| (0 << _I2C_ROUTE_LOCATION_SHIFT);

	I2C_Init(I2C0, &init_I2C);

	I2C0->CTRL |= I2C_CTRL_AUTOSN;						    //一个字节的读取，这里需要更改
	//I2C0->CTRL |= I2C_CTRL_AUTOACK | I2C_CTRL_AUTOSN;		//两个字节的读取，这里需要更改

	//NVIC_EnableIRQ(I2C0_IRQn);
}

I2C_TransferReturn_TypeDef i2c_write(uint8_t waddr, uint8_t* txData, uint8_t writeLen)
{
	uint32_t loop = TIME_OUTT;

	I2C_TransferReturn_TypeDef st;

	I2C_TransferSeq_TypeDef tx_Init = 
	{
		.addr = I2C_ADDRESS,
		.buf[0].data = &waddr,
		.buf[0].len = 1,
		.buf[1].data = txData,
		.buf[1].len = writeLen,
		.flags = I2C_FLAG_WRITE_WRITE,
	};
	
	st = I2C_TransferInit(I2C0, &tx_Init);

	while ((st == i2cTransferInProgress) && loop--)
	{
		st = I2C_Transfer(I2C0);
	}	

	if (loop == TIME_OUTT) {
		//Serial.println("i2c_wirte loop faild...");
		st = -5;
	}

	return st;
}
 
I2C_TransferReturn_TypeDef i2c_read(uint8_t raddr, uint8_t *rxData, uint8_t readLen)
{
	uint32_t loop = TIME_OUTT;

	I2C_TransferReturn_TypeDef st;

	I2C_TransferSeq_TypeDef rx_Init = 
	{
		.addr = I2C_ADDRESS,
		.buf[0].data = &raddr,
		.buf[0].len = 1,
		.buf[1].data = rxData,
		.buf[1].len = readLen,
		.flags = I2C_FLAG_WRITE_READ,
	};

	/* Do a polled transfer */
	st = I2C_TransferInit(I2C0, &rx_Init);

	while ((st == i2cTransferInProgress) && (loop--))
	{
		/* Enter EM1 while waiting for I2C interrupt */
		st = I2C_Transfer(I2C0);
		//EMU_EnterEM1();
		/* Could do a timeout function here. */
	}

	if (loop == TIME_OUTT) {
		//Serial.println("i2c_read loop fail...");
		st = -5;
	}

	return(st);
}
 
//void I2C0_IRQHandler(void)
//{
//	I2C_Status = I2C_Transfer(I2C0);
//}
