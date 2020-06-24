#ifndef __I2C_H__
#define	__I2C_H__
 
#include "Arduino.h"



extern "C" {

	void i2c_init(uint8_t addr);

	I2C_TransferReturn_TypeDef i2c_write(uint8_t addr, uint8_t* txData, uint8_t len);
	I2C_TransferReturn_TypeDef i2c_read(uint8_t addr, uint8_t *rxData, uint8_t len);
}
 
#endif
