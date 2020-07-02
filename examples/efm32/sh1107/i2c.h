#ifndef __I2C_H__
#define	__I2C_H__
 
#include "Arduino.h"

//extern "C" {

	void i2c_init(uint8_t addr);

	I2C_TransferReturn_TypeDef i2c_write(uint8_t d1, uint8_t d2);
	I2C_TransferReturn_TypeDef i2c_writeto(uint8_t *buf, uint8_t len);
//}
 
#endif
