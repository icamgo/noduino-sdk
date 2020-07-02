#ifndef __I2C_H__
#define	__I2C_H__
 
#include "Arduino.h"



extern "C" {

	void i2c_init();

	I2C_TransferReturn_TypeDef i2c_write(uint8_t addr, uint8_t *txbuf, int8_t txlen, uint8_t *rxbuf, uint8_t rxlen);
	I2C_TransferReturn_TypeDef i2c_read(uint8_t addr, uint8_t *rxData, uint8_t len);
}
 
#endif
