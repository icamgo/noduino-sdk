#ifndef __SPICLASS_H__
#define __SPICLASS_H__

#include "spidrv.h"

#ifdef __cplusplus   /*add arduino style class lib  by huaweiwx@sinq.com 2018.9.*/
class SPISettings {
  public:
    SPISettings(uint32_t clock, uint8_t bitOrder, uint8_t dataMode): clock(clock), bitOrder(bitOrder), dataMode(dataMode) {};
    SPISettings(): clock(0), bitOrder(0), dataMode(0) {};
    //init(4000000, MSBFIRST, SPI_MODE0);

    uint32_t clock;
    uint8_t bitOrder;
    uint8_t dataMode;
};


class SPIClass {
public:
  // Initialize the SPI library
  Ecode_t  begin( SPIDRV_Handle_t handle, SPIDRV_Init_t *initData ){
       return SPIDRV_Init(handle,initData);
  };
  Ecode_t end( SPIDRV_Handle_t handle ){
     return SPIDRV_DeInit(handle);
  };

  void beginTransaction(SPISettings settings) {
  }

  // Write to the SPI bus (MOSI pin) and also receive (MISO pin)
  inline static uint8_t transfer(uint8_t data) {
  }
  inline static uint16_t transfer16(uint16_t data) {
  }
  inline static void transfer(void *buf, size_t count) {
  }
  // After performing a group of transfers and releasing the chip select
  // signal, this function allows others to access the SPI bus
  inline static void endTransaction(void) {
  }
  inline static void setBitOrder(uint8_t bitOrder) {
  }
  // This function is deprecated.  New applications should use
  // beginTransaction() to configure SPI settings.
  inline static void setDataMode(uint8_t dataMode) {
  }
  // This function is deprecated.  New applications should use
  // beginTransaction() to configure SPI settings.
  inline static void setClockDivider(uint8_t clockDiv) {
  }
  // These undocumented functions should not be used.  SPI.transfer()
  // polls the hardware flag which is automatically cleared as the
  // AVR responds to SPI's interrupt
  inline static void attachInterrupt() {}
  inline static void detachInterrupt() {}

private:

};

#include "SoftSPI.h"

extern SPIClass SPI;

#endif /*__cplusplus*/

#endif /* __SILICON_LABS_SPIDRV_H__ */
