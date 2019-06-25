#include "Wire.h"
#include "MCP342x.h"

#include "Arduino.h"

// Assuming a 100kHz clock the address and config byte take 18 clock
// cycles, or 180 microseconds. Use a timeout of 250us to be safe.

const MCP342x::Channel MCP342x::channel1 = Channel(0x00);
const MCP342x::Channel MCP342x::channel2 = Channel(0x20);
const MCP342x::Channel MCP342x::channel3 = Channel(0x40);
const MCP342x::Channel MCP342x::channel4 = Channel(0x60);

const MCP342x::Mode MCP342x::oneShot = Mode(0x00);
const MCP342x::Mode MCP342x::continous = Mode(0x10);

const MCP342x::Resolution MCP342x::resolution12 = Resolution(0x00);
const MCP342x::Resolution MCP342x::resolution14 = Resolution(0x04);
const MCP342x::Resolution MCP342x::resolution16 = Resolution(0x08);
const MCP342x::Resolution MCP342x::resolution18 = Resolution(0x0c);

const MCP342x::Gain MCP342x::gain1 = Gain(0x00);
const MCP342x::Gain MCP342x::gain2 = Gain(0x01);
const MCP342x::Gain MCP342x::gain4 = Gain(0x02);
const MCP342x::Gain MCP342x::gain8 = Gain(0x03);

uint8_t MCP342x::generalCallReset(void)
{
  Wire.beginTransmission(0x00);
  Wire.write(0x06);
  return Wire.endTransmission();
}

uint8_t MCP342x::generalCallLatch(void)
{
  Wire.beginTransmission(0x00);
  Wire.write(0x04);
  return Wire.endTransmission();
}

uint8_t MCP342x::generalCallConversion(void)
{
  Wire.beginTransmission(0x00);
  Wire.write(0x08);
  return Wire.endTransmission();
}

void MCP342x::normalise(long &result, Config config)
{
  /* Resolution is 12, 14, 16,or 18; gain is 1, 2, 4, or 8. Shift
   * least places necessary such that all possibilities can be
   * accounted for:
   *
   * 18 - resolution + 3 - log2(gain)
   *
   * Largest shift is for resolution==12 and gain==1 (9 places)
   * Smallest is for resolution==18 and gain==8 (0 places) This means
   * that the lowest 21 bits of the long result are used and that up
   * to 1024 results can be safely accumulated without
   * underflow/overflow.
   */ 
  result <<= (21 - int(config.getResolution()) - config.getGain().log2());
}


MCP342x::MCP342x(void) : address(0x68)
{
  ;
}
				
MCP342x::MCP342x(uint8_t add) : address(add)
{
  ;
}

bool MCP342x::autoprobe(const uint8_t *addressList, uint8_t len)
{
  for (uint8_t i = 0; i < len; ++i) {
    Wire.requestFrom(addressList[i], (uint8_t)1);
    if (Wire.available()) {
      address = addressList[i];
      return true;
    }
  }
  return false;
}


/** Initiate a conversion by writing to the configuration register
 */
MCP342x::error_t MCP342x::convert(Channel channel, Mode mode, Resolution resolution, Gain gain)
 {
    return convert(Config(channel, mode, resolution, gain));
  }

MCP342x::error_t MCP342x::configure(const Config &config) const
{
  Wire.beginTransmission(address);
  Wire.write(config.val);
  if (Wire.endTransmission())
    return errorConfigureFailed;
  else
    return errorNone;
}

MCP342x::error_t MCP342x::convert(const Config &config) const
{
  Wire.beginTransmission(address);
  Wire.write(config.val | newConversionMask);
  if (Wire.endTransmission())
    return errorConvertFailed;
  else
    return errorNone;
}

MCP342x::error_t MCP342x::read(long &result, Config& status) const
{
  // Read 4 bytes, the 4th byte will configuration. From that deduce
  // if 18 bit conversion. If not use the 3rd byte, as that is the
  // most appropriate configuration value (ready may have changed).
  const uint8_t len = 4;
  uint8_t buffer[len] = {};
  Wire.requestFrom(address, len);
  if (Wire.available() != len)
    return errorReadFailed;
  
  for (uint8_t i = 0; i < len; ++i)
    buffer[i] = Wire.read();

  uint8_t dataBytes;
  if ((buffer[3] & 0x0c) == 0x0c) {
    status = Config(buffer[3]); // 18 bit conversion
    dataBytes = 3;
  }
  else {
    status = Config(buffer[2]);
    dataBytes = 2;
  }
  
  if ((status & notReadyMask) != 0)
    return errorConversionNotReady;

  long signBit = 0;    // Location of sign bit
  long signExtend = 0; // Bits to be set if sign is set
  switch (int(status.getResolution())) {
  case 12:
    signBit = 0x800;
    signExtend = 0xFFFFF000;
    break;
  case 14:
    signBit = 0x2000;
    signExtend = 0xFFFFC000;
    break;
  case 16:
    signBit = 0x8000;
    signExtend = 0xFFFF0000;
    break;
  case 18:
    signBit = 0x20000;
    signExtend = 0xFFFC0000;
    break;
  }

  result = 0;
  for (uint8_t i = 0; i < dataBytes; ++i) {
    result <<= 8;
    result |= (long)buffer[i];
  }
  
  // Fill/blank remaining bits  
  if ((result & signBit) != 0)
    result |= signExtend; // Sign bit is set, sign-extend

  return errorNone;  
}



MCP342x::error_t MCP342x::convertAndRead(Channel channel, Mode mode, Resolution resolution, Gain gain, unsigned long timeout, long &result, Config &status)
{
  error_t err = convert(channel, mode, resolution, gain);
  if (err != errorNone)
    return err;
  unsigned long t = micros() + timeout;
  unsigned long convTime = resolution.getConversionTime();
  if (convTime > 16383) {
    // Unreliable (see arduino reference), use delay() instead
    convTime /= 1000;
    delay(convTime);
  }
  else
    delayMicroseconds(convTime);

  do {
    err = read(result, status);
    if (!err && status.isReady())
      return err;
    
  } while (long(micros() - t) >= 0);
  return errorReadTimeout;
}

unsigned long MCP342x::Resolution::getConversionTime(void) const
{
  switch ((int)(*this)) {
  case 12:
    return 4167; // 240 SPS
  case 14:
    return 16667; // 60 SPS
  case 16:
    return 66667; // 15 SPS
  case 18:
    return 266667; // 3.75 SPS
  }
  return 0; // Shouldn't happen
}

unsigned long MCP342x::Config::getConversionTime(void) const
{
  return Resolution(val).getConversionTime();
  // switch ((int)getResolution()) {
  // case 12:
  //   return 4167; // 240 SPS
  // case 14:
  //   return 16667; // 60 SPS
  // case 16:
  //   return 66667; // 15 SPS
  // case 18:
  //   return 266667; // 3.75 SPS
  // }
  // return 0; // Shouldn't happen
}


