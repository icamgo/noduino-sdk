#ifndef MCP342X_h
#define MCP342X_h

#define MCP342X_VERSION "1.0.2"

class MCP342x {
public:
  class Config;
  class Channel;
  class Mode;
  class Resolution;
  class Gain;
  
  static const Channel channel1;
  static const Channel channel2;
  static const Channel channel3;
  static const Channel channel4;

  static const Mode oneShot;
  static const Mode continous;
  
  static const Resolution resolution12;
  static const Resolution resolution14;
  static const Resolution resolution16;
  static const Resolution resolution18;
  
  static const Gain gain1;
  static const Gain gain2;
  static const Gain gain4;
  static const Gain gain8;

  static const uint8_t notReadyMask = 0x80;
  static const uint8_t newConversionMask = 0x80;
  static const uint8_t numChannels = 4;
  static const uint8_t maxResolution = 18;
  static const uint8_t maxGain = 8;
  static const int writeTimeout_us = 250;

  enum error_t {
    errorNone,
    errorConvertFailed,
    errorReadFailed,
    errorReadTimeout,
    errorConversionNotReady,
    errorConfigureFailed,
  };

  static uint8_t generalCallReset(void);
  static uint8_t generalCallLatch(void);
  static uint8_t generalCallConversion(void);

  // Adjust result to account for gain and resolution settings
  static void normalise(long &result, Config config);

  MCP342x(void);
  MCP342x(uint8_t address);

  bool autoprobe(const uint8_t *addressList, uint8_t len);
  
  /** Return the I2C address used for communicating with this device.
   */
  uint8_t getAddress(void) const {
    return address;
  }

  /** Configure the device. Useful only for generalCallConversion()
   */
  error_t configure(const Config &config) const;

  
  /** Instruct the MCP342x device to begin a conversion.
   * @param channel The ADC channel, one of channel0, channel1,
   * channel2 or channel3. Not all supported devices have 4
   * channels.
   * @param mode The conversion mode, oneShot or continous.
   * @param resolution Number of bits in the result, one of res12,
   * res14, res16 or res18. Not all devices support 18-bit resolution.
   * @param gain The gain setting of the programmable gain amplifier,
   * one of gain1, gain2, gain4 or gain8.
   * @return Value indicating error (if any).
   */
  error_t convert(Channel channel, Mode mode, Resolution resolution, Gain gain);  error_t convert(const Config &config) const;
  

  
  /** Read the sample value from the MCP342x device.
   * @param result The signed result.
   * @param config The contents of the configuration register.
   * @return Value indicating error (if any).
   */
  error_t read(long &result, uint8_t& config) const;
  error_t read(long &result, Config& config) const;

  /** Instruct the MCP342x device to begin a conversion and block
   * until read completes or timed out.
   * @param channel The ADC channel, one of channel0, channel1,
   * channel2 or channel3. Not all supported devices have 4
   * channels.
   * @param mode The conversion mode, oneShotMode or continousMode.
   * @param resolution Number of bits in the result, one of res12,
   * res14, res16 or res18. Not all devices support 18-bit resolution.
   * @param gain The gain setting of the programmable gain amplifier,
   * one of gain1, gain2, gain4 or gain8.
   * @param timeout The time out value in microseconds. 
   * @param result The signed result.
   * @param config The contents of the configuration register.
   * @return Value indicating error (if any).
   */
  error_t convertAndRead(Channel channel, Mode mode, Resolution resolution, Gain gain, unsigned long timeout, long &result, Config &status);

private:
  uint8_t address;
  // For easy readout need to know whether 18 bit mode was selected
  // uint8_t config;
};


class MCP342x::Channel {
  friend class MCP342x;
  friend class MCP342x::Config;
public:
  inline operator int(void) const {
    return (val >> 5) + 1;
  }

private:
  inline Channel(uint8_t v) : val(v & 0x60) {
  };
  
  uint8_t val;
};


class MCP342x::Mode {
  friend class MCP342x;
  friend class MCP342x::Config;
public:
  //inline operator int(void) const {
  //return (val >> 1) + 12;
  //}
private:
  inline Mode(uint8_t v) : val(v & 0x10) {
  };

  uint8_t val;
};

class MCP342x::Resolution {
  friend class MCP342x;
  friend class MCP342x::Config;
public:
  inline operator int(void) const {
    return (val >> 1) + 12;
  }
  unsigned long getConversionTime(void) const;
private:
  inline Resolution(uint8_t v) : val(v & 0x0c) {
  };

  uint8_t val;
};


class MCP342x::Gain {
  friend class MCP342x;
  friend class MCP342x::Config;
public:
    inline operator int(void) const {
      return (1 << val);
    }
  inline uint8_t log2(void) const {
    return val;
  }
private:
  inline Gain(uint8_t v) : val(v & 0x03) {
  };

  uint8_t val;
};


class MCP342x::Config {
  friend class MCP342x;
public:
  inline Config(void) : val(0) {
  };

  inline Config(uint8_t v) : val(v) {
  };

  inline Config(Channel c, Mode m, Resolution r, Gain g) :
    val(c.val | m.val | r.val | g.val) {
  };
  
  inline Config(uint8_t c, bool continuous, uint8_t r, uint8_t g) :
    val((((c-1) & 3) << 5)
	| (uint8_t)(continuous ? 0x10 : 0)
	| ((((r-12) & 0x1e) << 1) & 0xc)) {
    switch(g) {
    case 2:
      val |= 0x01;
      break;
    case 4:
      val |= 0x02;
      break;
    case 8:
      val |= 0x03;
      break;
    };
  }
  
  inline operator int(void) const {
    return val;
  }

  inline Channel getChannel(void) const {
    return Channel(val);
  }
  inline Resolution getResolution(void) const {
    return Resolution(val);
  }
  inline Gain getGain(void) const {
    return Gain(val);
  }
  inline bool isReady(void) const {
    return !(val & notReadyMask); 
  }

  unsigned long getConversionTime(void) const;

  
private:
  uint8_t val;
};


#endif
