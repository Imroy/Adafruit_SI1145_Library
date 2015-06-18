/***************************************************
  This is a library for the Si1145 UV/IR/Visible Light Sensor

  Designed specifically to work with the Si1145 sensor in the
  adafruit shop
  ----> https://www.adafruit.com/products/1777

  These sensors use I2C to communicate, 2 pins are required to
  interface
  Adafruit invests time and resources providing this open source code,
  please support Adafruit and open-source hardware by purchasing
  products from Adafruit!

  Written by Limor Fried/Ladyada for Adafruit Industries.
  BSD license, all text above must be included in any redistribution
 ****************************************************/


#if (ARDUINO >= 100)
 #include "Arduino.h"
#else
 #include "WProgram.h"
#endif
#include <Wire.h>

/* COMMANDS */
enum SI1145_command {
  NOP,
  RESET,
  BUSADDR,

  PS_FORCE	= 0x05,
  ALS_FORCE,
  PSALS_FORCE,

  PS_PAUSE	= 0x09,
  ALS_PAUSE,
  PSALS_PAUSE,

  PS_AUTO	= 0x0D,
  ALS_AUTO,
  PSALS_AUTO,

  GET_CAL_INDEX = 0x11,
  GET_CAL,

  // bottom 5 bits are the "parameter" index
  PARAM_QUERY	= 0x80,
  PARAM_SET	= 0xA0,
};

enum SI1145_register {
  PARTID,
  REVID,
  SEQID,
  INTCFG,
  IRQEN,
  IRQMODE1,
  IRQMODE2,
  HWKEY,
  MEASRATE0,
  MEASRATE1,
  PSRATE,

  PSLED21	= 0x0F,
  PSLED3,

  UCOEFF0	= 0x13,
  UCOEFF1,
  UCOEFF2,
  UCOEFF3,
  PARAMWR,
  COMMAND,

  RESPONSE	= 0x20,
  IRQSTAT,
  ALSVISDATA0,
  ALSVISDATA1,
  ALSIRDATA0,
  ALSIRDATA1,
  PS1DATA0,
  PS1DATA1,
  PS2DATA0,
  PS2DATA1,
  PS3DATA0,
  PS3DATA1,
  UVINDEX0,
  UVINDEX1,
  PARAMRD,

  CHIPSTAT	= 0x30,
};

enum SI1145_register_value {
  INTCFG_INTOE = 1,
  INTCFG_INTMODE,

  IRQEN_ALSEVERYSAMPLE = 0x01,
  IRQEN_PS1EVERYSAMPLE = 0x04,
  IRQEN_PS2EVERYSAMPLE = 0x08,
  IRQEN_PS3EVERYSAMPLE = 0x10,

  IRQSTAT_ALS		= 0x01,
};

enum SI1145_parameter {
  I2CADDR,
  CHLIST,
  PSLED12SEL,
  PSLED3SEL,

  PSENCODE		= 0x05,
  ALSENCODE,
  PS1ADCMUX,
  PS2ADCMUX,
  PS3ADCMUX,
  PSADCOUNTER,
  PSADCGAIN,
  PSADCMISC,

  ALSIRADCMUX		= 0x0E,
  AUXADCMUX,
  ALSVISADCOUNTER,
  ALSVISADCGAIN,
  ALSVISADCMISC,

  ALSIRADCOUNTER	= 0x1D,
  ALSIRADCGAIN,
  ALSIRADCMISC,
};

enum SI1145_parameter_value {
  CHLIST_ENPS1		= 0x01,
  CHLIST_ENPS2		= 0x02,
  CHLIST_ENPS3		= 0x04,
  CHLIST_ENALSVIS	= 0x10,
  CHLIST_ENALSIR	= 0x20,
  CHLIST_ENAUX		= 0x40,
  CHLIST_ENUV		= 0x80,

  PSLED12SEL_PS2NONE	= 0x00,
  PSLED12SEL_PS2LED1	= 0x10,
  PSLED12SEL_PS2LED2	= 0x20,
  PSLED12SEL_PS2LED3	= 0x40,
  PSLED12SEL_PS1NONE	= 0x00,
  PSLED12SEL_PS1LED1	= 0x01,
  PSLED12SEL_PS1LED2	= 0x02,
  PSLED12SEL_PS1LED3	= 0x04,

  PSADCMISC_RANGE	= 0x20,
  PSADCMISC_PSMODE	= 0x04,

  ALSVISADCMISC_VISRANGE= 0x20,

  ALSIRADCMISC_RANGE	= 0x20,

  ADCCOUNTER_511CLK	= 0x70,

  ADCMUX_SMALLIR	= 0x00,
  ADCMUX_LARGEIR	= 0x03,
};

#define SI1145_ADDR				(uint8_t)0x60


class Adafruit_SI1145  {
 public:
  Adafruit_SI1145(void);
  boolean check(void);
  boolean begin(void);
  void reset(void);

  unsigned long doMeasurement(void) {
    return cmd(ALS_FORCE) ? 2 : 0;
  }
  bool wait(void);
  bool readIR(void);
  bool readVisible(void);
  bool readUV(void);

  uint16_t IR(void) const { return ir; }
  uint16_t visible(void) const { return vis; }
  uint16_t UV(void) const { return uv; }

 private:
  uint16_t uv, ir, vis;
  bool error;
  uint8_t last_count;

  uint8_t response(void) { return read8(RESPONSE); }
  bool cmd(SI1145_command code);

  uint16_t read16(SI1145_register reg);
  uint8_t read8(SI1145_register reg);
  void write8(SI1145_register reg, uint8_t val);

  uint8_t readParam(SI1145_parameter p);
  uint8_t writeParam(SI1145_parameter p, uint8_t v);
};

