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

#include "Adafruit_SI1145.h"

Adafruit_SI1145::Adafruit_SI1145() {
}

boolean Adafruit_SI1145::check(void) {
  uint8_t id = read8(SI1145_REG_PARTID);
  if (error) return false;
  return id == 0x45; // look for SI1145
}

boolean Adafruit_SI1145::begin(void) {
  if (!check())
    return false;

  reset();

  /***********************************/
  // enable UVindex measurement coefficients!
  write8(SI1145_REG_UCOEFF0, 0x29);
  write8(SI1145_REG_UCOEFF1, 0x89);
  write8(SI1145_REG_UCOEFF2, 0x02);
  write8(SI1145_REG_UCOEFF3, 0x00);

  // enable UV sensor
  writeParam(SI1145_PARAM_CHLIST,
	     SI1145_PARAM_CHLIST_ENUV |
	     SI1145_PARAM_CHLIST_ENALSIR |
	     SI1145_PARAM_CHLIST_ENALSVIS);

  // no interrupts
  write8(SI1145_REG_INTCFG, 0);
  write8(SI1145_REG_IRQEN, 0);

  writeParam(SI1145_PARAM_ALSIRADCMUX, SI1145_PARAM_ADCMUX_SMALLIR);  
  // fastest clocks, clock div 1
  writeParam(SI1145_PARAM_ALSIRADCGAIN, 0);
  // take 511 clocks to measure
  writeParam(SI1145_PARAM_ALSIRADCOUNTER, SI1145_PARAM_ADCCOUNTER_511CLK);
  // in high range mode
  writeParam(SI1145_PARAM_ALSIRADCMISC, SI1145_PARAM_ALSIRADCMISC_RANGE);

  // fastest clocks, clock div 1
  writeParam(SI1145_PARAM_ALSVISADCGAIN, 0);
  // take 511 clocks to measure
  writeParam(SI1145_PARAM_ALSVISADCOUNTER, SI1145_PARAM_ADCCOUNTER_511CLK);
  // in high range mode (not normal signal)
  writeParam(SI1145_PARAM_ALSVISADCMISC, SI1145_PARAM_ALSVISADCMISC_VISRANGE);

  // forced measurement mode
  write8(SI1145_REG_MEASRATE0, 0x00);
  write8(SI1145_REG_MEASRATE1, 0x00);

  return true;
}

void Adafruit_SI1145::reset() {
  write8(SI1145_REG_MEASRATE0, 0);
  write8(SI1145_REG_MEASRATE1, 0);
  write8(SI1145_REG_IRQEN, 0);
  write8(SI1145_REG_IRQMODE1, 0);
  write8(SI1145_REG_IRQMODE2, 0);
  write8(SI1145_REG_INTCFG, 0);
  write8(SI1145_REG_IRQSTAT, 0xFF);

  write8(SI1145_REG_COMMAND, SI1145_RESET);
  delay(10);
  write8(SI1145_REG_HWKEY, 0x17);
  
  delay(10);
}


//////////////////////////////////////////////////////

bool Adafruit_SI1145::readVisible(void) {
  vis = read16(0x22);
  return !error;
}

bool Adafruit_SI1145::readIR(void) {
  ir = read16(0x24);
  return !error;
}

bool Adafruit_SI1145::readUV(void) {
  uv = read16(0x2C);
  return !error;
}

/*********************************************************************/

uint8_t Adafruit_SI1145::writeParam(uint8_t p, uint8_t v) {
  write8(SI1145_REG_PARAMWR, v);
  write8(SI1145_REG_COMMAND, p | SI1145_PARAM_SET);
  return read8(SI1145_REG_PARAMRD);
}

uint8_t Adafruit_SI1145::readParam(uint8_t p) {
  write8(SI1145_REG_COMMAND, p | SI1145_PARAM_QUERY);
  return read8(SI1145_REG_PARAMRD);
}

bool Adafruit_SI1145::cmd(uint8_t code) {
  uint8_t res = response();
  if (res & 0x80) {
    reset();
    res = response();
  }
  last_count = res & 0xf;
  write8(SI1145_REG_COMMAND, code);

  return response() & 0x80 ? false : true;
}

bool Adafruit_SI1145::wait(void) {
  uint8_t res;
  do {
    delay(1);
    res = response();
    if (res & 0x80)
      return false;
  } while (((res & 0x80) == 0) && (last_count == (res & 0xf)));

  return true;
}


/*********************************************************************/

uint8_t  Adafruit_SI1145::read8(uint8_t reg) {
  Wire.beginTransmission(SI1145_ADDR);
  Wire.write((uint8_t)reg);
  Wire.endTransmission();

  Wire.requestFrom(SI1145_ADDR, (size_t)1);
  int ret = Wire.read();
  error = false;
  if (ret == -1) {
    error = true;
    return 0;
  }
  return ret;
}

uint16_t Adafruit_SI1145::read16(uint8_t a) {
  Wire.beginTransmission(SI1145_ADDR); // start transmission to device
  Wire.write(a); // sends register address to read from
  Wire.endTransmission(); // end transmission
  
  Wire.requestFrom(SI1145_ADDR, (size_t)2);// send data n-bytes read
  int d;
  error = false;
  d = Wire.read();
  if (d == -1) {
    error = true;
    return 0;
  }
  uint16_t ret = d;

  d = Wire.read();
  if (d == -1) {
    error = true;
    return 0;
  }
  ret |= (uint16_t)d << 8;

  return ret;
}

void Adafruit_SI1145::write8(uint8_t reg, uint8_t val) {
  Wire.beginTransmission(SI1145_ADDR); // start transmission to device
  Wire.write(reg); // sends register address to write
  Wire.write(val); // sends value
  Wire.endTransmission(); // end transmission
}
