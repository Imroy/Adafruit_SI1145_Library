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
  uint8_t id = read8(PARTID);
  if (error) return false;
  return id == 0x45; // look for SI1145
}

boolean Adafruit_SI1145::begin(void) {
  if (!check())
    return false;

  reset();

  /***********************************/
  // enable UVindex measurement coefficients!
  write8(UCOEFF0, 0x29);	// vis = 0.033486
  write8(UCOEFF1, 0x89);
  write8(UCOEFF2, 0x02);	// ir = 0.000002
  write8(UCOEFF3, 0x00);

  // enable UV sensor
  writeParam(CHLIST,
	     CHLIST_ENUV |
	     CHLIST_ENALSIR |
	     CHLIST_ENALSVIS);

  // no interrupts
  write8(INTCFG, 0);
  write8(IRQEN, 0);

  writeParam(ALSIRADCMUX, ADCMUX_SMALLIR);
  // fastest clocks, clock div 1
  writeParam(ALSIRADCGAIN, 0);
  // take 511 clocks to measure
  writeParam(ALSIRADCOUNTER, ADCCOUNTER_511CLK);
  // in high range mode
  writeParam(ALSIRADCMISC, ALSIRADCMISC_RANGE);

  // fastest clocks, clock div 1
  writeParam(ALSVISADCGAIN, 0);
  // take 511 clocks to measure
  writeParam(ALSVISADCOUNTER, ADCCOUNTER_511CLK);
  // in high range mode (not normal signal)
  writeParam(ALSVISADCMISC, ALSVISADCMISC_VISRANGE);

  // forced measurement mode
  write8(MEASRATE0, 0x00);
  write8(MEASRATE1, 0x00);

  return true;
}

void Adafruit_SI1145::reset() {
  write8(MEASRATE0, 0);
  write8(MEASRATE1, 0);
  write8(IRQEN, 0);
  write8(IRQMODE1, 0);
  write8(IRQMODE2, 0);
  write8(INTCFG, 0);
  write8(IRQSTAT, 0xFF);

  write8(COMMAND, RESET);
  delay(10);
  write8(HWKEY, 0x17);

  delay(10);
}


//////////////////////////////////////////////////////

bool Adafruit_SI1145::readVisible(void) {
  vis = read16(ALSVISDATA0);
  return !error;
}

bool Adafruit_SI1145::readIR(void) {
  ir = read16(ALSIRDATA0);
  return !error;
}

bool Adafruit_SI1145::readUV(void) {
  uv = read16(UVINDEX0);
  return !error;
}

/*********************************************************************/

uint8_t Adafruit_SI1145::writeParam(SI1145_parameter p, uint8_t v) {
  write8(PARAMWR, v);
  write8(COMMAND, p | PARAM_SET);
  return read8(PARAMRD);
}

uint8_t Adafruit_SI1145::readParam(SI1145_parameter p) {
  write8(COMMAND, p | PARAM_QUERY);
  return read8(PARAMRD);
}

bool Adafruit_SI1145::cmd(SI1145_command code) {
  uint8_t res = response();
  if (res & 0x80) {
    reset();
    res = response();
  }
  last_count = res & 0xf;
  write8(COMMAND, code);

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

uint8_t  Adafruit_SI1145::read8(SI1145_register reg) {
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

uint16_t Adafruit_SI1145::read16(SI1145_register reg) {
  Wire.beginTransmission(SI1145_ADDR); // start transmission to device
  Wire.write(reg); // sends register address to read from
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

void Adafruit_SI1145::write8(SI1145_register reg, uint8_t val) {
  Wire.beginTransmission(SI1145_ADDR); // start transmission to device
  Wire.write(reg); // sends register address to write
  Wire.write(val); // sends value
  Wire.endTransmission(); // end transmission
}
