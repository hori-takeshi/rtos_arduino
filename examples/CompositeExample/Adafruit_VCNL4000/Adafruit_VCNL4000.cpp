#if ARDUINO >= 100
 #include "Arduino.h"
#else
 #include "WProgram.h"
#endif

#ifdef __AVR_ATtiny85__
 #include "TinyWireM.h"
 #define Wire TinyWireM
#else
 #include <Wire.h>
#endif

#include "Adafruit_VCNL4000.h"

/**************************************************************************/
/*! 
    @brief  Instantiates a new VCNL4000 class
*/
/**************************************************************************/
Adafruit_VCNL4000::Adafruit_VCNL4000() {
}

/**************************************************************************/
/*! 
    @brief  Setups the HW
*/
/**************************************************************************/
boolean Adafruit_VCNL4000::begin(uint8_t addr) {
  _i2caddr = addr;
  Wire.begin();

  uint8_t rev = read8(VCNL4000_PRODUCTID);
  //Serial.println(rev, HEX);
  if ((rev & 0xF0) != 0x10) {
      return false;
  }
  
  setLEDcurrent(20);
  setFrequency(VCNL4000_781K25);
  write8(VCNL4000_PROXINITYADJUST, 0x81);
    
  return true;
}
 

/**************************************************************************/
/*! 
    @brief  Get and set the LED current draw
*/
/**************************************************************************/

void Adafruit_VCNL4000::setLEDcurrent(uint8_t c) {
  if (c > 20) c = 20;
  write8(VCNL4000_IRLED, c);
}

uint8_t Adafruit_VCNL4000::getLEDcurrent(void) {
  return read8(VCNL4000_IRLED);
}

/**************************************************************************/
/*! 
    @brief  Get and set the measurement signal frequency
*/
/**************************************************************************/

void Adafruit_VCNL4000::setFrequency(vcnl4000_freq f) {
  write8(VCNL4000_SIGNALFREQ, f);
}


/**************************************************************************/
/*! 
    @brief  Get proximity measurement
*/
/**************************************************************************/

uint16_t  Adafruit_VCNL4000::readProximity(void) {
  write8(VCNL4000_COMMAND, VCNL4000_MEASUREPROXIMITY);
  while (1) {
    //Serial.println(read8(VCNL4000_INTSTAT), HEX);
    uint8_t result = read8(VCNL4000_COMMAND);
    //Serial.print("Ready = 0x"); Serial.println(result, HEX);
    if (result & VCNL4000_PROXIMITYREADY) {
      return read16(VCNL4000_PROXIMITYDATA);
    }
    delay(1);
  }
}

uint16_t  Adafruit_VCNL4000::readAmbient(void) {
  write8(VCNL4000_COMMAND, VCNL4000_MEASUREAMBIENT);
  while (1) {
    //Serial.println(read8(VCNL4000_INTSTAT), HEX);
    uint8_t result = read8(VCNL4000_COMMAND);
    //Serial.print("Ready = 0x"); Serial.println(result, HEX);
    if (result & VCNL4000_AMBIENTREADY) {
      return read16(VCNL4000_AMBIENTDATA);
    }
    delay(1);
  }
}

/**************************************************************************/
/*! 
    @brief  I2C low level interfacing
*/
/**************************************************************************/


// Read 1 byte from the VCNL4000 at 'address'
uint8_t Adafruit_VCNL4000::read8(uint8_t address)
{
  uint8_t data;

  Wire.beginTransmission(_i2caddr);
#if ARDUINO >= 100
  Wire.write(address);
#else
  Wire.send(address);
#endif
  Wire.endTransmission();

  delayMicroseconds(170);  // delay required

  Wire.requestFrom(_i2caddr, (uint8_t)1);
  while(!Wire.available());

#if ARDUINO >= 100
  return Wire.read();
#else
  return Wire.receive();
#endif
}


// Read 2 byte from the VCNL4000 at 'address'
uint16_t Adafruit_VCNL4000::read16(uint8_t address)
{
  uint16_t data;

  Wire.beginTransmission(_i2caddr);
#if ARDUINO >= 100
  Wire.write(address);
#else
  Wire.send(address);
#endif
  Wire.endTransmission();

  Wire.requestFrom(_i2caddr, (uint8_t)2);
  while(!Wire.available());
#if ARDUINO >= 100
  data = Wire.read();
  data <<= 8;
  while(!Wire.available());
  data |= Wire.read();
#else
  data = Wire.receive();
  data <<= 8;
  while(!Wire.available());
  data |= Wire.receive();
#endif
  
  return data;
}

// write 1 byte
void Adafruit_VCNL4000::write8(uint8_t address, uint8_t data)
{
  Wire.beginTransmission(_i2caddr);
#if ARDUINO >= 100
  Wire.write(address);
  Wire.write(data);  
#else
  Wire.send(address);
  Wire.send(data);  
#endif
  Wire.endTransmission();
}
