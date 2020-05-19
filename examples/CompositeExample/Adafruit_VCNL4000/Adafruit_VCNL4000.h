/**************************************************************************/
/*! 
    @file     Adafruit_VCNL4000.h
    @author   K. Townsend (Adafruit Industries)
	@license  BSD (see license.txt)
	
	This is a library for the Adafruit VCNL4000 Temp Sensor breakout board
	----> http://www.adafruit.com/products/1782
	
	Adafruit invests time and resources providing this open source code, 
	please support Adafruit and open-source hardware by purchasing 
	products from Adafruit!

	@section  HISTORY

    v1.0  - First release
*/
/**************************************************************************/

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

// the i2c address
#define VCNL4000_I2CADDR_DEFAULT 0x13

// commands and constants
#define VCNL4000_COMMAND 0x80
#define VCNL4000_PRODUCTID 0x81
#define VCNL4000_IRLED 0x83
#define VCNL4000_AMBIENTPARAMETER 0x84
#define VCNL4000_AMBIENTDATA 0x85
#define VCNL4000_AMBIENTDATA 0x85
#define VCNL4000_PROXIMITYDATA 0x87
#define VCNL4000_SIGNALFREQ 0x89
#define VCNL4000_PROXINITYADJUST 0x8A

typedef enum
  {
    VCNL4000_3M125   = 0,
    VCNL4000_1M5625  = 1,
    VCNL4000_781K25  = 2,
    VCNL4000_390K625 = 3,
  } vcnl4000_freq;

#define VCNL4000_MEASUREAMBIENT 0x10
#define VCNL4000_MEASUREPROXIMITY 0x08
#define VCNL4000_AMBIENTREADY 0x40
#define VCNL4000_PROXIMITYREADY 0x20
  
class Adafruit_VCNL4000 {
 public:
  Adafruit_VCNL4000();
  boolean begin(uint8_t a = VCNL4000_I2CADDR_DEFAULT);  

  uint8_t getLEDcurrent(void);
  void setLEDcurrent(uint8_t c);

  void setFrequency(vcnl4000_freq f);
  uint16_t readProximity(void);
  uint16_t readAmbient(void);

 private:

  void write8(uint8_t address, uint8_t data);
  uint16_t read16(uint8_t address);
  uint8_t read8(uint8_t address);

  uint8_t _i2caddr;
};
