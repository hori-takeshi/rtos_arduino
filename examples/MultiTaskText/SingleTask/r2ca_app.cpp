#include "r2ca.h"

//#define LED_BLINK
//#define CLED_BLINK
//#define TOUCH_SENSE
#define LUX_SENSE

#include <Wire.h>
#include <Digital_Light_TSL2561.h>
#include <SeeedOLED.h>
#include <ChainableLED.h>

#define LED_PIN 4
#define TOUCH_PIN 3

#ifdef LED_BLINK

void setup() {
  pinMode(LED_PIN, OUTPUT);
}

void loop() {
  digitalWrite(LED_PIN, HIGH);
  delay(1000);           
  digitalWrite(LED_PIN, LOW); 
  delay(1000);           
}

#endif /* LED_BLINK */

#ifdef CLED_BLINK

#define NUM_LEDS 1
ChainableLED leds(8, 9, NUM_LEDS);

void setup() {
  leds.init();
}

void loop() {
  int i;
  for(i = 0; i < 25; i++){
    leds.setColorRGB(0, i*10, 0, 0);
    delay(10);
  }
  for(i = 0; i < 25; i++){
    leds.setColorRGB(0, 0, i*10, 0);
    delay(10);
  }
  for(i = 0; i < 25; i++){
    leds.setColorRGB(0, 0, 0, i*10);
    delay(10);
  }
}
#endif /* CLED_BLINK */

#ifdef TOUCH_SENSE

int PreTouchValue = 0;
int TouchState = 0;
int LEDState = 0;

void setup() {
  pinMode(TOUCH_PIN, INPUT_PULLUP);
  pinMode(LED_PIN, OUTPUT);
  digitalWrite(LED_PIN, LOW);
}

void loop() {
  int TouchValue = digitalRead(TOUCH_PIN);

  if ((PreTouchValue == 0) && (TouchValue == 1) && (TouchState == 0)) {
    TouchState = 1;
  }

  if ((PreTouchValue == 1) && (TouchValue == 0) && (TouchState == 1)) {
    TouchState = 0;
    if (LEDState == 0) {
      LEDState = 1;
      digitalWrite(LED_PIN, HIGH);
    }
    else {
      LEDState = 0;
      digitalWrite(LED_PIN, LOW);
    }
  }
  PreTouchValue = TouchValue;
}

#endif /* TOUCH_SENSE */


#ifdef LUX_SENSE
void setup() {
  Wire.begin();
  TSL2561.init();
  SeeedOled.init();
  SeeedOled.deactivateScroll();
  SeeedOled.setNormalDisplay();
  SeeedOled.clearDisplay();
}

int OLEDCount = 0;

void loop() {
  int lux = TSL2561.readVisibleLux();

  SeeedOled.setTextXY(0, 0);
  SeeedOled.putString("Digital Light");
  SeeedOled.setTextXY(1, 0);
  SeeedOled.putString("Sensor Value is");
  SeeedOled.setTextXY(2, 0);
  SeeedOled.putNumber(lux);
  SeeedOled.putString(" lux      ");
  SeeedOled.setTextXY(4, 0);
  SeeedOled.putString("Count is");
  SeeedOled.setTextXY(5, 0);
  SeeedOled.putNumber(OLEDCount++);
}
#endif /* LUX_SENSE */
