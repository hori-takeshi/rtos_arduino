#include "r2ca.h"

#include <Wire.h>
#include <Digital_Light_TSL2561.h>
#include <SeeedOLED.h>

#define TOUCH_PIN 3
#define LED_PIN  4

int PreTouchValue = 0;
int TouchState = 0;
int LEDState = 0;

extern void CheckTouch(void);
    
void setup() {
  Wire.begin();
  TSL2561.init();
  SeeedOled.init();
  SeeedOled.deactivateScroll();
  SeeedOled.setNormalDisplay();
  SeeedOled.clearDisplay();
  
  pinMode(TOUCH_PIN, INPUT_PULLUP);
  pinMode(LED_PIN, OUTPUT);
  digitalWrite(LED_PIN, LOW);    
  attachInterrupt(TOUCH_PIN, CheckTouch, CHANGE);
}

int OLEDCount = 0;

void CheckTouch(){
   int TouchValue = digitalRead(TOUCH_PIN);

  if ((PreTouchValue == 0) && (TouchValue == 1) && (TouchState == 0)) {
    TouchState = 1;
  }

  if ((PreTouchValue == 1) && (TouchValue == 0) && (TouchState == 1)) {
      TouchState = 0;
      iwup_tsk(R2CA_TASK1);
  }
  PreTouchValue = TouchValue; 
}

void loop() {
  int lux;
  int lcount;

//  wai_sem(COUNT_SEM);
  lcount = OLEDCount;        
  lux = TSL2561.readVisibleLux();
    
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
  SeeedOled.putNumber(lcount++);
  SeeedOled.putString("         "); 
  OLEDCount = lcount;
//  sig_sem(COUNT_SEM);
}


void loop1() {
    slp_tsk();
    
//    wai_sem(COUNT_SEM);
    OLEDCount = 0;
//    sig_sem(COUNT_SEM);
    
    digitalWrite(LED_PIN, HIGH);
    dly_tsk(2000);
    digitalWrite(LED_PIN, LOW);
}
