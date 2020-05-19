#include "r2ca.h"

#include <ChainableLED.h>

#define LED_PIN 4

#define NUM_LEDS 1
ChainableLED leds(8, 9, NUM_LEDS);

void setup() {
  pinMode(LED_PIN, OUTPUT);

  leds.init();
}

void loop() {
  digitalWrite(LED_PIN, HIGH);
  delay(1000);           
  digitalWrite(LED_PIN, LOW); 
  delay(1000);           
}

void loop1() {
  int i;
  leds.setColorRGB(0, 250, 0, 0);
  delay(250);
    
  leds.setColorRGB(0, 0, 250, 0);
  delay(250);

  leds.setColorRGB(0, 0, 0, 250);
  delay(250);
}
