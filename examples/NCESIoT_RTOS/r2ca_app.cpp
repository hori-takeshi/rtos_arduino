#include "r2ca.h"

//#define USE_INTERRUPT

#include <Wire.h>
#include <Digital_Light_TSL2561.h>
#include <SeeedOLED.h>
#include <ChainableLED.h>

#define LED_PIN  4

extern void task1_setup();
extern void task2_setup();
extern void task3_setup();

void setup()
{
    Serial.begin(115200);
    pinMode(LED_PIN, OUTPUT);

    task1_setup();
    task2_setup();
    task3_setup();
}

void loop() 
{
    digitalWrite(LED_PIN, HIGH);   // turn the LED on (HIGH is the voltage level)
    delay(1000);                   // wait for a second
    digitalWrite(LED_PIN, LOW);    // turn the LED off by making the voltage LOW
    delay(1000);                   // wait for a second    
}


#define TOUCH_PIN 3

int is_update_oled;

void task1_setup() 
{
    Wire.begin();
    TSL2561.init();

    SeeedOled.init();
    SeeedOled.deactivateScroll();

    is_update_oled = 1;
} 
 
void loop1() 
{
    int lux;

    lux = TSL2561.readVisibleLux();
    Serial.print("The Light value is: ");
    Serial.println(lux);

    if (is_update_oled == 1) {
        wai_sem(OLED_SEM);
        SeeedOled.setTextXY(0, 0);        
        SeeedOled.putNumber(lux);
        SeeedOled.putString("    ");        
        sig_sem(OLED_SEM);
    }
    
    delay(1000);
}

#ifdef USE_INTERRUPT
void onTouch(void) {
    Serial.print("hoge");
    iwup_tsk(R2CA_TASK1);
}
#endif /* USE_INTERRUPT */

void task2_setup()
{
    pinMode(TOUCH_PIN, INPUT_PULLUP);
#ifdef USE_INTERRUPT    
    attachInterrupt(TOUCH_PIN, onTouch, CHANGE);
#endif /* USE_INTERRUPT */    
}

void loop2() 
{
#ifdef USE_INTERRUPT    
    slp_tsk();
#else /* USE_INTERRUPT */
    delay(1);    
#endif /* USE_INTERRUPT */
    
    int TouchSensorValue = digitalRead(TOUCH_PIN);

    if(TouchSensorValue==1) {
        is_update_oled = 0;
        wai_sem(OLED_SEM);
        SeeedOled.setInverseDisplay();
        sig_sem(OLED_SEM);
    }else{        
        is_update_oled = 1;
        wai_sem(OLED_SEM);
        SeeedOled.setNormalDisplay();
        sig_sem(OLED_SEM);
    }
}

#define NUM_LEDS  1

ChainableLED leds(8, 9, NUM_LEDS);

void task3_setup()
{
    leds.init();
}

float hue = 0.0;
boolean up = true;
int count = 0;

void loop3()
{
    for (byte i=0; i<NUM_LEDS; i++)
      leds.setColorHSB(i, hue, 1.0, 0.5);
    
    delay(50);
    
    if (up)
      hue+= 0.025;
    else
      hue-= 0.025;
    
    if (hue>=1.0 && up)
      up = false;
    else if (hue<=0.0 && !up)
      up = true;
}
