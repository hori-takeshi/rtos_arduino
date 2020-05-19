#include "r2ca.h"

#define SERVO
//#define ULTRASONIC
//#define LCD_ULTRASONIC
//#define LCD_ULTRASONIC_SERVO  /* define R2CA_NUM_TASK as 1 */

#ifdef SERVO

/* Sweep
 by BARRAGAN <http://barraganstudio.com> 
 This example code is in the public domain.

 modified 8 Nov 2013
 by Scott Fitzgerald
 http://arduino.cc/en/Tutorial/Sweep
*/ 

#include <Servo.h> 
 
Servo myservo;  // create servo object to control a servo 
                // twelve servo objects can be created on most boards
 
int pos = 0;    // variable to store the servo position 
 
void setup() 
{ 
  myservo.attach(2);  // attaches the servo on pin 9 to the servo object 
} 
 
void loop() 
{ 
  for(pos = 0; pos <= 180; pos += 1) // goes from 0 degrees to 180 degrees 
  {                                  // in steps of 1 degree 
    myservo.write(pos);              // tell servo to go to position in variable 'pos' 
    delay(15);                       // waits 15ms for the servo to reach the position 
  } 
  for(pos = 180; pos>=0; pos-=1)     // goes from 180 degrees to 0 degrees 
  {                                
    myservo.write(pos);              // tell servo to go to position in variable 'pos' 
    delay(15);                       // waits 15ms for the servo to reach the position 
  } 
} 

#endif /* SERVO */

#ifdef ULTRASONIC

/*
 * UltrasonicDisplayOnTerm.ino
 * Example sketch for ultrasonic ranger
 *
 * Copyright (c) 2012 seeed technology inc.
 * Website    : www.seeed.cc
 * Author     : LG, FrankieChu
 * Create Time: Jan 17,2013
 * Change Log :
 *
 * The MIT License (MIT)
 *
 * Permission is hereby granted, free of charge, to any person obtaining a copy
 * of this software and associated documentation files (the "Software"), to deal
 * in the Software without restriction, including without limitation the rights
 * to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
 * copies of the Software, and to permit persons to whom the Software is
 * furnished to do so, subject to the following conditions:
 *
 * The above copyright notice and this permission notice shall be included in
 * all copies or substantial portions of the Software.
 *
 * THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
 * IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
 * FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
 * AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
 * LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
 * OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN
 * THE SOFTWARE.
 */


/***************************************************************************/
//	Function: Measure the distance to obstacles in front and print the distance
//			  value to the serial terminal.The measured distance is from
//			  the range 0 to 400cm(157 inches).
//	Hardware: Grove - Ultrasonic Ranger
//	Arduino IDE: Arduino-1.0
/*****************************************************************************/

#include "Ultrasonic.h"

Ultrasonic ultrasonic(3);

extern void task1_setup();

void setup()
{
	Serial.begin(115200);
}
void loop()
{
	long RangeInCentimeters;
	RangeInCentimeters = ultrasonic.MeasureInCentimeters(); // two measurements should keep an interval
	Serial.print(RangeInCentimeters);//0~400cm
	Serial.println(" cm");
	delay(250);

	task1_setup();
}

#endif /* ULTRASONIC */

#ifdef LCD_ULTRASONIC

#include <Wire.h>
#include <SeeedGrayOLED.h>
#include <avr/pgmspace.h>
#include "Ultrasonic.h"

Ultrasonic ultrasonic(3);

void setup()
{
    Wire.begin();
    SeeedGrayOled.init();             //initialize SEEED OLED display
    SeeedGrayOled.clearDisplay();     //Clear Display.
    SeeedGrayOled.setNormalDisplay(); //Set Normal Display Mode
    SeeedGrayOled.setVerticalMode();  // Set to vertical mode for displaying text
}
 
void loop()
{
    long RangeInCentimeters;

    SeeedGrayOled.setTextXY(0,0);  //set Cursor to ith line, 0th column
    SeeedGrayOled.setGrayLevel(15); //Set Grayscale level. Any number between 0 - 15.    
    RangeInCentimeters = ultrasonic.MeasureInCentimeters(); // two measurements should keep an interval
    SeeedGrayOled.putNumber(RangeInCentimeters);//0~400cm
    SeeedGrayOled.putString(" cm      ");
    delay(250);    
}
#endif /* LCD_ULTRASONIC */

#ifdef LCD_ULTRASONIC_SERVO

#include <Wire.h>
#include <SeeedGrayOLED.h>
#include <avr/pgmspace.h>
#include "Ultrasonic.h"

Ultrasonic ultrasonic(3);

void setup()
{
    Wire.begin();
    SeeedGrayOled.init();             //initialize SEEED OLED display
    SeeedGrayOled.clearDisplay();     //Clear Display.
    SeeedGrayOled.setNormalDisplay(); //Set Normal Display Mode
    SeeedGrayOled.setVerticalMode();  // Set to vertical mode for displaying text
}
 
void loop()
{
    long RangeInCentimeters;

    SeeedGrayOled.setTextXY(0,0);  //set Cursor to ith line, 0th column
    SeeedGrayOled.setGrayLevel(15); //Set Grayscale level. Any number between 0 - 15.    
    RangeInCentimeters = ultrasonic.MeasureInCentimeters(); // two measurements should keep an interval
    SeeedGrayOled.putNumber(RangeInCentimeters);//0~400cm
    SeeedGrayOled.putString(" cm      ");
    delay(250);    
}

/* Sweep
 by BARRAGAN <http://barraganstudio.com> 
 This example code is in the public domain.

 modified 8 Nov 2013
 by Scott Fitzgerald
 http://arduino.cc/en/Tutorial/Sweep
*/ 

#include <Servo.h> 
 
Servo myservo;  // create servo object to control a servo 
                // twelve servo objects can be created on most boards
 
int pos = 0;    // variable to store the servo position 
 
void task1_setup() 
{ 
  myservo.attach(2);  // attaches the servo on pin 9 to the servo object 
} 
 
void loop1() 
{ 
  for(pos = 0; pos <= 180; pos += 1) // goes from 0 degrees to 180 degrees 
  {                                  // in steps of 1 degree 
    myservo.write(pos);              // tell servo to go to position in variable 'pos' 
    delay(15);                       // waits 15ms for the servo to reach the position 
  } 
  for(pos = 180; pos>=0; pos-=1)     // goes from 180 degrees to 0 degrees 
  {                                
    myservo.write(pos);              // tell servo to go to position in variable 'pos' 
    delay(15);                       // waits 15ms for the servo to reach the position 
  } 
} 

#endif /* LCD_ULTRASONIC */
