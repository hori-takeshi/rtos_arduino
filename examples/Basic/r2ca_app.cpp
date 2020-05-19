#include "r2ca.h"

#define BLINK
//#define toneMelody
//#define USBUART
//#define SERIALUSB
//#define SERIAL5
//#define ATTACHINTERRUPT
//#define ANALOGWRITE
//#define ANALOGREAD
//#define RTC_ALARM
//#define SD_CARD

#ifdef BLINK
/*
  Blink
  Turns on an LED on for one second, then off for one second, repeatedly.

  Most Arduinos have an on-board LED you can control. On the Uno and
  Leonardo, it is attached to digital pin 13. If you're unsure what
  pin the on-board LED is connected to on your Arduino model, check
  the documentation at http://arduino.cc

  This example code is in the public domain.

  modified 8 May 2014
  by Scott Fitzgerald
 */

// the setup function runs once when you press reset or power the board
void setup() {
  // initialize digital pin 13 as an output.
  pinMode(13, OUTPUT);
  Serial.begin(115200);
}

// the loop function runs over and over again forever
void loop() {
    digitalWrite(13, HIGH);   // turn the LED on (HIGH is the voltage level)
    Serial.println("HIGH");    
    delay(1000);              // wait for a second
    digitalWrite(13, LOW);    // turn the LED off by making the voltage LOW
    Serial.println("LOW");        
    delay(1000);              // wait for a second
}
#endif /* BLINK */

#ifdef toneMelody
/*
  Melody

 Plays a melody

 circuit:
 * 8-ohm speaker on digital pin 8

 created 21 Jan 2010
 modified 30 Aug 2011
 by Tom Igoe

This example code is in the public domain.

 http://arduino.cc/en/Tutorial/Tone

 */
#include "pitches.h"

// notes in the melody:
int melody[] = {
  NOTE_C4, NOTE_G3, NOTE_G3, NOTE_A3, NOTE_G3, 0, NOTE_B3, NOTE_C4
};

// note durations: 4 = quarter note, 8 = eighth note, etc.:
int noteDurations[] = {
  4, 8, 8, 4, 4, 4, 4, 4
};

void setup() {
    // no need to repeat the melody.
}

void loop() {
  // iterate over the notes of the melody:
  for (int thisNote = 0; thisNote < 8; thisNote++) {

    // to calculate the note duration, take one second
    // divided by the note type.
    //e.g. quarter note = 1000 / 4, eighth note = 1000/8, etc.
    int noteDuration = 1000 / noteDurations[thisNote];
    tone(8, melody[thisNote], noteDuration);

    // to distinguish the notes, set a minimum time between them.
    // the note's duration + 30% seems to work well:
    int pauseBetweenNotes = noteDuration * 1.30;
    delay(pauseBetweenNotes);
    // stop the tone playing:
    noTone(8);
  }
  delay(2000);
}
#endif /* toneMelody */

#ifdef SERIALUSB

void setup() {
    SerialUSB.begin(115200);
    while(!SerialUSB){ ; }
    SerialUSB.println("SerialUSB start!");
}

int val;
void loop() {
    SerialUSB.println("arrive!");
    delay(1000);
}
#endif /* SERIALUSB */

#ifdef SERIAL5

void setup() {
    Serial5.begin(115200);
    Serial5.println("Serial5 start!");
}

int val;
void loop() {
    Serial5.println("arrive");
    delay(1000);
}
#endif /* SERIAL5 */

#ifdef ATTACHINTERRUPT
const int buttonPin = 7;     // the number of the pushbutton pin
const int ledPin =  13;      // the number of the LED pin

// variables will change:
int buttonState = 0;         // variable for reading the pushbutton status

extern void blink(void);
    
void setup() {
  // initialize the LED pin as an output:
  pinMode(ledPin, OUTPUT);
  // initialize the pushbutton pin as an input:
  pinMode(buttonPin, INPUT);
  Serial.begin(115200);
  attachInterrupt(buttonPin, blink, CHANGE);
}

int interrupt;
/* ToDo */
void
blink(void){
//    Serial.println("interrupt!");
    syslog(LOG_NOTICE, "data = %d", interrupt++);
}

void loop() {
  // read the state of the pushbutton value:
  buttonState = digitalRead(buttonPin);

  // check if the pushbutton is pressed.
  // if it is, the buttonState is HIGH:
  if (buttonState == HIGH) {
    // turn LED on:
    digitalWrite(ledPin, HIGH);
  }
  else {
    // turn LED off:
    digitalWrite(ledPin, LOW);
  }
  delay(10);
}
#endif /* ATTACHINTERRUPT */

#ifdef ANALOGWRITE

const int ledPin =  13;

void setup() {
}

void loop() {
  int i;
  while(1){
    for(i= 0; i<255; i++){
      analogWrite(ledPin,i);
      delay(10);
    }
    for(i= 255; i>0; i--){
      analogWrite(ledPin,i);
      delay(10);
    }        
  }
}
#endif /* ANALOGWRITE */

#ifdef ANALOGREAD
/*
  Analog Input
 Demonstrates analog input by reading an analog sensor on analog pin 0 and
 turning on and off a light emitting diode(LED)  connected to digital pin 13.
 The amount of time the LED will be on and off depends on
 the value obtained by analogRead().

 The circuit:
 * Potentiometer attached to analog input 0
 * center pin of the potentiometer to the analog pin
 * one side pin (either one) to ground
 * the other side pin to +5V
 * LED anode (long leg) attached to digital output 13
 * LED cathode (short leg) attached to ground

 * Note: because most Arduinos have a built-in LED attached
 to pin 13 on the board, the LED is optional.


 Created by David Cuartielles
 modified 30 Aug 2011
 By Tom Igoe

 This example code is in the public domain.

 http://arduino.cc/en/Tutorial/AnalogInput

 */
int sensorPin = A0;    // select the input pin for the potentiometer
int ledPin = 13;      // select the pin for the LED
int sensorValue = 0;  // variable to store the value coming from the sensor

void setup() {
  // declare the ledPin as an OUTPUT:
  pinMode(ledPin, OUTPUT);
}

int i = 0;
void loop() {
    // read the value from the sensor:
    sensorValue = analogRead(sensorPin);
    analogWrite(ledPin,sensorValue/4);
    delay(10);
    if(i++ == 100){
        i = 0;
        syslog(LOG_NOTICE, "hoge = %d", sensorValue);
    }
}
#endif /* ANALOGREAD */

#ifdef RTC_ALARM
/*****************************************************************************************************************************************************************************
* This sketch demonstrate how to use alarm in interrupt mode.
This mode is more conveniently because you use processor for other tasks and when alarm match occurs interrupt routine is executed.
In this way, alarm flag checking is indipendent from main program flow.
******************************************************************************************************************************************************************************/

#include <RTCInt.h>

RTCInt rtc;

extern void alarm_int(void);

void setup() 
{
  Serial.begin(115200);       //serial communication initializing
  pinMode(13,OUTPUT);
  rtc.begin(TIME_H24);      //RTC initializing with 24 hour representation mode
  rtc.setTime(17,0,5,0);    //setting time (hour minute and second)
  rtc.setDate(13,8,15);     //setting date
  rtc.enableAlarm(SEC,ALARM_INTERRUPT,alarm_int); //enabling alarm in polled mode and match on second
  rtc.local_time.hour=17;
  rtc.local_time.minute=5;
  rtc.local_time.second=10;  //setting second to match
  rtc.setAlarm();  //write second in alarm register
}

void loop()
{
//  digitalWrite(13,HIGH);   //main program code
  Serial.println("HIGH!");
  delay(1000);
//  digitalWrite(13,LOW);
  Serial.println("LOW!");
  delay(1000);
  
}


/*************** Interrupt routine for alarm ******************************/
void alarm_int(void)
{
  Serial.println("Alarm match!");
//    RTC->MODE2.INTFLAG.bit.ALARM0=1; //clearing alarm0 flag
     rtc.getDate();      //getting date in local structure (local_date)
 rtc.getTime();      //getting time in local structure(local_time)
 
 //printing date in format YYYY/MM/DD
 Serial.print(rtc.local_date.year+2000); // year
 Serial.print('/');
 Serial.print(rtc.local_date.month);    // month
 Serial.print('/');
 Serial.print(rtc.local_date.day);      // day
 Serial.print(' ');
 
 //printing time
 Serial.print(rtc.local_time.hour);    //hour
 Serial.print(':');
 Serial.print(rtc.local_time.minute);  //minute
 Serial.print(':');
 Serial.println(rtc.local_time.second);  //second 
}
#endif /* RTC_ALARM */


#ifdef SD_CARD
/*
  SD card datalogger

 This example shows how to log data from three analog sensors
 to an SD card using the SD library.

 The circuit:
 * analog sensors on analog ins 0, 1, and 2
 * SD card attached to SPI bus as follows:
 ** MOSI - pin 11
 ** MISO - pin 12
 ** CLK - pin 13
 ** CS - pin 4

 created  24 Nov 2010
 modified 9 Apr 2012
 by Tom Igoe

 This example code is in the public domain.

 */

#include <SPI.h>
#include <SD.h>

// On the Ethernet Shield, CS is pin 4. Note that even if it's not
// used as the CS pin, the hardware CS pin (10 on most Arduino boards,
// 53 on the Mega) must be left as an output or the SD library
// functions will not work.
const int chipSelect = 10;

void setup()
{
  // Open serial communications and wait for port to open:
  Serial.begin(115200);
  while (!Serial) {
    ; // wait for serial port to connect. Needed for Leonardo only
  }

  Serial.print("Initializing SD card...");
  // make sure that the default chip select pin is set to
  // output, even if you don't use it:
  // pinMode(10, OUTPUT);

  // see if the card is present and can be initialized:
  if (!SD.begin(chipSelect)) {
    Serial.println("Card failed, or not present");
    // don't do anything more:
    return;
  }
  Serial.println("card initialized.");

  if (SD.remove("datalog.txt")) {
    Serial.println("Delete datalog.txt");
  }    
}

void loop()
{
  // make a string for assembling the data to log:
  String dataString = "";

  // read three sensors and append to the string:
  for (int analogPin = 0; analogPin < 3; analogPin++) {
    int sensor = analogRead(analogPin);
    dataString += String(sensor);
    if (analogPin < 2) {
      dataString += ",";
    }
  }

  File dataFile = SD.open("datalog.txt");
  if (dataFile) {    
    Serial.println("=== data from file ===");
    while (dataFile.available()) {
      Serial.write(dataFile.read());
    }
    dataFile.close();
  }
    
  // open the file. note that only one file can be open at a time,
  // so you have to close this one before opening another.
  dataFile = SD.open("datalog.txt", FILE_WRITE);

  // if the file is available, write to it:
  if (dataFile) {
    dataFile.println(dataString);
    dataFile.close();
    // print to the serial port too:
    Serial.println("=== new data to file ===");      
    Serial.println(dataString);
  }
  // if the file isn't open, pop up an error:
  else {
    Serial.println("error opening datalog.txt");
  }
  delay(4000);  
}
#endif /* SD_CARD */


