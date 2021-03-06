#include "r2ca.h"

/****************************************************************************
* Copyright (C) 2011 - 2014 Bosch Sensortec GmbH
*
* Accelerometer.ino
* Date: 2014/09/09
* Revision: 3.0 $
*
* Usage:        Example code to stream Accelerometer data
*
****************************************************************************
/***************************************************************************
* License:
*
* Redistribution and use in source and binary forms, with or without
* modification, are permitted provided that the following conditions are met:
*
*   Redistributions of source code must retain the above copyright
*   notice, this list of conditions and the following disclaimer.
*
*   Redistributions in binary form must reproduce the above copyright
*   notice, this list of conditions and the following disclaimer in the
*   documentation and/or other materials provided with the distribution.
*
*   Neither the name of the copyright holder nor the names of the
*   contributors may be used to endorse or promote products derived from
*   this software without specific prior written permission.
*
* THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS" AND
* ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED
* WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
* DISCLAIMED. IN NO EVENT SHALL COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE FOR ANY
* DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES
* (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
* LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND
* ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT
* (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS
* SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE
*
* The information provided is believed to be accurate and reliable.
* The copyright holder assumes no responsibility for the consequences of use
* of such information nor for any infringement of patents or
* other rights of third parties which may result from its use.
* No license is granted by implication or otherwise under any patent or
* patent rights of the copyright holder.
*/

#include "NAxisMotion.h"        //Contains the bridge code between the API and the Arduino Environment
#include <Wire.h>

NAxisMotion mySensor;                 //Object that for the sensor
unsigned long lastStreamTime = 0;     //To store the last streamed time stamp
const int streamPeriod = 20;          //To stream at 25Hz without using additional timers (time period(ms) =1000/frequency(Hz))
   
//bool updateSensorData = true;         //Flag to update the sensor data. Default is true to perform the first read before the first stream

void setup() //This code is executed once
{
  //Peripheral Initialization
  Serial.begin(115200);           //Initialize the Serial Port to view information on the Serial Monitor
  I2C.begin();                    //Initialize I2C communication to the let the library communicate with the sensor. 
  //Sensor Initialization
  mySensor.initSensor(0x29);       //The I2C Address can be changed here inside this function in the library
  mySensor.setOperationMode(OPERATION_MODE_NDOF);   //Can be configured to other operation modes as desired
  mySensor.setUpdateMode(MANUAL);	//The default is AUTO. Changing to manual requires calling the relevant update functions prior to calling the read functions

  Serial5.begin(115200);
  Serial.print("Start!");
}

float g_eula_h;
float g_eula_r;
float g_eula_p;

void loop() //This code is looped forever
{
      if ((millis() - lastStreamTime) >= streamPeriod)
  {
    lastStreamTime = millis();    
    mySensor.updateEuler();        //Update the Euler data into the structure of the object
    mySensor.updateCalibStatus();  //Update the Calibration Status

    g_eula_h = mySensor.readEulerHeading();
    g_eula_r = mySensor.readEulerRoll();
    g_eula_p = mySensor.readEulerPitch();
#ifdef PRINT_SENSOR      
    Serial.print("Time: ");
    Serial.print(lastStreamTime);
    Serial.print("ms ");

    Serial.print(" H: ");
    Serial.print(g_eula_h); //Heading(h) 0 to 360 data
    Serial.print("deg ");

    Serial.print(" R: ");
    Serial.print(g_eula_r); //Roll(y) 180 to -180 data
    Serial.print("deg");

    Serial.print(" P: ");
    Serial.print(g_eula_p); //Pitch(x) 180 to -180 data
    Serial.print("deg ");
    
    Serial.print(" A: ");
    Serial.print(mySensor.readAccelCalibStatus());  //Accelerometer Calibration Status (0 - 3)
	
    Serial.print(" M: ");
    Serial.print(mySensor.readMagCalibStatus());    //Magnetometer Calibration Status (0 - 3)
	
    Serial.print(" G: ");
    Serial.print(mySensor.readGyroCalibStatus());   //Gyroscope Calibration Status (0 - 3)
	
    Serial.print(" S: ");
    Serial.print(mySensor.readSystemCalibStatus());   //System Calibration Status (0 - 3)

    Serial.println();
#endif       
  }
    
#if 0    
  if (updateSensorData)  //Keep the updating of data as a separate task
  {
    mySensor.updateAccel();        //Update the Accelerometer data
    mySensor.updateLinearAccel();  //Update the Linear Acceleration data
    mySensor.updateGravAccel();    //Update the Gravity Acceleration data
    mySensor.updateCalibStatus();  //Update the Calibration Status
    updateSensorData = false;
  }

  if ((millis() - lastStreamTime) >= streamPeriod)
  {
    lastStreamTime = millis();
    Serial.print("Time: ");
    Serial.print(lastStreamTime);
    Serial.print("ms ");

    Serial.print("      aX: ");
    Serial.print(mySensor.readAccelX()); //Accelerometer X-Axis data
    Serial.print("m/s2 ");

    Serial.print(" aY: ");
    Serial.print(mySensor.readAccelY());  //Accelerometer Y-Axis data
    Serial.print("m/s2 ");

    Serial.print(" aZ: ");
    Serial.print(mySensor.readAccelZ());  //Accelerometer Z-Axis data
    Serial.print("m/s2 ");

    Serial.print("      lX: ");
    Serial.print(mySensor.readLinearAccelX()); //Linear Acceleration X-Axis data
    Serial.print("m/s2 ");

    Serial.print(" lY: ");
    Serial.print(mySensor.readLinearAccelY());  //Linear Acceleration Y-Axis data
    Serial.print("m/s2 ");

    Serial.print(" lZ: ");
    Serial.print(mySensor.readLinearAccelZ());  //Linear Acceleration Z-Axis data
    Serial.print("m/s2 ");

    Serial.print("      gX: ");
    Serial.print(mySensor.readGravAccelX()); //Gravity Acceleration X-Axis data
    Serial.print("m/s2 ");

    Serial.print(" gY: ");
    Serial.print(mySensor.readGravAccelY());  //Gravity Acceleration Y-Axis data
    Serial.print("m/s2 ");

    Serial.print(" gZ: ");
    Serial.print(mySensor.readGravAccelZ());  //Gravity Acceleration Z-Axis data
    Serial.print("m/s2 ");

    Serial.print("      C: ");
    Serial.print(mySensor.readAccelCalibStatus());  //Accelerometer Calibration Status (0 - 3)

    Serial.println();
    updateSensorData = true;
  }
#endif
    
  delay(1);
}

bool processing_connected = false;

void establishContact() {
    while (!Serial5.available()) {
        Serial5.print('A'); // send a capital A
        delay(300);
    }
    Serial.println("Processing Task : Connect!");
    processing_connected = true;
}

int inByte = 0;
int last_connect = 0;

#define TIMEOUT_MS  3000

void loop1()
{
    if(!processing_connected){
        establishContact();        
    }else{
        if((millis() - last_connect) > TIMEOUT_MS){
            processing_connected = false;
            Serial.println("Processing Task : Disconnect!");
        }              
    }
    
    int16_t roll = (int16_t)g_eula_r;
    int16_t pitch = (int16_t)g_eula_p;
    int16_t heading;

    if(g_eula_h <= 180){
        heading = (int16_t)g_eula_h;
    }
    else{
        heading = (int16_t)(g_eula_h - 360);
    }

    if (Serial5.available()){
        inByte = Serial5.read();
        last_connect = millis();
        
        Serial5.write((uint8_t)(heading >> 8));
        Serial5.write((uint8_t)heading);
        Serial5.write((uint8_t)(roll >> 8));                
        Serial5.write((uint8_t)roll);        
        Serial5.write((uint8_t)(pitch >> 8));
        Serial5.write((uint8_t)pitch);
    }
    delay(1);
}
