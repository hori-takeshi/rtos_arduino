#include "r2ca.h"
#include "ESP8266.h"
#include "Client_ESP8266.h"
#include "ThingSpeak.h"

#include "../examples_gdef.h"

ESP8266Client wifi_client;

void setup()
{
    int ret;
    
    Serial.begin(115200);
    Serial.print("Milkcocoa SDK demo");

    // Connect to WiFi access point.
    Serial.println(); Serial.println();
    Serial.print("Connecting to ");
    Serial.println(STA_SSID);

    ret = WiFi.begin(Serial5, 115200);

    if(ret == 1) {
        Serial.print("Cannot communicate with ESP8266.");
        while(1);        
    } else if(ret == 2) {
        Serial.println("FW Version mismatch.");
        Serial.print("FW Version:");
        Serial.println(WiFi.getVersion().c_str());
        Serial.print("Supported FW Version:");
        Serial.println(ESP8266_SUPPORT_VERSION);
        while(1);
    } else {
        Serial.print("begin ok\r\n");
    }

    Serial.print("FW Version:");
    Serial.println(WiFi.getVersion().c_str());
    
    if (WiFi.setOprToStation()) {
        Serial.print("to station ok\r\n");
    } else {
        Serial.print("to station err\r\n");
    }
    
    if (WiFi.joinAP(STA_SSID, STA_PASSWORD)) {
        Serial.print("Join AP success\r\n");
        Serial.print("IP: ");
        Serial.println(WiFi.getLocalIP().c_str());    
    } else {
        Serial.print("Join AP failure\r\n");
    }

    if (WiFi.stopServer()) {
        Serial.print("Stop server ok\r\n");
    } else {
        Serial.print("Stop server err\r\n");
    }        
    
    if (WiFi.disableMUX()) {
        Serial.print("single ok\r\n");
    } else {
        Serial.print("single err\r\n");
    }
    
    //Setup
    if (ThingSpeak.begin(wifi_client))  {
        Serial.print("ThingSpeak.begin() ok\r\n");
    } else {
        Serial.print("ThingSpeak.begin() err\r\n");
    }
    
    pinMode(13, OUTPUT);
    digitalWrite(13, LOW);   

    Serial.println("setup end\r\n");
}

unsigned long myChannelNumber =THINGSPEAK_CHANNELNUMBER ;
const char * myWriteAPIKey = THINGSPEAK_WRITEAPIKEY ;

int cnt1;
int cnt2;

#include <math.h>
int a;
float temperature;
int B=3975;                  //B value of the thermistor
float resistance;

void loop() {
    int ret;

    ThingSpeak.setField(1,cnt1);
    ThingSpeak.setField(2,cnt2);

    cnt1 += 1;
    cnt2 += 2;

    // Write the fields that you've set all at once.
    ret = ThingSpeak.writeFields(myChannelNumber, myWriteAPIKey);
    Serial.print("Call writeFields() : return code is ");
    Serial.println(ret);

    delay(20000);
}
