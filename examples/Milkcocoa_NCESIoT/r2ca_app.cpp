#include "r2ca.h"
#include "ESP8266.h"
#include "Milkcocoa.h"
#include "Client_ESP8266.h"

#include "../examples_gdef.h"

#define MILKCOCOA_SERVERPORT  1883

ESP8266Client wifi_client;

const char MQTT_SERVER[] PROGMEM    = MILKCOCOA_APP_ID ".mlkcca.com";
const char MQTT_CLIENTID[] PROGMEM  = __TIME__ MILKCOCOA_APP_ID;

Milkcocoa milkcocoa = Milkcocoa(&wifi_client, MQTT_SERVER, MILKCOCOA_SERVERPORT, MILKCOCOA_APP_ID, MQTT_CLIENTID);

#include <Servo.h> 
Servo myservo; 

#include "Ultrasonic.h"
Ultrasonic ultrasonic(3);

extern void onpush(DataElement *elem);

void setup()
{
    int ret;
    
    Serial.begin(115200);
    Serial.print("Milkcocoa NCES IoT demo");

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
    
    if (milkcocoa.on(MILKCOCOA_DATASTORE, "push", onpush)){
        Serial.println("milkcocoa on sucesss");   
    }
    else {
        Serial.println("milkcocoa on failure");   
    }

    myservo.attach(2);
    
    Serial.println("setup end\r\n");
}

void loop() {
    int8_t ret;
    int8_t push_ret;
    
    long RangeInCentimeters;
    RangeInCentimeters = ultrasonic.MeasureInCentimeters();
    Serial.print(RangeInCentimeters);//0~400cm
    Serial.println(" cm");
    
    DataElement elem = DataElement();
    elem.setValue("RangeInCentimeters", (int)RangeInCentimeters);
    
    do {
        push_ret = milkcocoa.push(MILKCOCOA_DATASTORE, &elem);
        if (push_ret != 0) {
            Serial.println("Milkcocoa.push() error.");
            Serial.println(milkcocoa.pushErrorString(push_ret));
            Serial.println(push_ret);
            Serial.println("Retrying MQTT push in 5 seconds...");
            delay(5000);
        }        
    }while(push_ret != 0);
    
    Serial.println("Push success.");
    delay(2000);
    
    if (Serial.available() > 0) {
        Serial.read();
        Serial.print("Pause  : Input any char to continue.\n\r");
        while(!(Serial.available() > 0));
        Serial.print("Resume.\n\r");
        Serial.read();
    }    
}

void onpush(DataElement *pelem) {
    char *data;
    int  pos;
    if(!pelem->getString("SERVO", &data)) {
        return;
    };
    
    pos = atoi(data);

    if ((pos >=  0) && (pos <= 180))  {
        myservo.write(pos);
        Serial.print("Ser Servo : ");
        Serial.println(pos);
    }
}


void loop1() {
    int8_t ret;
    while((ret = milkcocoa.loop(1)) != 0) {
        Serial.println("Milkcocoa.loop() connection error.");
        Serial.println(milkcocoa.connectErrorString(ret));
        Serial.println(ret);
        Serial.println("Retrying MQTT connection in 5 seconds...");
        delay(5000);
    }
    delay(100);
}
