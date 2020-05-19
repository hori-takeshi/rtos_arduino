#include "r2ca.h"
#include "ESP8266.h"
#include "Client_ESP8266.h"
#include <PubSubClient.h>
#include <ArduinoJson.h>

#include "../examples_gdef.h"

char serverName[] = BLUEMIX_ORGANIZATION ".messaging.internetofthings.ibmcloud.com";
String clientName = String("d:") + String(BLUEMIX_ORGANIZATION) + String(":") + String(BLUEMIX_DEVICE_TYPE) + String(":") + String(BLUEMIX_DEVICE_ID);
String topicName = String("iot-2/evt/status/fmt/json");
char username[] = "use-token-auth";
char password[] = BLUEMIX_PASSWORD;

ESP8266Client wifi_client;
PubSubClient client(serverName, 1883, 0, wifi_client);

void MqttConnect()
{
  while(!client.connected())
  {
    char clientStr[128];
    clientName.toCharArray(clientStr,sizeof(clientStr));

    Serial.print("Trying to connect to: ");
    Serial.println(clientStr);
    client.connect(clientStr,username,password);
  }
}

void MqttRequest(int level)
{
  MqttConnect();

  if(client.connected())
  {
    char topicStr[128];
    topicName.toCharArray(topicStr,sizeof(topicStr));
    char jsonStr[200];
    DynamicJsonBuffer jsonBuffer;
    JsonObject& root = jsonBuffer.createObject();
    JsonObject& d = root.createNestedObject("d");
    d["cds"] = level;
    root.printTo(jsonStr, sizeof(jsonStr));

    boolean pubresult = client.publish(topicStr,jsonStr);

    Serial.print("attempt to send ");
    Serial.println(jsonStr);
    Serial.print("to ");
    Serial.println(topicStr);

    if(pubresult)
      Serial.println("successfully sent");
    else
      Serial.println("unsuccessfully sent");
  }
  else
  {
      Serial.println("connect error");
  }
}

void setup()
{
    int ret;
    
    Serial.println("Echo Server : Start!");
    
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

    MqttConnect();
    Serial.println("MQTT set up");
}

int cnt = 0;
void loop()
{    
    MqttRequest(cnt++);
    delay(1000);
    client.loop();

    if (Serial.available() > 0) {
        Serial.read();
        Serial.print("Pause  : Input any char to continue.\n\r");
        while(!(Serial.available() > 0));
        Serial.print("Resume.\n\r");
        Serial.read();
    }
}
