/*
The MIT License (MIT)

Copyright (c) 2015 Technical Rockstars
Copyright (C) 2015 Embedded and Real-Time Systems Laboratory
              Graduate School of Information Science, Nagoya Univ., JAPAN
Permission is hereby granted, free of charge, to any person obtaining a copy
of this software and associated documentation files (the "Software"), to deal
in the Software without restriction, including without limitation the rights
to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
copies of the Software, and to permit persons to whom the Software is
furnished to do so, subject to the following conditions:

The above copyright notice and this permission notice shall be included in all
copies or substantial portions of the Software.

THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
SOFTWARE.
*/
#include "Milkcocoa.h"
#include "r2ca.h"

DataElement::DataElement() {
  params = aJson.createObject();
  paJsonObj = aJson.createObject();
  aJson.addItemToObject(paJsonObj, "params", params);
}

DataElement::DataElement(char *json_string) {
  paJsonObj = aJson.parse(json_string);
  params = aJson.getObjectItem(paJsonObj, "params");
}

DataElement::~DataElement() {
  aJson.deleteItem(paJsonObj);
  paJsonObj = NULL;
  params = NULL;    
}

void DataElement::setValue(const char *key, const char *v) {
  aJson.addStringToObject(params, key, v);
}

void DataElement::setValue(const char *key, int v) {
  aJson.addNumberToObject(params, key, v);
}

void DataElement::setValue(const char *key, double v) {
  aJson.addNumberToObject(params, key, v);
}

bool DataElement::getString(const char *key, char **pdata) {
  aJsonObject* obj = aJson.getObjectItem(params, key);
  if((obj == NULL) || (obj->type != aJson_String)) {return false;}
  *pdata = obj->valuestring;    
  return true;
}

bool DataElement::getInt(const char *key, int *pdata) {
  aJsonObject* obj = aJson.getObjectItem(params, key);    
  if((obj == NULL) || (obj->type != aJson_Int)) {return false;}
  *pdata = obj->valueint;
  return true;
}

bool DataElement::getFloat(const char *key, float *pdata) {
  aJsonObject* obj = aJson.getObjectItem(params, key);
  if((obj == NULL) || (obj->type != aJson_Float)) {return false;}
  *pdata = obj->valuefloat;
  return true;
}

char *DataElement::toCharArray() {
  return aJson.print(paJsonObj);
}

Milkcocoa::Milkcocoa(Client *client, const char *host, uint16_t port, const char *_app_id, const char *client_id) {
  app_id = _app_id;
  mqtt = new Adafruit_MQTT_Client(client, host, port, client_id, "sdammy", app_id);

  for (uint8_t i=0; i<MILKCOCOA_SUBSCRIBERS; i++) {
    milkcocoaSubscribers[i] = 0;
  }

}

Milkcocoa::Milkcocoa(Client *client, const char *host, uint16_t port, const char *_app_id, const char *client_id, const char *_session) {
  sprintf(session, "%s", _session);
  app_id = _app_id;
  mqtt = new Adafruit_MQTT_Client(client, host, port, client_id, session, app_id);

  for (uint8_t i=0; i<MILKCOCOA_SUBSCRIBERS; i++) {
    milkcocoaSubscribers[i] = 0;
  }

}

Milkcocoa* Milkcocoa::createWithApiKey(Client *client, const char *host, uint16_t port, const char *app_id, const char *client_id, const char *key, const char *secret)
{
  char session[60];
  sprintf(session, "k%s:%s", key, secret);
  return new Milkcocoa(client, host, port, app_id, client_id, session);
}

/*
 *  tmout
 *   1-   : Connect try time.
 *   0    : retry Forever
 *  Return code
 *   0     : Success
 *   other : Error code (see Adafruit_MQTT::connectErrorString()) 
 */
int8_t Milkcocoa::connect(int32_t tmout) {
  int8_t ret;
  int32_t retry = 0;

  if (mqtt->connected()) {
    return 0;
  }

  MILKCOCOA_DEBUG_PRINTLN("Connecting to MQTT... ");
    
  wai_sem(MILKCOCOA_SEM);
  while ((ret = mqtt->connect()) != 0) { // connect will return 0 for connected
      mqtt->disconnect();
      // Timeout
      if ((tmout !=  0) && (++retry == tmout)) {
          sig_sem(MILKCOCOA_SEM);
          return ret;
      }
      delay(MILKCOCOA_CONNECT_DELAY_MS);
  }
  MILKCOCOA_DEBUG_PRINTLN("MQTT Connected!");
  sig_sem(MILKCOCOA_SEM);
    
  return 0;  
}

int8_t Milkcocoa::push(const char *path, DataElement *pdataelement, uint16_t timeout) {
  char topic[100];
  int8_t ret;
  char *send_array;

  if((ret = connect(timeout)) != 0){return ret;}

  wai_sem(MILKCOCOA_SEM);    
  sprintf(topic, "%s/%s/push", app_id, path);
  Adafruit_MQTT_Publish pushPublisher = Adafruit_MQTT_Publish(mqtt, topic);
  send_array = pdataelement->toCharArray();
  ret = (pushPublisher.publish(send_array))? 0 : MILKCOCOA_PUSH_ERRORNO;
  free(send_array);
  sig_sem(MILKCOCOA_SEM);
    
  return ret;
}

int8_t Milkcocoa::send(const char *path, DataElement *pdataelement, uint16_t timeout) {
  char topic[100];
  int8_t ret;
  char *send_array;

  if((ret = connect(timeout)) != 0){return ret;}

  wai_sem(MILKCOCOA_SEM);        
  sprintf(topic, "%s/%s/send", app_id, path);
  Adafruit_MQTT_Publish pushPublisher = Adafruit_MQTT_Publish(mqtt, topic);
  send_array = pdataelement->toCharArray();
  ret = (pushPublisher.publish(send_array))? 0 : MILKCOCOA_PUSH_ERRORNO;
  free(send_array);
  sig_sem(MILKCOCOA_SEM);
    
  return ret;
}

/*
 *  Return code
 *   0     : Success
 *   other : Error code
 *           -1,1-6 : see Adafruit_MQTT::connectErrorString()
 */
int8_t Milkcocoa::loop(uint16_t timeout) {
  int8_t ret;

  if((ret = connect(timeout)) != 0){return ret;}

  wai_sem(MILKCOCOA_SEM);    
  Adafruit_MQTT_Subscribe *subscription;
  while ((subscription = mqtt->readSubscription(1000))) {
    for (uint8_t i=0; i<MILKCOCOA_SUBSCRIBERS; i++) {
      if(milkcocoaSubscribers[i] == 0) continue;
      if (subscription == (milkcocoaSubscribers[i]->mqtt_sub) ) {
          DataElement de = DataElement((char *)milkcocoaSubscribers[i]->mqtt_sub->lastread);
          milkcocoaSubscribers[i]->cb( &de );
      }
    }
  }
  sig_sem(MILKCOCOA_SEM);
    
  return 0;  
}

bool Milkcocoa::on(const char *path, const char *event, GeneralFunction cb) {
  MilkcocoaSubscriber *sub = new MilkcocoaSubscriber(cb);
  sprintf(sub->topic, "%s/%s/%s", app_id, path, event);

  uint8_t i;
  Adafruit_MQTT_Subscribe *mqtt_sub = new Adafruit_MQTT_Subscribe(mqtt, sub->topic);
  sub->set_mqtt_sub(mqtt_sub);
  if(!mqtt->subscribe(mqtt_sub)) {
    return false;
  }
  for (i=0; i<MILKCOCOA_SUBSCRIBERS; i++) {
    if (milkcocoaSubscribers[i] == sub) {
      return false;
    }
  }
  for (i=0; i<MILKCOCOA_SUBSCRIBERS; i++) {
    if (milkcocoaSubscribers[i] == 0) {
      milkcocoaSubscribers[i] = sub;
      return true;
    }
  }
  return false;
}

MilkcocoaSubscriber::MilkcocoaSubscriber(GeneralFunction _cb) {
  cb = _cb;
}

void MilkcocoaSubscriber::set_mqtt_sub(Adafruit_MQTT_Subscribe *_mqtt_sub) {
  mqtt_sub = _mqtt_sub;
}


