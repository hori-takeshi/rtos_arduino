#include "r2ca.h"

void task1_setup();

void setup(){
    Serial.begin(115200);
}

void loop(){
    Serial.print("idle     : ");
    Serial.println(r2ca_idle_result);
    Serial.print("isr       : ");
    Serial.println(r2ca_isr_result);
    Serial.print("dispatch  : ");
    Serial.println(r2ca_dispatch_result);        
    delay(1000);

    task1_setup();
}

#define PSERIAL SerialUSB

bool processing_connected = false;

void establishContact() {
    while (!PSERIAL.available()) {
        PSERIAL.print('A'); // send a capital A
        delay(300);
    }
    Serial.println("Processing Task : Connect!");
    processing_connected = true;
}

void task1_setup()
{
    PSERIAL.begin(115200);
    while(!PSERIAL){
        delay(1);
    }
    Serial.print("Processing Task : Start!");
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
    
    uint16_t load;
    uint16_t isr_cnt;
    uint16_t dispatch_cnt;

    load = 100 - map(r2ca_idle_result, 0, IDLE_TASK_IDLE_LOOP_10MS/10, 0, 100);
    isr_cnt = (r2ca_isr_result > 0xffff)? 0xffff : r2ca_isr_result;
    dispatch_cnt = (r2ca_dispatch_result > 0xffff)? 0xffff : r2ca_dispatch_result;
    
    if (PSERIAL.available()){
        inByte = PSERIAL.read();        
        last_connect = millis();
        
        PSERIAL.write((uint8_t)(load >> 8));
        PSERIAL.write((uint8_t)load);
        PSERIAL.write((uint8_t)(isr_cnt >> 8));                
        PSERIAL.write((uint8_t)isr_cnt);        
        PSERIAL.write((uint8_t)(dispatch_cnt >> 8));
        PSERIAL.write((uint8_t)dispatch_cnt);
    }
    delay(1);
}
