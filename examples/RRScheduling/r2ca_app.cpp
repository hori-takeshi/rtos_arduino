#include "r2ca.h"


void
loop_wait(uint32_t num){
    volatile uint32_t loop;
    for(loop = 0; loop < (num*100); loop++){
        Asm("nop");
    }
}

/*
 *  For MAINTASK
 */ 
void setup() {
    Serial.begin(115200);
    Serial.println("setup() : running");
}

void loop() {
    dis_dsp();
    Serial.println("loop() : running");
    ena_dsp();
    loop_wait(10000);
}

/*
 *  For TASK1
 */
void loop1() {
    dis_dsp();
    Serial.println("task1loop() : running");
    ena_dsp();
    digitalWrite(13, HIGH);
    loop_wait(10000);
    dis_dsp();
    Serial.println("task1loop() : running");
    ena_dsp();
    digitalWrite(13, LOW); 
    loop_wait(10000);
}

/*
 *  For TASK2
 */
void loop2() {
    dis_dsp();
    Serial.println("task2loop() : running");
    ena_dsp();
    loop_wait(10000);
}

/*
 *  For TASK3
 */
void loop3() {
    dis_dsp();
    Serial.println("task3loop() : running");
    ena_dsp();
    loop_wait(20000);
}

/*
 *  For TASK4
 */
void loop4() {
    dis_dsp();
    Serial.println("task4loop() : running");
    ena_dsp();
    loop_wait(40000);
}

