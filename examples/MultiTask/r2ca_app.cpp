#include "r2ca.h"
#define Serial SerialUSB  // Arduino M0 の場合

/*
 *  For MAINTASK
 */ 
void setup() {
    Serial.begin(115200);
    Serial.println("setup() : running ");

    pinMode(13, OUTPUT);
}

int loop_cnt = 0;

void loop() {
    Serial.print("loop() : running ");
    Serial.print(loop_cnt++);
    Serial.println(" .");
    delay(1000);
}

/*
 *  For TASK1
 */
void loop1() {
    digitalWrite(13, HIGH);
    delay(1000);           
    digitalWrite(13, LOW); 
    delay(1000);           
}

int task2_loop_cnt = 0;

/*
 *  For TASK2
 */
void loop2() {
    Serial.print("task2_loop() : running ");
    Serial.print(task2_loop_cnt++);
    Serial.println(" .");
    delay(2000);
}

int task3_loop_cnt = 0;

/*
 *  For TASK3
 */
void loop3() {
    Serial.print("task3_loop() : running ");
    Serial.print(task3_loop_cnt++);
    Serial.println(" .");
    delay(2000);
}

int task4_loop_cnt = 0;

/*
 *  For TASK4
 */
void loop4() {
    Serial.print("task4_loop() : running ");
    Serial.print(task4_loop_cnt++);
    Serial.println(" .");    
    delay(2000);
}

