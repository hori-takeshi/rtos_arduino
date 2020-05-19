/* Connection
MASTER           SLAVE
MISO      ->     MOSI
MOSI      ->     MISO
SCK       ->     SCK
D10       ->     D10
GND       ->     GND
*/

#include <SPI.h>

#define SS 10
#define DIM 11

void setup() {

  pinMode (SS, OUTPUT);
  digitalWrite(SS, HIGH);
  Serial.begin(9600);
  //
  SPI.begin();
  
}

void loop() {
  String x ="hello world";
  digitalWrite(SS, LOW);
  for (int i=0;i<DIM;i++){
    SPI.transfer(x[i]);
    delay(1);
  } 
  digitalWrite(SS,HIGH);
  delay(1000);
}


