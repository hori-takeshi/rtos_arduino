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
        
uint8_t data[11];
  
void setup() {

  SerialUSB.begin(9600);
  pinMode (SS, INPUT);
  while(!SerialUSB); 
   SPI.beginSlave(); 
}

void loop() {
  while(digitalRead(SS)==LOW){
    for (int i=0;i<DIM;i++){
      data[i] = SPI.read();
      SerialUSB.print((char)data[i]);
   }
    SerialUSB.println("");
  }    
}



