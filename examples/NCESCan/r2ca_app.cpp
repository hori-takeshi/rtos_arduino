#include "r2ca.h"

// demo: CAN-BUS Shield, receive data
#include <SPI.h>
#include "mcp_can.h"
#include <stdio.h>
#define uint8_t unsigned char

const int SPI_CS_PIN = 9;
#define CAN_INT_PIN 3
MCP_CAN CAN(SPI_CS_PIN);                                    // Set CS pin

extern void MCP2515_ISR();

/*
 *  CAN Controller : MCP2515
 *  TXB : MCP_TXB0:0, MCP_RXB1:1,MCP_RXB2:2
 *  RXB : MCP_RXB0:0 : Mask MCP_RXM0:0 : Filer MCP_RXF0:0, MCP_RXF1:1
 *        MCP_RXB1:1 : Mask MCP_RXM2:1 : Filer MCP_RXF2:2, MCP_RXF3:3, MCP_RXF4:4, MCP_RXF5:5
 *  Interrupt mask
 *    MCP_TX0IF
 *    MCP_TX1IF
 *    MCP_TX2IF
 *    MCP_RX0IF
 *    MCP_RX1IF
 */

void setup()
{
    Serial.begin(115200);
    pinMode(13, OUTPUT);

    /*
     *  CAN Initialize
     */
    if(CAN.begin(CAN_500KBPS) ==CAN_OK) {
        Serial.print("can init ok!!\r\n");
    }
    else {
        while(1){
            Serial.print("Can init fail!!\r\n");
            digitalWrite(13, HIGH); 
            delay(500);             
            digitalWrite(13, LOW);  
            delay(500);             
        };
    }
    
    /*
     *  CAN Recive mabx Init
     */
    
    /*
     *  RXB0
     */
    CAN.init_Mask(MCP_RXM0, 0x0f);
    CAN.init_Filter(MCP_RXF0, 0x01);
    CAN.init_Filter(MCP_RXF1, 0x02);
    CAN.startReceive(MCP_RXB0);

    /*
     *  RXB1
     */
    CAN.init_Mask(MCP_RXM1, 0x0f);
    CAN.init_Filter(MCP_RXF2, 0x03);
    CAN.init_Filter(MCP_RXF3, 0x04);
    CAN.init_Filter(MCP_RXF4, 0x05);
    CAN.init_Filter(MCP_RXF5, 0x06);    
    CAN.startReceive(MCP_RXB1);

    CAN.enableInterrupt(MCP_RX0IF|MCP_RX1IF);
    CAN.attachInterrupt(CAN_INT_PIN, MCP2515_ISR);

}

uint8_t Flag_Recv = 0;

void MCP2515_ISR()
{
     Flag_Recv = 1;
}

void loop()
{
    uint8_t len = 0;
    uint8_t buf[8];
    uint8_t i;
    uint32_t id;    
    
    if(!Flag_Recv) {
       delay(1);
        return;
    }
      
    Flag_Recv = 0;

    for(i = 0; i < 2; i++){
        if(CAN.checkReceive(i) == CAN_NOMSG) {
            continue;
        }       
        CAN.readMsg(i, &id, &len, buf);    // read data,  len: data length, buf: data buf
        Serial.print("CAN_BUS GET DATA MBX");
        Serial.println(i);
        Serial.print("ID = ");Serial.print(id, HEX);
        Serial.print(", DLC = ");Serial.print(len);
        Serial.print(", data = ");
        for(int i = 0; i<len; i++){    // print the data          
              Serial.print(buf[i]);Serial.print("\t");
          }
        Serial.println();
    }
}

unsigned char stmp[8][8] = {
    {1, 2, 3, 4, 5, 6, 7, 8},
    {2, 3, 4, 5, 6, 7, 8, 1},
    {3, 4, 5, 6, 7, 8, 1, 2},
    {1, 2, 3, 4, 5, 6, 7, 8},
    {1, 2, 3, 4, 5, 6, 7, 8},
    {1, 2, 3, 4, 5, 6, 7, 8},
    {1, 2, 3, 4, 5, 6, 7, 8},
    {0, 1, 2, 3, 4, 5, 6, 7}
};

unsigned char stmp1[8] =     {0, 1, 2, 3, 4, 5, 6, 7};
    
void loop1()
{
    int i, e;

    for(e = 0;  e < 3; e++){
        Serial.print("Send can message from mbx ");
        Serial.println(e);
        for(i = 1; i <= 8; i++){
            while(CAN_SENDWAIT == CAN.checkSend(e)){delay(10);};
            /* mailbox, ID, DLC, DATA */
            CAN.sendMsg(e, i, i, stmp[i-1]);
        }
    }

    delay(5000);
}
