#include "r2ca.h"
#include "ESP8266.h"

#include <SPI.h>  
#include <Pixy.h>

#include "../examples_gdef.h"

#define WMODE_STATION

extern void task1_setup();

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
        Serial.print(ESP8266_SUPPORT_VERSION);
        while(1);
    } else {
        Serial.print("begin ok\r\n");
    }

    Serial.print("FW Version:");
    Serial.println(WiFi.getVersion().c_str());
    
#ifdef  WMODE_STATION
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
        while(1);
    }
#else /* !WMODE_STATION */ 
    if (WiFi.setOprToSoftAP()) {
        Serial.print("to softap ok\r\n");
    } else {
        Serial.print("to softap err\r\n");
    }    
    if(WiFi.setSoftAPParam(AP_SSID, AP_PASSWORD, 7, 0)){
        Serial.print("Set SoftAP success\r\n");
        Serial.print("IP: ");
        Serial.println(WiFi.getLocalIP().c_str());            
    }
    else {
        Serial.print("Set SoftAP failure\r\n");
    }
#endif /* WMODE_STATION */
    
    if (WiFi.enableMUX()) {
        Serial.print("multiple ok\r\n");
    } else {
        Serial.print("multiple err\r\n");
        while(1);
    }

    if (WiFi.startTCPServer(80)) {
        Serial.print("start tcp server ok\r\n");
    } else {
        Serial.print("start tcp server err\r\n");
        while(1);
    }

    if (WiFi.setTCPServerTimeout(60)) { 
        Serial.print("set tcp server timout 60 seconds\r\n");
    } else {
        Serial.print("set tcp server timout err\r\n");
        while(1);
    }

    Serial.print("setup end\r\n");

    task1_setup();
}

#define MUX_NULL 0xff

uint8_t mux_id_ptn;

uint8_t mux_id = MUX_NULL;
uint8_t task1_mux_id = MUX_NULL;


uint16_t pixy_blocks = 0;
char     pixy_buf[16][128]; 

void loop()
{
    char buf[128]; 
    uint8_t pre_mux_id_ptn;    

    delay(1);    
    /* Check Connection Status */
    pre_mux_id_ptn = mux_id_ptn;

    if(!WiFi.getMuxCStatus(&mux_id_ptn)) {
        Serial.println("getMuxCStatus(&mux_id_ptn) : Error!");
    }
    else {        
        if (pre_mux_id_ptn != mux_id_ptn) {
            Serial.print("Connection Status changed! : 0x");
            Serial.println(mux_id_ptn, HEX);
            if (mux_id_ptn & 0x01) {
                mux_id = 0; 
            }
            if (mux_id_ptn & 0x02) {
                task1_mux_id = 1;
            }
        }
    }

    if (mux_id == MUX_NULL) {
        return;
    }
    
    if (!WiFi.isConnected(mux_id)) {
        Serial.print("Echo Server : Port is closed: ");
        Serial.println(mux_id);
        mux_id = MUX_NULL;
        return;
    }

    if(pixy_blocks != 0) {
        sprintf(buf, "Detected %d:\n", pixy_blocks);
        WiFi.send(mux_id, (uint8_t*)buf, strlen(buf));        
        for (int j=0; j<pixy_blocks; j++){
            WiFi.send(mux_id, (uint8_t*)pixy_buf[j], strlen(pixy_buf[j]));
        }        
        pixy_blocks = 0;
    }        
}

Pixy pixy;

#define X_CENTER        ((PIXY_MAX_X-PIXY_MIN_X)/2)       
#define Y_CENTER        ((PIXY_MAX_Y-PIXY_MIN_Y)/2)

class ServoLoop
{
public:
  ServoLoop(int32_t pgain, int32_t dgain);

  void update(int32_t error);
   
  int32_t m_pos;
  int32_t m_prevError;
  int32_t m_pgain;
  int32_t m_dgain;
};


ServoLoop panLoop(300, 500);
ServoLoop tiltLoop(500, 700);

ServoLoop::ServoLoop(int32_t pgain, int32_t dgain)
{
  m_pos = PIXY_RCS_CENTER_POS;
  m_pgain = pgain;
  m_dgain = dgain;
  m_prevError = 0x80000000L;
}

void ServoLoop::update(int32_t error)
{
  long int vel;
  char buf[32];
  if (m_prevError!=0x80000000)
  {	
    vel = (error*m_pgain + (error - m_prevError)*m_dgain)>>10;
    //sprintf(buf, "%ld\n", vel);
    //Serial.print(buf);
    m_pos += vel;
    if (m_pos>PIXY_RCS_MAX_POS) 
      m_pos = PIXY_RCS_MAX_POS; 
    else if (m_pos<PIXY_RCS_MIN_POS) 
      m_pos = PIXY_RCS_MIN_POS;
  }
  m_prevError = error;
}

void task1_setup()
{
    Serial.print("PIXY Starting...\n");  
    pixy.init();
    pixy.setServos(PIXY_RCS_CENTER_POS, PIXY_RCS_CENTER_POS);
}

void loop1()
{
    static int i = 0;
    int j;
    uint16_t blocks;
    char buf[128]; 
    int32_t panError, tiltError;
    
    blocks = pixy.getBlocks();
    
    if (blocks) {
        panError = X_CENTER-pixy.blocks[0].x;
        tiltError = pixy.blocks[0].y-Y_CENTER;
    
        panLoop.update(panError);
        tiltLoop.update(tiltError);

        pixy.setServos(panLoop.m_pos, tiltLoop.m_pos);
        i++;
        
        // do this (print) every 50 frames because printing every
        // frame would bog down the Arduino
        if (i%50==0) {
            sprintf(buf, "Detected %d:\n", blocks);
            Serial.print(buf);
            for (j=0; j<blocks; j++){
                sprintf(buf, "  block %d: ", j);
                Serial.print(buf);                
                pixy.blocks[j].print();
            }
            if(pixy_blocks == 0) {
                if (blocks > 16) {
                    blocks = 16;
                }
                pixy_blocks = blocks;
                for (j=0; j<blocks; j++){
                    sprintf(pixy_buf[j], "sig: %d x: %d y: %d width: %d height: %d\n",
                            pixy.blocks[j].signature, pixy.blocks[j].x, pixy.blocks[j].y,
                            pixy.blocks[j].width, pixy.blocks[j].height);
                }
            }
        }
    }
   
    delay(10);
}
