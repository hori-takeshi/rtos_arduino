#include "r2ca.h"
#include "ESP8266.h"

#include "../examples_gdef.h"

#define WMODE_STATION

bool setup_done = false;

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

    setup_done = true;
}

#define MUX_NULL 0xff

uint8_t mux_id_ptn;

uint8_t mux_id = MUX_NULL;
uint8_t task1_mux_id = MUX_NULL;

void loop()
{
    uint8_t buffer[128] = {0};
    uint8_t pre_mux_id_ptn;    
    uint32_t len;
    uint32_t i;

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
    
    if((len = WiFi.recv(mux_id, buffer, sizeof(buffer))) == 0) {
        return;
    }

    /* Recived Data */

    for(i = 0; i < len; i++) {
        /* If Recive Ctrl-q(17) */
        if(buffer[i] == 17) {
            Serial.print("Echo Server : Close port : ");
            Serial.println(mux_id);            
            WiFi.releaseTCP(mux_id);
            mux_id = MUX_NULL;
            return;
        }
    }

    Serial.print("Echo Server : Recive Data from mux : ");
    Serial.println(mux_id);
    Serial.print("Echo Server : Recive len   : ");
    Serial.println(len);
    Serial.print("Echo Server : Recive Data  : ");
    Serial.println((char*)buffer);
    
    if(!WiFi.send(mux_id, buffer, len)) {
        Serial.println("Echo Server : send(mux_id, cmd) : Error!");
    }
}

#ifdef MULTI_ECHO_SERVER
/*
 *  For TASK1
 */
void loop1() {
    uint8_t buffer[128] = {0};
    uint32_t len;
    uint32_t i;

    delay(1);
    
    if (task1_mux_id == MUX_NULL) {
        delay(1);
        return;
    }
    
    if (!WiFi.isConnected(task1_mux_id)) {
        Serial.print("Echo Server Task1 : Port is closed: ");
        Serial.println(task1_mux_id);
        task1_mux_id = MUX_NULL;
        return;
    }
    if((len = WiFi.recv(task1_mux_id, buffer, sizeof(buffer))) == 0) {
        return;
    }        

    /* Recived Data */

    for(i = 0; i < len; i++) {
        /* If Recive Ctrl-q(17) */
        if(buffer[i] == 17) {
            Serial.print("Echo Server Task1 : Close port : ");
            Serial.println(task1_mux_id);            
            WiFi.releaseTCP(task1_mux_id);
            task1_mux_id = MUX_NULL;
            return;
        }
    }

    Serial.print("Echo Server Task1 : Recive Data from mux : ");
    Serial.println(task1_mux_id);
    Serial.print("Echo Server Task1 : Recive len   : ");
    Serial.println(len);
    Serial.print("Echo Server Task1 : Recive Data  : ");
    Serial.println((char*)buffer);
    
    if(!WiFi.send(task1_mux_id, buffer, len)) {
        Serial.println("Echo Server Task1 : send(task1_mux_id, cmd) : Error!");
    }
}
#endif /* MULTI_ECHO_SERVER */
