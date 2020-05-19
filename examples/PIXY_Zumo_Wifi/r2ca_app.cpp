#include "r2ca.h"
#include "ESP8266.h"

#include <Wire.h>
#include <ZumoShield.h>

#include <SPI.h>  
#include <Pixy.h>

#include "../examples_gdef.h"

#define WMODE_STATION

uint16_t pixy_blocks = 0;
char     pixy_buf[16][128]; 

//==========================================================================
//
//  Pixy Pet Robot
//
//   Adafruit invests time and resources providing this open source code, 
//  please support Adafruit and open-source hardware by purchasing 
//  products from Adafruit!
//
// Written by: Bill Earl for Adafruit Industries
//
//==========================================================================
// begin license header
//
// All Pixy Pet source code is provided under the terms of the
// GNU General Public License v2 (http://www.gnu.org/licenses/gpl-2.0.html).
//
// end license header
//
//==========================================================================
//
// Portions of this code are derived from the Pixy CMUcam5 pantilt example code. 
//
//==========================================================================
 
#define X_CENTER    160L
#define Y_CENTER    100L
#define RCS_MIN_POS     0L
#define RCS_MAX_POS     1000L
#define RCS_CENTER_POS	((RCS_MAX_POS-RCS_MIN_POS)/2)
 
//---------------------------------------
// Servo Loop Class
// A Proportional/Derivative feedback
// loop for pan/tilt servo tracking of
// blocks.
// (Based on Pixy CMUcam5 example code)
//---------------------------------------
class ServoLoop
{
public:
	ServoLoop(int32_t proportionalGain, int32_t derivativeGain);
 
	void update(int32_t error);
 
	int32_t m_pos;
	int32_t m_prevError;
	int32_t m_proportionalGain;
	int32_t m_derivativeGain;
};
 
// ServoLoop Constructor
ServoLoop::ServoLoop(int32_t proportionalGain, int32_t derivativeGain)
{
	m_pos = RCS_CENTER_POS;
	m_proportionalGain = proportionalGain;
	m_derivativeGain = derivativeGain;
	m_prevError = 0x80000000L;
}
 
// ServoLoop Update 
// Calculates new output based on the measured
// error and the current state.
void ServoLoop::update(int32_t error)
{
	long int velocity;
	char buf[32];
	if (m_prevError!=0x80000000)
	{	
		velocity = (error*m_proportionalGain + (error - m_prevError)*m_derivativeGain)>>10;
 
		m_pos += velocity;
		if (m_pos>RCS_MAX_POS) 
		{
			m_pos = RCS_MAX_POS; 
		}
		else if (m_pos<RCS_MIN_POS) 
		{
			m_pos = RCS_MIN_POS;
		}
	}
	m_prevError = error;
}
// End Servo Loop Class
//---------------------------------------
 
Pixy pixy;  // Declare the camera object
 
ServoLoop panLoop(200, 200);  // Servo loop for pan
ServoLoop tiltLoop(150, 200); // Servo loop for tilt
 
void ScanForBlocks();
void FollowBlock(int trackedBlock);
int TrackBlock(int blockCount);
void task1_setup();

//---------------------------------------
// Setup - runs once at startup
//---------------------------------------
void setup()
{
	ZumoInit();
	buzzer.playOn();    
	Serial.begin(115200);
	Serial.print("Starting...\n");
	pixy.init();

    button.waitForPress();
    buzzer.playStart();

	task1_setup();
}
 
uint32_t lastBlockTime = 0;
 
//---------------------------------------
// Main loop - runs continuously after setup
//---------------------------------------
void loop()
{ 
	uint16_t blocks;
	blocks = pixy.getBlocks();
    
	// If we have blocks in sight, track and follow them
	if (blocks)
	{
		int trackedBlock = TrackBlock(blocks);
		FollowBlock(trackedBlock);
		lastBlockTime = millis();
        if(pixy_blocks == 0) {
            if (blocks > 16) {
                blocks = 16;
            }
            pixy_blocks = blocks;
            for (int j=0; j<blocks; j++){
                sprintf(pixy_buf[j], "sig: %d x: %d y: %d width: %d height: %d\n",
                        pixy.blocks[j].signature, pixy.blocks[j].x, pixy.blocks[j].y,
                        pixy.blocks[j].width, pixy.blocks[j].height);
            }
        }
        delay(1);
	}  
	else if (millis() - lastBlockTime > 100)
	{
		motors.setLeftSpeed(0);
		motors.setRightSpeed(0);
		ScanForBlocks();
	}
}
 
int oldX, oldY, oldSignature;
 
//---------------------------------------
// Track blocks via the Pixy pan/tilt mech
// (based in part on Pixy CMUcam5 pantilt example)
//---------------------------------------
int TrackBlock(int blockCount)
{
	int trackedBlock = 0;
	long maxSize = 0;
 
	Serial.print("blocks =");
	Serial.println(blockCount);
 
	for (int i = 0; i < blockCount; i++)
	{
		if ((oldSignature == 0) || (pixy.blocks[i].signature == oldSignature))
		{
			long newSize = pixy.blocks[i].height * pixy.blocks[i].width;
			if (newSize > maxSize)
			{
				trackedBlock = i;
				maxSize = newSize;
			}
		}
	}
 
	int32_t panError = X_CENTER - pixy.blocks[trackedBlock].x;
	int32_t tiltError = pixy.blocks[trackedBlock].y - Y_CENTER;
 
	panLoop.update(panError);
	tiltLoop.update(tiltError);
 
	pixy.setServos(panLoop.m_pos, tiltLoop.m_pos);
 
	oldX = pixy.blocks[trackedBlock].x;
	oldY = pixy.blocks[trackedBlock].y;
	oldSignature = pixy.blocks[trackedBlock].signature;
	return trackedBlock;
}
 
//---------------------------------------
// Follow blocks via the Zumo robot drive
//
// This code makes the robot base turn 
// and move to follow the pan/tilt tracking
// of the head.
//---------------------------------------
int32_t size = 400;
void FollowBlock(int trackedBlock)
{
	int32_t followError = RCS_CENTER_POS - panLoop.m_pos;  // How far off-center are we looking now?
 
	// Size is the area of the object.
	// We keep a running average of the last 8.
	size += pixy.blocks[trackedBlock].width * pixy.blocks[trackedBlock].height; 
	size -= size >> 3;
 
	// Forward speed decreases as we approach the object (size is larger)
	int forwardSpeed = constrain(400 - (size/256), -100, 400);  
 
	// Steering differential is proportional to the error times the forward speed
	int32_t differential = (followError + (followError * forwardSpeed))>>8;
 
	// Adjust the left and right speeds by the steering differential.
	int leftSpeed = constrain(forwardSpeed + differential, -400, 400);
	int rightSpeed = constrain(forwardSpeed - differential, -400, 400);
 
	// And set the motor speeds
	motors.setLeftSpeed(leftSpeed);
	motors.setRightSpeed(rightSpeed);
}
 
//---------------------------------------
// Random search for blocks
//
// This code pans back and forth at random
// until a block is detected
//---------------------------------------
int scanIncrement = (RCS_MAX_POS - RCS_MIN_POS) / 150;
uint32_t lastMove = 0;
 
void ScanForBlocks()
{
	if (millis() - lastMove > 20)
	{
		lastMove = millis();
		panLoop.m_pos += scanIncrement;
		if ((panLoop.m_pos >= RCS_MAX_POS)||(panLoop.m_pos <= RCS_MIN_POS))
		{
			tiltLoop.m_pos = random(RCS_MAX_POS * 0.6, RCS_MAX_POS);
			scanIncrement = -scanIncrement;
			if (scanIncrement < 0)
			{
				motors.setLeftSpeed(-250);
				motors.setRightSpeed(250);
			}
			else
			{
				motors.setLeftSpeed(+180);
				motors.setRightSpeed(-180);
			}
			delay(random(250, 500));
		}
 
		pixy.setServos(panLoop.m_pos, tiltLoop.m_pos);
	}
}


void task1_setup()
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
}

#define MUX_NULL 0xff

uint8_t mux_id_ptn;

uint8_t mux_id = MUX_NULL;
uint8_t task1_mux_id = MUX_NULL;

void loop1()
{
    char buf[128]; 
    uint8_t pre_mux_id_ptn;    

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
