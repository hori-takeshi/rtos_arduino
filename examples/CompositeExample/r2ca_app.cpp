#include "r2ca.h"
#include "i2c_lcd.h"

#define LED1_PORT 2
#define LED2_PORT 3

#define SSID       "XXXXX"
#define PASSWORD   "YYYYY"

#define AP_SSID       "M0_AP"
#define AP_PASSWORD   "none"

//#define Serial SerialUSB

/////////////////////////////////////////////////////////////////////
//                    Inter Task Variable
/////////////////////////////////////////////////////////////////////
#define LED_MODE_ONOFF 0
#define LED_MODE_BLINK 1
#define LED_BLINK_CYC_INIT 500

bool gled1_state = false;
bool gled2_state = false;
uint8_t led_mode = LED_MODE_ONOFF;
uint32_t led_blink_cyc = LED_BLINK_CYC_INIT;

/////////////////////////////////////////////////////////////////////
//                    Command  Task 
/////////////////////////////////////////////////////////////////////
const char *usage = " \
1 : Select Sensor Task.\n\
2 : Select Web Task. \n\
3 : Select LED Task. \n\
4 : Select TFT Task. \n\
5 : Select Processing Task. \n\
";

extern void task1_setup();
extern void task2_setup();
extern void task3_setup();
extern void task4_setup();
extern void task5_setup();

void setup(){
    Serial.begin(115200);    
    while(!Serial){
        delay(1);
    }
    Serial.println("Main Task : setup start!");
    Serial.println(usage);

    task1_setup();
    task2_setup();
    task3_setup();
    task4_setup();
    task5_setup();
}


/*
 *  サービスコールのエラーのログ出力
 */
Inline void
svc_perror(const char *file, int_t line, const char *expr, ER ercd)
{
	if (ercd < 0) {
		t_perror(LOG_ERROR, file, line, expr, ercd);
	}
}

#define	SVC_PERROR(expr)	svc_perror(__FILE__, __LINE__, #expr, (expr))

ID		tskid = R2CA_TASK1;
int_t	tskno = 1;

#define HIGH_PRIORITY  4
#define MID_PRIORITY   5
#define LOW_PRIORITY   6

void loop(){
    char c;
    ER_UINT	ercd;
    PRI		tskpri;
    SYSUTM	utime1, utime2;
    
    if (Serial.available()){
        c = Serial.read();
        switch (c) {
        case '1':
            tskno = 1;
            tskid = R2CA_TASK1;
            syslog(LOG_INFO, "Select Sensor Task.");
            break;
        case '2':
            tskno = 2;
            tskid = R2CA_TASK2;
            syslog(LOG_INFO, "Select Web Task.");
            break;
        case '3':
            tskno = 3;
            tskid = R2CA_TASK3;
            syslog(LOG_INFO, "Select LED Task.");
            break;
        case '4':
            tskno = 4;
            tskid = R2CA_TASK4;
            syslog(LOG_INFO, "Select TFT Task.");
            break;
        case '5':
            tskno = 5;
            tskid = R2CA_TASK5;
            syslog(LOG_INFO, "Select Processing Task.");
            break;
        case 'a':
            syslog(LOG_INFO, "#act_tsk(%d)", tskno);
            SVC_PERROR(act_tsk(tskid));
            break;
        case 'A':
            syslog(LOG_INFO, "#can_act(%d)", tskno);
            SVC_PERROR(ercd = can_act(tskid));
            if (ercd >= 0) {
                syslog(LOG_NOTICE, "can_act(%d) returns %d", tskno, ercd);
            }
            break;
        case 't':
            syslog(LOG_INFO, "#ter_tsk(%d)", tskno);
            SVC_PERROR(ter_tsk(tskid));
            break;
        case '>':
            syslog(LOG_INFO, "#chg_pri(%d, HIGH_PRIORITY)", tskno);
            SVC_PERROR(chg_pri(tskid, HIGH_PRIORITY));
            break;
        case '=':
            syslog(LOG_INFO, "#chg_pri(%d, MID_PRIORITY)", tskno);
            SVC_PERROR(chg_pri(tskid, MID_PRIORITY));
            break;
        case '<':
            syslog(LOG_INFO, "#chg_pri(%d, LOW_PRIORITY)", tskno);
            SVC_PERROR(chg_pri(tskid, LOW_PRIORITY));
            break;
        case 'G':
            syslog(LOG_INFO, "#get_pri(%d, &tskpri)", tskno);
            SVC_PERROR(ercd = get_pri(tskid, &tskpri));
            if (ercd >= 0) {
                syslog(LOG_NOTICE, "priority of task %d is %d", tskno, tskpri);
            }
            break;
        case 'w':
            syslog(LOG_INFO, "#wup_tsk(%d)", tskno);
            SVC_PERROR(wup_tsk(tskid));
            break;
        case 'W':
            syslog(LOG_INFO, "#can_wup(%d)", tskno);
            SVC_PERROR(ercd = can_wup(tskid));
            if (ercd >= 0) {
                syslog(LOG_NOTICE, "can_wup(%d) returns %d", tskno, ercd);
            }
            break;
        case 'l':
            syslog(LOG_INFO, "#rel_wai(%d)", tskno);
            SVC_PERROR(rel_wai(tskid));
            break;
        case 'u':
            syslog(LOG_INFO, "#sus_tsk(%d)", tskno);
            SVC_PERROR(sus_tsk(tskid));
            break;
        case 'm':
            syslog(LOG_INFO, "#rsm_tsk(%d)", tskno);
            SVC_PERROR(rsm_tsk(tskid));
            break;
        case 'x':
            syslog(LOG_INFO, "#ras_tex(%d, 0x0001U)", tskno);
            SVC_PERROR(ras_tex(tskid, 0x0001U));
            break;
        case 'X':
            syslog(LOG_INFO, "#ras_tex(%d, 0x0002U)", tskno);
            SVC_PERROR(ras_tex(tskid, 0x0002U));
            break;
        case 'r':
            syslog(LOG_INFO, "#rot_rdq(three priorities)");
            SVC_PERROR(rot_rdq(HIGH_PRIORITY));
            SVC_PERROR(rot_rdq(MID_PRIORITY));
            SVC_PERROR(rot_rdq(LOW_PRIORITY));
            break;
        case 'V':
            SVC_PERROR(get_utm(&utime1));
            SVC_PERROR(get_utm(&utime2));
            syslog(LOG_NOTICE, "utime1 = %ld, utime2 = %ld",
                                        (ulong_t) utime1, (ulong_t) utime2);
            break;
        }
    }
    delay(10);
}

/////////////////////////////////////////////////////////////////////
//
//                       Sensor Task
//
/////////////////////////////////////////////////////////////////////
//#define SENSOR_DEBUG_PRINT

float tmp007_objt;
float tmp007_diet;
float tsl2591_light;
uint16_t vcnl4000_proximity;

#include <Wire.h>
#include <Adafruit_Sensor.h>

#define USE_TMP007
#define USE_TSL2591
#define USE_VCNL4000
#define USE_I2CLCD

#ifdef USE_TMP007
#include "Adafruit_TMP007.h"
Adafruit_TMP007 tmp007;
#endif /*  USE_TMP007 */

#ifdef USE_TSL2591
#include "Adafruit_TSL2591.h"
Adafruit_TSL2591 tsl2591(2591);
#endif /* USE_TSL2591 */

#ifdef USE_VCNL4000
#include "Adafruit_VCNL4000.h"
Adafruit_VCNL4000 vcnl4000;
#endif /* USE_VCNL4000 */

#define SENSOR_DELAY_MS     10

#define TMP007_CYCLE_MS   1000
uint32_t tmp007_cycle;

#define TSL2591_CYCLE_MS  100
uint32_t tsl2591_cycle;

#define VCNL4000_CYCLE_MS 100
uint32_t vcnl4000_cycle;

#define I2CLCD_CYCLE_MS 100
uint32_t i2clcd_cycle;

uint32_t led_cycle;

#ifdef USE_TSL2591
/**************************************************************************/
/*
    Displays some basic information on this sensor from the unified
    sensor API sensor_t type (see Adafruit_Sensor for more information)
*/
/**************************************************************************/
void tsl2591_displaySensorDetails(void)
{
  sensor_t sensor;
  tsl2591.getSensor(&sensor);
  Serial.println("------------------------------------");
  Serial.print  ("Sensor:       "); Serial.println(sensor.name);
  Serial.print  ("Driver Ver:   "); Serial.println(sensor.version);
  Serial.print  ("Unique ID:    "); Serial.println(sensor.sensor_id);
  Serial.print  ("Max Value:    "); Serial.print(sensor.max_value); Serial.println(" lux");
  Serial.print  ("Min Value:    "); Serial.print(sensor.min_value); Serial.println(" lux");
  Serial.print  ("Resolution:   "); Serial.print(sensor.resolution); Serial.println(" lux");  
  Serial.println("------------------------------------");
  Serial.println("");
  delay(500);
}

/**************************************************************************/
/*
    Configures the gain and integration time for the TSL2591
*/
/**************************************************************************/
void tsl2591_configureSensor(void)
{
  // You can change the gain on the fly, to adapt to brighter/dimmer light situations
  //tsl.setGain(TSL2591_GAIN_LOW);    // 1x gain (bright light)
  tsl2591.setGain(TSL2591_GAIN_MED);      // 25x gain
  //tsl.setGain(TSL2591_GAIN_HIGH);   // 428x gain
  
  // Changing the integration time gives you a longer time over which to sense light
  // longer timelines are slower, but are good in very low light situtations!
  tsl2591.setTiming(TSL2591_INTEGRATIONTIME_100MS);  // shortest integration time (bright light)
  //tsl.setTiming(TSL2591_INTEGRATIONTIME_200MS);
  //tsl.setTiming(TSL2591_INTEGRATIONTIME_300MS);
  //tsl.setTiming(TSL2591_INTEGRATIONTIME_400MS);
  //tsl.setTiming(TSL2591_INTEGRATIONTIME_500MS);
  //tsl.setTiming(TSL2591_INTEGRATIONTIME_600MS);  // longest integration time (dim light)

  /* Display the gain and integration time for reference sake */  
  Serial.println("------------------------------------");
  Serial.print  ("Gain:         ");
  tsl2591Gain_t gain = tsl2591.getGain();
  switch(gain)
  {
    case TSL2591_GAIN_LOW:
      Serial.println("1x (Low)");
      break;
    case TSL2591_GAIN_MED:
      Serial.println("25x (Medium)");
      break;
    case TSL2591_GAIN_HIGH:
      Serial.println("428x (High)");
      break;
    case TSL2591_GAIN_MAX:
      Serial.println("9876x (Max)");
      break;
  }
  Serial.print  ("Timing:       ");
  Serial.print((tsl2591.getTiming() + 1) * 100, DEC); 
  Serial.println(" ms");
  Serial.println("------------------------------------");
  Serial.println("");
}
#endif /* USE_TSL2591 */

void task1_setup() {
    Serial.println("Sensor Task : setup start!");

#ifdef USE_TMP007    
    if (tmp007.begin()) {
        Serial.println("Found a TMP007 sensor.");
    } else {
        Serial.println("No TMP007 found!");
        while (1);
    }
    tmp007_cycle = 0;
#endif /*  USE_TMP007 */
    
#ifdef USE_TSL2591
    if (tsl2591.begin()) {
        Serial.println("Found a TSL2591 sensor");
    } else {
        Serial.println("No TSL2591 found.");
        while (1);
    }
    
    /* Display some basic information on this sensor */
    tsl2591_displaySensorDetails();
  
    /* Configure the sensor */
    tsl2591_configureSensor();
#endif /* USE_TSL2591 */

#ifdef USE_VCNL4000
    if (vcnl4000.begin()) {
        Serial.println("Found a VCNL4000 sensor");
    } else {
        Serial.println("No VCNL4000 found.");
        while (1);
    }
#endif /* USE_VCNL4000 */

#ifdef USE_I2CLCD    
    i2clcd_begin();
#endif /* USE_I2CLCD */    
}

void loop1() {
    static uint32_t sensor_cyc_tim = 0;

    if ((millis() - sensor_cyc_tim) < SENSOR_DELAY_MS){
        delay(1);
        return;
    }else{
        sensor_cyc_tim = millis();
    }
    
#ifdef USE_TMP007
    /* TMP007 */
    if((++tmp007_cycle) == (TMP007_CYCLE_MS/SENSOR_DELAY_MS)) {
        tmp007_cycle = 0;
        tmp007_objt = tmp007.readObjTempC();
        tmp007_diet = tmp007.readDieTempC();
#ifdef SENSOR_DEBUG_PRINT        
        Serial.print("Object Temperature: "); Serial.print(tmp007_objt); Serial.println("*C");
        Serial.print("Die Temperature:    "); Serial.print(tmp007_diet); Serial.println("*C");
#endif /* SENSOR_DEBUG_PRINT */        
    }
#endif /*  USE_TMP007 */
    
#ifdef USE_TSL2591
    /* TSL2591 */
    if((++tsl2591_cycle) == (TSL2591_CYCLE_MS/SENSOR_DELAY_MS)) {
        tsl2591_cycle = 0;
        /* Get a new sensor event */ 
        sensors_event_t event;
        tsl2591.getEvent(&event);
        tsl2591_light = event.light;
#ifdef SENSOR_DEBUG_PRINT
        /* Display the results (light is measured in lux) */
        if ((event.light == 0) |
            (event.light > 4294966000.0) | 
            (event.light <-4294966000.0)) {
            /* If event.light = 0 lux the sensor is probably saturated */
            /* and no reliable data could be generated! */
            /* if event.light is +/- 4294967040 there was a float over/underflow */
            Serial.println("Invalid data (adjust gain or timing)");
        } else {
            Serial.print(event.light); Serial.println(" lux");
        }
#endif /* SENSOR_DEBUG_PRINT */
    }
#endif /* USE_TSL2591 */
    
#ifdef USE_VCNL4000    
    /* VCNL4000 */
    if((++vcnl4000_cycle) == (VCNL4000_CYCLE_MS/SENSOR_DELAY_MS)) {
        vcnl4000_cycle = 0;
        vcnl4000_proximity = vcnl4000.readProximity();
#ifdef SENSOR_DEBUG_PRINT        
        Serial.print("Ambient: "); Serial.println(vcnl4000.readAmbient());
        Serial.print("Proimity: "); Serial.println(vcnl4000_proximity);
#endif /* SENSOR_DEBUG_PRINT */        
    }
#endif /* USE_VCNL4000 */

#ifdef USE_I2CLCD    
    if((++i2clcd_cycle) == (I2CLCD_CYCLE_MS/SENSOR_DELAY_MS)) {
        i2clcd_cycle = 0;
        i2clcd_clear();
        i2clcd_setCursor(0, 0);
        i2clcd_printfloat(tmp007_objt);
        i2clcd_printStr("*C");
        i2clcd_setCursor(0, 1);
        i2clcd_printInt(tsl2591_light);
        i2clcd_printStr("lux");
    }
#endif /* USE_I2CLCD */    
}

/////////////////////////////////////////////////////////////////////
//
//                       Web Server Task
//
/////////////////////////////////////////////////////////////////////
#include "ESP8266.h"

ESP8266 wifi;

void task2_setup()
{
    Serial.println("Web Server Task : Start!");
    
    wifi.begin(Serial5, 115200);
    
    Serial.print("FW Version:");
    Serial.println(wifi.getVersion().c_str());
    
#if 0
    if (wifi.setOprToStation()) {
        Serial.print("to station ok\r\n");
    } else {
        Serial.print("to station err\r\n");
    }
    
    if (wifi.joinAP(SSID, PASSWORD)) {
        Serial.print("Join AP success\r\n");
        Serial.print("IP: ");
        Serial.println(wifi.getLocalIP().c_str());    
    } else {
        Serial.print("Join AP failure\r\n");
    }
#else
    if (wifi.setOprToSoftAP()) {
        Serial.print("to softap ok\r\n");
    } else {
        Serial.print("to softap err\r\n");
    }
    
    if(wifi.setSoftAPParam(AP_SSID, AP_PASSWORD, 7, 0)){
        Serial.print("Set SoftAP success\r\n");
        Serial.print("IP: ");
        Serial.println(wifi.getLocalIP().c_str());            
    }
    else {
        Serial.print("Set SoftAP failure\r\n");
    }
#endif
    if (wifi.enableMUX()) {
        Serial.print("multiple ok\r\n");
    } else {
        Serial.print("multiple err\r\n");
    }
    
    if (wifi.startTCPServer(80)) {
        Serial.print("start tcp server ok\r\n");
    } else {
        Serial.print("start tcp server err\r\n");
    }
    
    if (wifi.setTCPServerTimeout(10)) { 
        Serial.print("set tcp server timout 10 seconds\r\n");
    } else {
        Serial.print("set tcp server timout err\r\n");
    }
    
    Serial.print("setup end\r\n");
}


const char *web_led_control = " \
HTTP/1.0 200 OK \r\n\
Content-Type: text/html \r\n\
\r\n\
<html> \r\n\
<head> \r\n\
<script language=javascript> \r\n\
function connecttext( textid, ischecked ) { \r\n\
   document.getElementById(textid).disabled = !ischecked; \r\n\
}  \r\n\
</script> \r\n\
</head> \r\n\
<body><font size=\"7\"> \r\n\
<form method=get> \r\n\
 \r\n\
<p> \r\n\
<h3>LED</h3> \r\n\
<input type=\"checkbox\" name=\"LED2_ONOFF\" value=\"on\" onChange=\"this.form.submit()\" LED2_ISCHECKED /> LED2 \r\n\
<input type=\"checkbox\" name=\"LED1_ONOFF\" value=\"on\" onChange=\"this.form.submit()\" LED1_ISCHECKED /> LED1 \r\n\
</p> \r\n\
 \r\n\
<p> \r\n\
<h3>Mode</h3> \r\n\
OnOff<INPUT type=\"radio\" name=\"led_mode\" value=\"onoff\" onclick=\"connecttext('textforscb3', false);\" onChange=\"this.form.submit()\" LED_MODE_ONOFF > \r\n\
Blink<INPUT type=\"radio\" name=\"led_mode\" value=\"blink\" onclick=\"connecttext('textforscb3', true);\" onChange=\"this.form.submit()\" LED_MODE_BLINK > \r\n\
</p> \r\n\
 \r\n\
<p> \r\n\
<INPUT style=\"font-size:24px;\" type=\"text\" name=\"led_cycle\" id=\"textforscb3\" size=\"10\" value=\"LED_BLINK_CYC\" LED_BLINK_CYC_DISABLE> ms \r\n\
<input style=\"font-size:24px;\" type=\"submit\" value=\"Update\"> \r\n\
</p> \r\n\
</form> \r\n\
</body> \r\n\
</html> \r\n\
";


const char *web_sensormonitor = " \
HTTP/1.1 200 OK\r\n\
Content-Type: text/html\r\n\
Connection: close\r\n\
Refresh: 2\r\n\
\r\n\
<!DOCTYPE HTML>\r\n\
<html>\r\n \
<body><font size=\"7\"> \r\n\
<p>Die Temperature      : DIE_TEMP_I.DIE_TEMP_D *C</p>\r\n\
<p>Object Temperature   : OBJ_TEMP_I.OBJ_TEMP_D *C</p>\r\n\
<p>Illuminance          : TSL2591_LIGHT_I.TSL2591_LIGHT_D Lux</p>\r\n\
<p>Proximity            : VCNL4000_PROXIMITY</p>\r\n\
</body></html> ";

const char *web_top = " \
HTTP/1.0 200 OK \r\n\
Content-Type: text/html \r\n\
\r\n\
<html> \r\n\
<body><font size=\"7\"> \r\n\
<ul> \r\n\
<li><a href=\"./ledcontrol\">LED Control</a><br></li> \r\n\
<li><a href=\"./sensormonitor\">Sensor Monitor</a><br></li> \r\n\
</ul> \r\n\
</body></html> ";


void loop2()
{
    uint8_t buffer[128] = {0};
    uint8_t mux_id;
    uint32_t len = wifi.recv(&mux_id, buffer, sizeof(buffer));
    String cmd;
    
    if(len == 0) {
        return;
    }
    
    Serial.println("Web Server Task : Recive Request");
    String buf_string((char*)buffer);
//    Serial.println(buf_string);

    if((buf_string.indexOf("/ledcontrol") != -1)) {    
        gled1_state = (buf_string.indexOf("LED1_ONOFF=on") != -1);
        gled2_state = (buf_string.indexOf("LED2_ONOFF=on") != -1);
        led_mode = (buf_string.indexOf("led_mode=blink") != -1)? LED_MODE_BLINK : LED_MODE_ONOFF;
        int index = buf_string.indexOf("led_cycle=");
        if(index != -1){
            index += sizeof("led_cycle=") - 1;
            int last_index = buf_string.indexOf(" ", index);
            String sub_string = buf_string.substring(index, last_index);
            led_blink_cyc = 0;
            for(int i=0; i < sub_string.length(); i++){
                led_blink_cyc = led_blink_cyc * 10 + (sub_string.charAt(i) - '0');
            }
        }      
        cmd = web_led_control;
        cmd.replace("LED1_ISCHECKED", (gled1_state)?"checked":"");
        cmd.replace("LED2_ISCHECKED", (gled2_state)?"checked":"");
        cmd.replace("LED_MODE_ONOFF", (led_mode == LED_MODE_ONOFF)?"checked":"");
        cmd.replace("LED_MODE_BLINK", (led_mode == LED_MODE_BLINK)?"checked":"");      
        String led_blink_cyc_str(led_blink_cyc);
        cmd.replace("LED_BLINK_CYC_DISABLE",  (led_mode != LED_MODE_BLINK)?"disabled":"");      
        cmd.replace("LED_BLINK_CYC",  led_blink_cyc_str);
    }
    else if((buf_string.indexOf("/sensormonitor") != -1)) {
        cmd = web_sensormonitor;
        cmd.replace("OBJ_TEMP_I", String((int)tmp007_objt));
        cmd.replace("OBJ_TEMP_D", String((int)((tmp007_objt - (int)tmp007_objt) * 100)));
        cmd.replace("DIE_TEMP_I", String((int)tmp007_diet));
        cmd.replace("DIE_TEMP_D", String((int)((tmp007_diet - (int)tmp007_diet) * 100)));
        cmd.replace("TSL2591_LIGHT_I", String((int)tsl2591_light));
        cmd.replace("TSL2591_LIGHT_D", String((int)((tsl2591_light - (int)tsl2591_light) * 100)));
        cmd.replace("VCNL4000_PROXIMITY", String(vcnl4000_proximity));      
    }else {
        cmd = web_top;
    }
        
    wifi.send(mux_id, cmd);
    wifi.releaseTCP(mux_id);
}

/////////////////////////////////////////////////////////////////////
//
//                       LED Task
//
/////////////////////////////////////////////////////////////////////
void task3_setup() {
    Serial.println("LED Task : setup start!");
    pinMode(LED1_PORT, OUTPUT);
    digitalWrite(LED1_PORT, LOW);
    pinMode(LED2_PORT, OUTPUT);
    digitalWrite(LED2_PORT, LOW);        
}

void loop3() {
    static bool gled1_blink_state = false;
    static bool gled2_blink_state = false;
    static uint32_t analog_cnt = 0;
    static bool     analog_cnt_mode = true;
    
    if(led_mode == LED_MODE_ONOFF) {
        digitalWrite(LED1_PORT, (gled1_state)?HIGH:LOW);
        digitalWrite(LED2_PORT, (gled2_state)?HIGH:LOW);
    }
    else {
        if((++led_cycle) > (led_blink_cyc/2)) {
            led_cycle = 0;            
            if(gled1_state){
                if(gled1_blink_state){
                    gled1_blink_state = false;
                    digitalWrite(LED1_PORT,LOW);
                }else{
                    gled1_blink_state = true;
                    digitalWrite(LED1_PORT,HIGH);
                }                
            }
            else {
                digitalWrite(LED1_PORT, LOW);
            }            
            if(gled2_state){
                if(gled2_blink_state){
                    gled2_blink_state = false;
                    digitalWrite(LED2_PORT,LOW);
                }else{
                    gled2_blink_state = true;
                    digitalWrite(LED2_PORT,HIGH);
                }                
            }
            else {
                digitalWrite(LED2_PORT,LOW);
            }            
        }
    }

    if(analog_cnt_mode){
        analog_cnt++;
        if(analog_cnt > 255*5){
            analog_cnt_mode = false;
        }
    }else{
        analog_cnt--;
        if(analog_cnt == 0){
            analog_cnt_mode = true;
        }
    }
    
    analogWrite(13,analog_cnt/5);

    delay(1);
}

/////////////////////////////////////////////////////////////////////
//
//                       TFT Task
//
/////////////////////////////////////////////////////////////////////
/*

 Arduino TFT Bitmap Logo example

 This example reads an image file from a micro-SD card
 and draws it on the screen, at random locations.

 In this sketch, the Arduino logo is read from a micro-SD card.
 There is a .bmp file included with this sketch.
 - open the sketch folder (Ctrl-K or Cmd-K)
 - copy the "arduino.bmp" file to a micro-SD
 - put the SD into the SD slot of the Arduino TFT module.

 This example code is in the public domain.

 Created 19 April 2013 by Enrico Gueli

 http://arduino.cc/en/Tutorial/TFTBitmapLogo

 */

// include the necessary libraries
#include <SPI.h>
#include <SD.h>
#include <TFT.h>  // Arduino LCD library

// pin definition for the Uno
#define sd_cs  7
#define lcd_cs 10
#define dc     9
#define rst    8

// pin definition for the Leonardo
//#define sd_cs  8
//#define lcd_cs 7
//#define dc     0
//#define rst    1

TFT TFTscreen = TFT(lcd_cs, dc, rst);

// this variable represents the image to be drawn on screen
PImage logo;

void task4_setup() {
  Serial.println(F("TFT Task : start!"));
    
  // initialize the GLCD and show a message
  // asking the user to open the serial line
  TFTscreen.begin();
  TFTscreen.setRotation(0);
  TFTscreen.background(255, 255, 255);

  TFTscreen.stroke(0, 0, 255);
  TFTscreen.println();
  TFTscreen.println(F("Arduino TFT Bitmap Example"));
  TFTscreen.stroke(0, 0, 0);
  TFTscreen.println(F("Open serial monitor"));
  TFTscreen.println(F("to run the sketch"));

  delay(1000);
    
  // clear the GLCD screen before starting
  TFTscreen.background(255, 255, 255);

  // try to access the SD card. If that fails (e.g.
  // no card present), the setup process will stop.
  Serial.print(F("Initializing SD card..."));
  if (!SD.begin(sd_cs)) {
    Serial.println(F("failed!"));
    return;
  }
  Serial.println(F("OK!"));

  // initialize and clear the GLCD screen
  TFTscreen.begin();
  TFTscreen.setRotation(0);    
  TFTscreen.background(255, 255, 255);

  // now that the SD card can be access, try to load the
  // image file.
  logo = TFTscreen.loadImage("topame.bmp");
  if (!logo.isValid()) {
    Serial.println(F("error while loading arduino.bmp"));
  }
}

int loop_cnt = 0;;

void loop4() {    
  // don't do anything if the image wasn't loaded correctly.
  if (logo.isValid() == false) {
    return;
  }
    
  TFTscreen.setCursor(0, 30);
  TFTscreen.print(F("loop() is running "));
  TFTscreen.print(loop_cnt++);
  TFTscreen.print(F("."));    
  delay(1000);
  TFTscreen.background(255, 255, 255);

  // get a random location where to draw the image.
  // To avoid the image to be draw outside the screen,
  // take into account the image size.
  int x = random(TFTscreen.width() - logo.width());
  int y = random(TFTscreen.height() - logo.height());

  // draw the image to the screen
  TFTscreen.image(logo, x, y);

  // wait a little bit before drawing again    
  delay(2000);
  TFTscreen.background(255, 255, 255);    
}

/////////////////////////////////////////////////////////////////////
//
//                       Processing Task
//
/////////////////////////////////////////////////////////////////////
//#undef Serial 
#define PSERIAL Serial3
#define OSERIAL Serial

bool processing_connected = false;

void establishContact() {
    while (!PSERIAL.available()) {
        PSERIAL.print('A'); // send a capital A
        delay(300);
    }
    OSERIAL.println("Processing Task : Connect!");
    processing_connected = true;
}

void task5_setup()
{
    PSERIAL.begin(115200);
    while(!PSERIAL){
        delay(1);
    }
    OSERIAL.println("Processing Task : Start!");
}

int inByte = 0;
int last_connect = 0;

#define TIMEOUT_MS  3000

void loop5()
{
    if(!processing_connected){
        establishContact();        
    }else{
        if((millis() - last_connect) > TIMEOUT_MS){
            processing_connected = false;
            OSERIAL.println("Processing Task : Disconnect!");
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
#if 0                
        uint8_t buf[10];
        buf[0] = (uint8_t)(load >> 8);
        buf[1] = (uint8_t)load;
        buf[2] = (uint8_t)(isr_cnt >> 8);
        buf[3] = (uint8_t)isr_cnt;
        buf[4] = (uint8_t)(dispatch_cnt >> 8);
        buf[5] = dispatch_cnt;
        PSERIAL.write(buf, 6);
#endif        
        PSERIAL.write((uint8_t)(load >> 8));
        PSERIAL.write((uint8_t)load);
        PSERIAL.write((uint8_t)(isr_cnt >> 8));                
        PSERIAL.write((uint8_t)isr_cnt);        
        PSERIAL.write((uint8_t)(dispatch_cnt >> 8));
        PSERIAL.write((uint8_t)dispatch_cnt);

    }
    delay(1);
}
