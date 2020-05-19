#include "r2ca.h"
#include <Wire.h>
#include <ZumoShield.h>

#define BUTTON_BASIC
//#define MOTOR_BASIC
//#define BUZZER_BASIC
//#define GYRO_BASIC
//#define REFLECTANCE_BASIC
//#define COMPASS_BASIC
//#define RELECTORNCE_STOP
//#define ROTATIONRESIST
//#define FORCEUPHILL
//#define LINEFOLLOWER
//#define BORDERDETECT
//#define SUMOCOLLISIONDETECT
//#define MAZESOLVER

#ifdef BUTTON_BASIC

void button_int() {
    Serial.println("Button Interrupt!");
}

void setup() {
    Serial.begin(115200);
    ZumoInit();
    attachInterrupt(ZUMO_BUTTON,  button_int, FALLING);
}

void loop() {
    button.waitForPress();
    led.on();
    delay(1000);
    led.off();  
    delay(1000);
    led.on();
    delay(1000);
    led.off();    
    button.waitForRelease();
}
#endif /* BUTTON_BASIC */

#ifdef MOTOR_BASIC

void setup() {
    ZumoInit();
    buzzer.playOn();
}

void loop() {
    button.waitForPress();

    // run left motor forward
    led.on();
    for (int speed = 0; speed <= 400; speed++) {
        motors.setLeftSpeed(speed);
        delay(2);
    }

    for (int speed = 400; speed >= 0; speed--) {
        motors.setLeftSpeed(speed);
        delay(2);
    }

    // run left motor backward
    led.off();
    for (int speed = 0; speed >= -400; speed--) {
        motors.setLeftSpeed(speed);
        delay(2);
    }
    
    for (int speed = -400; speed <= 0; speed++) {
        motors.setLeftSpeed(speed);
        delay(2);
    }

    // run right motor forward
    led.on();
    buzzer.playNum(1);
    for (int speed = 0; speed <= 400; speed++){
        motors.setRightSpeed(speed);
        delay(2);
    }

    for (int speed = 400; speed >= 0; speed--){
        motors.setRightSpeed(speed);
        delay(2);
    }

    // run right motor backward
    led.off();
    buzzer.playNum(1);    
    for (int speed = 0; speed >= -400; speed--){
        motors.setRightSpeed(speed);
        delay(2);
    }

    for (int speed = -400; speed <= 0; speed++){
        motors.setRightSpeed(speed);
        delay(2);
    }
    
    delay(500);
    
    led.on();
    buzzer.playNum(1);        
    motors.setSpeeds(100, 100);
    delay(2000);
    motors.setSpeeds(0, 0);
    
    led.off();
    buzzer.playNum(1);            
    motors.setSpeeds(-100, -100);
    delay(2000);
    motors.setSpeeds(0, 0);
    
    led.on();
    buzzer.playNum(1);
    motors.setSpeeds(-100, 100);
    delay(2000);
    motors.setSpeeds(0, 0);

    led.off();
    buzzer.playNum(1);
    motors.setSpeeds(100, -100);
    delay(2000);
    motors.setSpeeds(0, 0);
}
#endif /* MOTOR_BASIC */

#ifdef BUZZER_BASIC
void setup() {
    ZumoInit();
    buzzer.playOn();
    buzzer.playNum(2);
    buzzer.playMode(PLAY_AUTOMATIC);
    Serial.begin(115200);
}

const char sound_effect[] = "O4 T100 V15 L4 MS g12>c12>e12>G6>E12 ML>G2"; // "charge" melody

void loop() {
    button.waitForPress();
    buzzer.playFromProgramSpace(sound_effect);
    button.waitForRelease();
}
#endif /* BUZZER_BASIC */


#ifdef GYRO_BASIC
void setup()
{
    Serial.begin(115200);
    
    buzzer.playOn();  
    gyro.turnSensorSetup();
    delay(500);
    gyro.turnSensorReset();
    
    buzzer.playStart();
}

void loop()
{  
  gyro.turnSensorUpdate();
  Serial.println(gyro.turnAngleDegree);
}

#endif /* GYRO_BASIC */


#ifdef REFLECTANCE_BASIC

void setup() {
  buzzer.playOn();
  Serial.begin(115200);
  Serial.println("Zumo sample Start!");
}

void loop() {
  reflectances.update();
  
  Serial.print(reflectances.value(1));
  Serial.print(',');
  Serial.print(reflectances.value(2));
  Serial.print(',');  
  Serial.print(reflectances.value(3));
  Serial.print(',');
  Serial.print(reflectances.value(4));
  Serial.print(',');  
  Serial.print(reflectances.value(5));
  Serial.print(',');  
  Serial.print(reflectances.value(6));
  Serial.print(',');    
  Serial.println();
  
  delay(1000);
}
#endif /* REFLECTANCE_BASIC */




#ifdef COMPASS_BASIC

#define min(a,b) ((a)<(b)?(a):(b))
#define max(a,b) ((a)>(b)?(a):(b))
#define SPEED           200
#define CALIBRATION_SAMPLES 70

void setup() {
    Serial.begin(115200);
    ZumoInit();
    compass.begin();
#if 0    
  LSM303::vector<int16_t> running_min = {32767, 32767, 32767}, running_max = {-32767, -32767, -32767};
  unsigned char index;
  Serial.println("starting calibration");

  // To calibrate the magnetometer, the Zumo spins to find the max/min
  // magnetic vectors. This information is used to correct for offsets
  // in the magnetometer data.
  motors.setLeftSpeed(SPEED);
  motors.setRightSpeed(-SPEED);

  for(index = 0; index < CALIBRATION_SAMPLES; index ++)
  {
    // Take a reading of the magnetic vector and store it in compass.m
    compass.read();

    running_min.x = min(running_min.x, compass.m.x);
    running_min.y = min(running_min.y, compass.m.y);

    running_max.x = max(running_max.x, compass.m.x);
    running_max.y = max(running_max.y, compass.m.y);

    Serial.println(index);

    delay(50);
  }

  motors.setLeftSpeed(0);
  motors.setRightSpeed(0);

  Serial.print("max.x   ");
  Serial.print(running_max.x);
  Serial.println();
  Serial.print("max.y   ");
  Serial.print(running_max.y);
  Serial.println();
  Serial.print("min.x   ");
  Serial.print(running_min.x);
  Serial.println();
  Serial.print("min.y   ");
  Serial.print(running_min.y);
  Serial.println();

  // Set calibrated values to compass.m_max and compass.m_min
  compass.m_max.x = running_max.x;
  compass.m_max.y = running_max.y;
  compass.m_min.x = running_min.x;
  compass.m_min.y = running_min.y;
  button.waitForButton();
#endif    
}

void loop() {
  float heading;

  heading = compass.averageHeading();
  Serial.print("Heading: ");
  Serial.println(heading);
}

#endif /* COMPASS_BASIC */

#ifdef RELECTORNCE_STOP

#define REFLECTANCE_THRESHOLD  400

void setup() {
    ZumoInit();
    Serial.begin(115200);    
    buzzer.playOn();
    
    Serial.println("Zumo sample Start!");
    button.waitForPress();
}

void loop() {
    reflectances.update();

    while ((reflectances.value(3) < REFLECTANCE_THRESHOLD) && 
           (reflectances.value(4) < REFLECTANCE_THRESHOLD)) {
        motors.setSpeeds(100, 100);
        led.on();
        reflectances.update();
    }
    motors.setSpeeds(0, 0);
    led.off();
    button.waitForPress();
}
#endif /* COMPASS_BASIC */


#ifdef ROTATIONRESIST
/* This demo shows how the Zumo can use its gyroscope to detect
when it is being rotated, and use the motors to resist that
rotation.

This code was tested on a Zumo with 4 NiMH batteries and two 75:1
HP micro metal gearmotors.  If you have different batteries or
motors, you might need to adjust the PID constants.

Be careful to not move the robot for a few seconds after starting
it while the gyro is being calibrated.  During the gyro
calibration, the yellow LED is on and the words "Gyro cal" are
displayed on the LCD.

After the gyro calibration is done, press button A to start the
demo.  If you try to turn the Zumo, or put it on a surface that
is turning, it will drive its motors to counteract the turning.

This demo only uses the Z axis of the gyro, so it is possible to
pick up the Zumo, rotate it about its X and Y axes, and then put
it down facing in a new position. */

// This is the maximum speed the motors will be allowed to turn.
// A maxSpeed of 400 lets the motors go at top speed.  Decrease
// this value to impose a speed limit.
const int16_t maxSpeed = 400;

void setup()
{  
    buzzer.playOn();
    ZumoInit();
    Serial.begin(115200);    
    button.waitForButton();
  
    gyro.turnSensorSetup();
    delay(500);
    gyro.turnSensorReset();
    
    buzzer.playStart();
    delay(1000);
}

void loop()
{  
  // Read the gyro to update turnAngle, the estimation of how far
  // the robot has turned, and turnRate, the estimation of how
  // fast it is turning.
  gyro.turnSensorUpdate();
  Serial.println(gyro.turnAngleDegree);

  // Calculate the motor turn speed using proportional and
  // derivative PID terms.  Here we are a using a proportional
  // constant of 56 and a derivative constant of 1/20.
  int32_t turnSpeed = -(int32_t)(gyro.turnAngle) / (gyro.turnAngle1 / 56)
    - gyro.turnRate / 20;

  // Constrain our motor speeds to be between
  // -maxSpeed and maxSpeed.
  turnSpeed = constrain(turnSpeed, -maxSpeed, maxSpeed);

  motors.setSpeeds(-turnSpeed, turnSpeed);  
}
#endif /* ROTATIONRESIST */

#ifdef FORCEUPHILL
/* This demo shows how the Zumo can use its gyroscope to detect
when it is being rotated, and use the motors to resist that
rotation.

This code was tested on a Zumo with 4 NiMH batteries and two 75:1
HP micro metal gearmotors.  If you have different batteries or
motors, you might need to adjust the PID constants.

Be careful to not move the robot for a few seconds after starting
it while the gyro is being calibrated.  During the gyro
calibration, the yellow LED is on and the words "Gyro cal" are
displayed on the LCD.

After the gyro calibration is done, press button A to start the
demo.  If you try to turn the Zumo, or put it on a surface that
is turning, it will drive its motors to counteract the turning.

This demo only uses the Z axis of the gyro, so it is possible to
pick up the Zumo, rotate it about its X and Y axes, and then put
it down facing in a new position. */

// This is the maximum speed the motors will be allowed to turn.
// A maxSpeed of 400 lets the motors go at top speed.  Decrease
// this value to impose a speed limit.
const int16_t maxSpeed = 400;

void setup()
{  
    buzzer.playOn();
    ZumoInit();
    Serial.begin(115200);
    
    button.waitForButton();
    compass.begin();
    buzzer.playStart();
    led.on();
}

void loop()
{  
  // Read the acceleration from the LSM303.
  // A value of 16384 corresponds to approximately 1 g.
  compass.read();
  int16_t x = compass.a.x;
  int16_t y = compass.a.y;
  int32_t magnitudeSquared = (int32_t)x * x + (int32_t)y * y;

  // Use the encoders to see how much we should drive forward.
  // If the robot rolls downhill, the encoder counts will become
  // negative, resulting in a positive forwardSpeed to counteract
  // the rolling.
  int16_t forwardSpeed = 20;

  // See if we are actually on an incline.
  // 16384 * sin(5 deg) = 1427
  int16_t turnSpeed;
  if (magnitudeSquared > (int32_t)1427 * 1427)
  {
    // We are on an incline of more than 5 degrees, so
    // try to face uphill using a feedback algorithm.
    turnSpeed = y / 16;
  }
  else
  {
    // We not on a noticeable incline, so don't turn.
    turnSpeed = 0;
  }

  // To face uphill, we need to turn so that the X acceleration
  // is negative and the Y acceleration is 0.  Therefore, when
  // the Y acceleration is positive, we want to turn to the
  // left (counter-clockwise).
  int16_t leftSpeed = forwardSpeed - turnSpeed;
  int16_t rightSpeed = forwardSpeed + turnSpeed;

  // Constrain the speeds to be between -maxSpeed and maxSpeed.
  leftSpeed = constrain(leftSpeed, -maxSpeed, maxSpeed);
  rightSpeed = constrain(rightSpeed, -maxSpeed, maxSpeed);

  motors.setSpeeds(leftSpeed, rightSpeed);  
}
#endif /* FORCEUPHILL */


#ifdef LINEFOLLOWER
/*
 * Demo line-following code for the Pololu Zumo Robot
 *
 * This code will follow a black line on a white background, using a
 * PID-based algorithm.  It works decently on courses with smooth, 6"
 * radius curves and has been tested with Zumos using 30:1 HP and
 * 75:1 HP motors.  Modifications might be required for it to work
 * well on different courses or with different motors.
 *
 * http://www.pololu.com/catalog/product/2506
 * http://www.pololu.com
 * http://forum.pololu.com
 */
int lastError = 0;

// This is the maximum speed the motors will be allowed to turn.
// (400 lets the motors go at top speed; decrease to impose a speed limit)
const int MAX_SPEED = 400;

// Sound Effects
const char sound_effect[] PROGMEM = "O4 T100 V15 L4 MS g12>c12>e12>G6>E12 ML>G2"; // "charge" melody

void setup()
{
    // Play a little welcome song
    buzzer.playOn();
    ZumoInit();
    Serial.begin(115200);    

    // Initialize the reflectance sensors module
    reflectances.init();

    // Wait for the user button to be pressed and released
    button.waitForButton();
    
    // Turn on LED to indicate we are in calibration mode
    led.on();

    // Wait 1 second and then begin automatic sensor calibration
    // by rotating in place to sweep the sensors over the line
    delay(1000);
    int i;
    for(i = 0; i < 80; i++) {
        if ((i > 10 && i <= 30) || (i > 50 && i <= 70))
          motors.setSpeeds(-200, 200);
        else
          motors.setSpeeds(200, -200);
        reflectances.calibrate();
        
        // Since our counter runs to 80, the total delay will be
        // 80*20 = 1600 ms.
        delay(20);
    }
    motors.setSpeeds(0,0);
    
    // Turn off LED to indicate we are through with calibration
    led.off();
    buzzer.playOn();

    // Wait for the user button to be pressed and released
    button.waitForButton();
    
    // Play music and wait for it to finish before we start driving.
    buzzer.playFromProgramSpace(sound_effect);
    while(buzzer.isPlaying()){delay(1);};
}

void loop()
{
  unsigned int sensors[6];

  // Get the position of the line.  Note that we *must* provide the "sensors"
  // argument to readLine() here, even though we are not interested in the
  // individual sensor readings
  int position = reflectances.readLine(sensors);

  // Our "error" is how far we are away from the center of the line, which
  // corresponds to position 2500.
  int error = position - 2500;

  // Get motor speed difference using proportional and derivative PID terms
  // (the integral term is generally not very useful for line following).
  // Here we are using a proportional constant of 1/4 and a derivative
  // constant of 6, which should work decently for many Zumo motor choices.
  // You probably want to use trial and error to tune these constants for
  // your particular Zumo and line course.
  int speedDifference = error / 4 + 6 * (error - lastError);

  lastError = error;

  // Get individual motor speeds.  The sign of speedDifference
  // determines if the robot turns left or right.
  int m1Speed = MAX_SPEED + speedDifference;
  int m2Speed = MAX_SPEED - speedDifference;

  // Here we constrain our motor speeds to be between 0 and MAX_SPEED.
  // Generally speaking, one motor will always be turning at MAX_SPEED
  // and the other will be at MAX_SPEED-|speedDifference| if that is positive,
  // else it will be stationary.  For some applications, you might want to
  // allow the motor speed to go negative so that it can spin in reverse.
  if (m1Speed < 0)
    m1Speed = 0;
  if (m2Speed < 0)
    m2Speed = 0;
  if (m1Speed > MAX_SPEED)
    m1Speed = MAX_SPEED;
  if (m2Speed > MAX_SPEED)
    m2Speed = MAX_SPEED;

  motors.setSpeeds(m1Speed, m2Speed);
}

#endif /* LINEFOLLOWER */

#ifdef BORDERDETECT

// this might need to be tuned for different lighting conditions, surfaces, etc.
#define QTR_THRESHOLD  600

// these might need to be tuned for different motor types
#define REVERSE_SPEED     200 // 0 is stopped, 400 is full speed
#define TURN_SPEED        200
#define FORWARD_SPEED     200
#define REVERSE_DURATION  300 // ms
#define TURN_DURATION     400 // ms

void setup()
{
    buzzer.playOn();

    ZumoInit();
    
    Serial.begin(115200);
    Serial.println("Zumo sample Start!");

    led.on();
    button.waitForButton();
    led.off();
    buzzer.playNum(3);
    delay(1000);
    buzzer.playStart();
    delay(1000);
}

void loop()
{
  if (button.isPressed()) {
    motors.setSpeeds(0, 0);
    button.waitForRelease();

    led.on();
    button.waitForButton();
    led.off();
    buzzer.playNum(3);
    delay(1000);
    buzzer.playStart();
    delay(1000);
  }

  reflectances.update();

  if (reflectances.value(1) > QTR_THRESHOLD) {
    // if leftmost sensor detects line, reverse and turn to the right
    motors.setSpeeds(-REVERSE_SPEED, -REVERSE_SPEED);
    delay(REVERSE_DURATION);
    motors.setSpeeds(TURN_SPEED, -TURN_SPEED);
    delay(TURN_DURATION);
    motors.setSpeeds(FORWARD_SPEED, FORWARD_SPEED);
  } else if (reflectances.value(6)  > QTR_THRESHOLD) {
    // if rightmost sensor detects line, reverse and turn to the left
    motors.setSpeeds(-REVERSE_SPEED, -REVERSE_SPEED);
    delay(REVERSE_DURATION);
    motors.setSpeeds(-TURN_SPEED, TURN_SPEED);
    delay(TURN_DURATION);
    motors.setSpeeds(FORWARD_SPEED, FORWARD_SPEED);
  } else {
    // otherwise, go straight
    motors.setSpeeds(FORWARD_SPEED, FORWARD_SPEED);
  }
}
#endif /* BORDERDETECT */


#ifdef SUMOCOLLISIONDETECT
/* This example uses the accelerometer in the Zumo Shield's onboard LSM303DLHC with the LSM303 Library to 
 * detect contact with an adversary robot in the sumo ring. The LSM303 Library is not included in the Zumo 
 * Shield libraries; it can be downloaded separately from GitHub at: 
 *
 *    https://github.com/pololu/LSM303 
 *
 * This example extends the BorderDetect example, which makes use of the onboard Zumo Reflectance Sensor Array
 * and its associated library to detect the border of the sumo ring.  It also illustrates the use of the 
 * ZumoMotors, PushButton, and ZumoBuzzer libraries.
 *
 * In loop(), the program reads the x and y components of acceleration (ignoring z), and detects a
 * contact when the magnitude of the 3-period average of the x-y vector exceeds an empirically determined
 * XY_ACCELERATION_THRESHOLD.  On contact detection, the forward speed is increased to FULL_SPEED from
 * the default SEARCH_SPEED, simulating a "fight or flight" response.
 *
 * The program attempts to detect contact only when the Zumo is going straight.  When it is executing a
 * turn at the sumo ring border, the turn itself generates an acceleration in the x-y plane, so the 
 * acceleration reading at that time is difficult to interpret for contact detection.  Since the Zumo also 
 * accelerates forward out of a turn, the acceleration readings are also ignored for MIN_DELAY_AFTER_TURN 
 * milliseconds after completing a turn. To further avoid false positives, a MIN_DELAY_BETWEEN_CONTACTS is 
 * also specified.
 *
 * This example also contains the following enhancements:
 * 
 *  - uses the Zumo Buzzer library to play a sound effect ("charge" melody) at start of competition and 
 *    whenever contact is made with an opposing robot
 *
 *  - randomizes the turn angle on border detection, so that the Zumo executes a more effective search pattern
 *
 *  - supports a FULL_SPEED_DURATION_LIMIT, allowing the robot to switch to a SUSTAINED_SPEED after a short 
 *    period of forward movement at FULL_SPEED.  In the example, both speeds are set to 400 (max), but this 
 *    feature may be useful to prevent runoffs at the turns if the sumo ring surface is unusually smooth.
 *
 *  - logging of accelerometer output to the serial monitor when LOG_SERIAL is #defined.
 *
 *  This example also makes use of the public domain RunningAverage library from the Arduino website; the relevant
 *  code has been copied into this .ino file and does not need to be downloaded separately.
 */

// #define LOG_SERIAL // write log output to serial port

// Accelerometer Settings
#define RA_SIZE 3  // number of readings to include in running average of accelerometer readings
#define XY_ACCELERATION_THRESHOLD 2400  // for detection of contact (~16000 = magnitude of acceleration due to gravity)

// this might need to be tuned for different lighting conditions, surfaces, etc.
#define QTR_THRESHOLD  1000 // microseconds

// these might need to be tuned for different motor types
#define REVERSE_SPEED     200 // 0 is stopped, 400 is full speed
#define TURN_SPEED        200
#define SEARCH_SPEED      200
#define SUSTAINED_SPEED   400 // switches to SUSTAINED_SPEED from FULL_SPEED after FULL_SPEED_DURATION_LIMIT ms
#define FULL_SPEED        400
#define STOP_DURATION     100 // ms
#define REVERSE_DURATION  300 // ms
#define TURN_DURATION     300 // ms

#define RIGHT 1
#define LEFT -1

enum ForwardSpeed { SearchSpeed, SustainedSpeed, FullSpeed };
ForwardSpeed _forwardSpeed;  // current forward speed setting
unsigned long full_speed_start_time;
#define FULL_SPEED_DURATION_LIMIT     250  // ms

// Sound Effects
const char sound_effect[] PROGMEM = "O4 T100 V15 L4 MS g12>c12>e12>G6>E12 ML>G2"; // "charge" melody
 // use V0 to suppress sound effect; v15 for max volume
 
 // Timing
unsigned long loop_start_time;
unsigned long last_turn_time;
unsigned long contact_made_time;
#define MIN_DELAY_AFTER_TURN          400  // ms = min delay before detecting contact event
#define MIN_DELAY_BETWEEN_CONTACTS   1000  // ms = min delay between detecting new contact event

Accelerometer lsm303;
boolean in_contact;  // set when accelerometer detects contact with opposing robot

// forward declaration
void setForwardSpeed(ForwardSpeed speed);
void waitForButtonAndCountDown(bool restarting);
void turn(char direction, bool randomize);
bool check_for_contact();
void on_contact_made();
void on_contact_lost();
int getForwardSpeed();
void setForwardSpeed(ForwardSpeed speed);

void setup()
{
  buzzer.playOn();

  ZumoInit();
    
  lsm303.begin();
  
#ifdef LOG_SERIAL
  Serial.begin(115200);
  lsm303.getLogHeader();
#endif

  randomSeed((unsigned int) millis());  

  led.on();
  buzzer.playMode(PLAY_AUTOMATIC);
  waitForButtonAndCountDown(false);
}

void loop()
{
  if (button.isPressed()) {
    // if button is pressed, stop and wait for another press to go again
    motors.setSpeeds(0, 0);
    button.waitForRelease();
    waitForButtonAndCountDown(true);
  }
  
  loop_start_time = millis();
  lsm303.readAcceleration(loop_start_time); 
  reflectances.update();
  
  if ((_forwardSpeed == FullSpeed) && 
      (loop_start_time - full_speed_start_time > FULL_SPEED_DURATION_LIMIT)){ 
    setForwardSpeed(SustainedSpeed);
  }
  
  if (reflectances.value(1) > QTR_THRESHOLD) {
    // if leftmost sensor detects line, reverse and turn to the right
    turn(RIGHT, true);
  }
  else if (reflectances.value(6) > QTR_THRESHOLD) {
    // if rightmost sensor detects line, reverse and turn to the left
    turn(LEFT, true);
  }
  else {
    // otherwise, go straight
    if (check_for_contact()) on_contact_made();
    int speed = getForwardSpeed();
    motors.setSpeeds(speed, speed);
  }
}

void waitForButtonAndCountDown(bool restarting)
{ 
#ifdef LOG_SERIAL
  Serial.print(restarting ? "Restarting Countdown" : "Starting Countdown");
  Serial.println();
#endif
  
  led.on();
  button.waitForButton();
  led.off();
   
  // play audible countdown
  buzzer.playNum(3);
  delay(1000);
  buzzer.playFromProgramSpace(sound_effect);
  delay(1000);
  
  // reset loop variables
  in_contact = false;  // 1 if contact made; 0 if no contact or contact lost
  contact_made_time = 0;
  last_turn_time = millis();  // prevents false contact detection on initial acceleration
  _forwardSpeed = SearchSpeed;
  full_speed_start_time = 0;
}

// execute turn 
// direction:  RIGHT or LEFT
// randomize: to improve searching
void turn(char direction, bool randomize)
{
#ifdef LOG_SERIAL
  Serial.print("turning ...");
  Serial.println();
#endif

  // assume contact lost
  on_contact_lost();
  
  static unsigned int duration_increment = TURN_DURATION / 4;
  
  // motors.setSpeeds(0,0);
  // delay(STOP_DURATION);
  motors.setSpeeds(-REVERSE_SPEED, -REVERSE_SPEED);
  delay(REVERSE_DURATION);
  motors.setSpeeds(TURN_SPEED * direction, -TURN_SPEED * direction);
  delay(randomize ? TURN_DURATION + (random(8) - 2) * duration_increment : TURN_DURATION);
  int speed = getForwardSpeed();
  motors.setSpeeds(speed, speed);
  last_turn_time = millis();
}

void setForwardSpeed(ForwardSpeed speed)
{
  _forwardSpeed = speed;
  if (speed == FullSpeed) full_speed_start_time = loop_start_time;
}

int getForwardSpeed()
{
  int speed;
  switch (_forwardSpeed)
  {
    case FullSpeed:
      speed = FULL_SPEED;
      break;
    case SustainedSpeed:
      speed = SUSTAINED_SPEED;
      break;
    default:
      speed = SEARCH_SPEED;
      break;
  }
  return speed;
}
  
// check for contact, but ignore readings immediately after turning or losing contact
bool check_for_contact()
{
  static long threshold_squared = (long) XY_ACCELERATION_THRESHOLD * (long) XY_ACCELERATION_THRESHOLD;
  return (lsm303.ss_xy_avg() >  threshold_squared) && \
    (loop_start_time - last_turn_time > MIN_DELAY_AFTER_TURN) && \
    (loop_start_time - contact_made_time > MIN_DELAY_BETWEEN_CONTACTS);
}

// sound horn and accelerate on contact -- fight or flight
void on_contact_made()
{
#ifdef LOG_SERIAL
  Serial.print("contact made");
  Serial.println();
#endif
  in_contact = true;
  contact_made_time = loop_start_time;
  setForwardSpeed(FullSpeed);
  buzzer.playFromProgramSpace(sound_effect);
}

// reset forward speed
void on_contact_lost()
{
#ifdef LOG_SERIAL
  Serial.print("contact lost");
  Serial.println();
#endif
  in_contact = false;
  setForwardSpeed(SearchSpeed);
}
#endif /* SUMOCOLLISIONDETECT */


#ifdef MAZESOLVER

/* This example uses the Zumo Reflectance Sensor Array
 * to navigate a black line maze with no loops. This program
 * is based off the 3pi maze solving example which can be
 * found here:
 *
 * http://www.pololu.com/docs/0J21/8.a
 * 
 * The Zumo first calibrates the sensors to account
 * for differences of the black line on white background.
 * Calibration is accomplished in setup().
 *
 * In loop(), the function solveMaze() is called and navigates
 * the Zumo until it finds the finish line which is defined as
 * a large black area that is thick and wide enough to
 * cover all six sensors at the same time.
 * 
 * Once the Zumo reaches the finishing line, it will stop and
 * wait for the user to place the Zumo back at the starting
 * line. The Zumo can then follow the shortest path to the finish
 * line.
 *
 * The macros SPEED, TURN_SPEED, ABOVE_LINE(), and LINE_THICKNESS 
 * might need to be adjusted on a case by case basis to give better 
 * line following results.
 */

// SENSOR_THRESHOLD is a value to compare reflectance sensor
// readings to to decide if the sensor is over a black line
#define SENSOR_THRESHOLD 300

// ABOVE_LINE is a helper macro that takes returns
// 1 if the sensor is over the line and 0 if otherwise
#define ABOVE_LINE(sensor)((sensor) > SENSOR_THRESHOLD)

// Motor speed when turning. TURN_SPEED should always
// have a positive value, otherwise the Zumo will turn
// in the wrong direction.
#define TURN_SPEED 200

// Motor speed when driving straight. SPEED should always
// have a positive value, otherwise the Zumo will travel in the
// wrong direction.
#define SPEED 200 

// Thickness of your line in inches
#define LINE_THICKNESS .75 

// When the motor speed of the zumo is set by
// motors.setSpeeds(200,200), 200 is in ZUNITs/Second.
// A ZUNIT is a fictitious measurement of distance
// and only helps to approximate how far the Zumo has
// traveled. Experimentally it was observed that for 
// every inch, there were approximately 17142 ZUNITs.
// This value will differ depending on setup/battery
// life and may be adjusted accordingly. This value
// was found using a 75:1 HP Motors with batteries
// partially discharged.
#define INCHES_TO_ZUNITS 17142.0

// When the Zumo reaches the end of a segment it needs
// to find out three things: if it has reached the finish line,
// if there is a straight segment ahead of it, and which
// segment to take. OVERSHOOT tells the Zumo how far it needs
// to overshoot the segment to find out any of these things.
#define OVERSHOOT(line_thickness)(((INCHES_TO_ZUNITS * (line_thickness)) / SPEED))

// path[] keeps a log of all the turns made
// since starting the maze
char path[100] = "";
unsigned char path_length = 0; // the length of the path

void turn(char dir);
char selectTurn(unsigned char found_left, unsigned char found_straight, unsigned char found_right);
void solveMaze();
void goToFinishLine();
void simplifyPath();

void setup()
{
    unsigned int sensors[6];
    unsigned short count = 0;
    unsigned short last_status = 0;
    int turn_direction = 1;  

    buzzer.playOn();
    ZumoInit();
    reflectances.init();
  
    delay(500);
    led.on(); // turn on LED to indicate we are in calibration mode
   
    button.waitForButton();
  
    // Calibrate the Zumo by sweeping it from left to right
    for(int i = 0; i < 4; i ++)  {
        // Zumo will turn clockwise if turn_direction = 1.
        // If turn_direction = -1 Zumo will turn counter-clockwise.
        turn_direction *= -1;
        
        // Turn direction.
        motors.setSpeeds(turn_direction * TURN_SPEED, -1*turn_direction * TURN_SPEED);
      
        // This while loop monitors line position
        // until the turn is complete. 
        while(count < 2) {
            reflectances.calibrate();
            reflectances.readLine(sensors);
            if(turn_direction < 0) {
                // If the right  most sensor changes from (over white space -> over 
                // line or over line -> over white space) add 1 to count.
                count += ABOVE_LINE(sensors[5]) ^ last_status;
                last_status = ABOVE_LINE(sensors[5]);
            }
            else {
                // If the left most sensor changes from (over white space -> over 
                // line or over line -> over white space) add 1 to count.
                count += ABOVE_LINE(sensors[0]) ^ last_status;
                last_status = ABOVE_LINE(sensors[0]);	  
            }
        }
        
        count = 0;
        last_status = 0;
    }
    
    // Turn left.
    turn('L');
    
    motors.setSpeeds(0, 0);
    
    // Sound off buzzer to denote Zumo is finished calibrating
    buzzer.playStart();
    
    // Turn off LED to indicate we are through with calibration
    led.off();
}

void loop()
{

    // solveMaze() explores every segment
    // of the maze until it finds the finish
    // line.
    solveMaze();
  
    // Sound off buzzer to denote Zumo has solved the maze
    buzzer.play(">>a32");
    
    // The maze has been solved. When the user
    // places the Zumo at the starting line
    // and pushes the Zumo button, the Zumo
    // knows where the finish line is and
    // will automatically navigate.
    while(1) {
        button.waitForButton();
        goToFinishLine();
        // Sound off buzzer to denote Zumo is at the finish line.
        buzzer.play(">>a32");
    }
}

// Turns according to the parameter dir, which should be 
// 'L' (left), 'R' (right), 'S' (straight), or 'B' (back).
void turn(char dir)
{
    // count and last_status help
    // keep track of how much further
    // the Zumo needs to turn.
    unsigned short count = 0;
    unsigned short last_status = 0;
    unsigned int sensors[6];
  
    // dir tests for which direction to turn
    switch(dir) {
  
        // Since we're using the sensors to coordinate turns instead of timing them, 
        // we can treat a left turn the same as a direction reversal: they differ only 
        // in whether the zumo will turn 90 degrees or 180 degrees before seeing the 
        // line under the sensor. If 'B' is passed to the turn function when there is a
        // left turn available, then the Zumo will turn onto the left segment.
      case 'L':
      case 'B':
        // Turn left.
        motors.setSpeeds(-TURN_SPEED, TURN_SPEED);
        
        // This while loop monitors line position
        // until the turn is complete. 
        while(count < 2) {
            reflectances.readLine(sensors);
            
            // Increment count whenever the state of the sensor changes 
            // (white->black and black->white) since the sensor should 
            // pass over 1 line while the robot is turning, the final 
            // count should be 2
            count += ABOVE_LINE(sensors[1]) ^ last_status; 
            last_status = ABOVE_LINE(sensors[1]);
        }
        
        break;
        
      case 'R':
        // Turn right.
        motors.setSpeeds(TURN_SPEED, -TURN_SPEED);
      
        // This while loop monitors line position
        // until the turn is complete. 
        while(count < 2) {
          reflectances.readLine(sensors);
            count += ABOVE_LINE(sensors[4]) ^ last_status;
            last_status = ABOVE_LINE(sensors[4]);
        }
        
        break;
        
      case 'S':
        // Don't do anything!
        break;
    }
}

// This function decides which way to turn during the learning phase of
// maze solving.  It uses the variables found_left, found_straight, and
// found_right, which indicate whether there is an exit in each of the
// three directions, applying the "left hand on the wall" strategy.
char selectTurn(unsigned char found_left, unsigned char found_straight,
  unsigned char found_right)
{
    // Make a decision about how to turn.  The following code
    // implements a left-hand-on-the-wall strategy, where we always
    // turn as far to the left as possible.
    if(found_left)
      return 'L';
    else if(found_straight)
      return 'S';
    else if(found_right)
      return 'R';
    else
      return 'B';
}

// The maze is broken down into segments. Once the Zumo decides
// which segment to turn on, it will navigate until it finds another
// intersection. followSegment() will then return after the
// intersection is found.
void followSegment()
{
    unsigned int position;
    unsigned int sensors[6];
    int offset_from_center;
    int power_difference;
    
    while(1) {     
        // Get the position of the line.
        position = reflectances.readLine(sensors);
        
        // The offset_from_center should be 0 when we are on the line.
        offset_from_center = ((int)position) - 2500;
        
        // Compute the difference between the two motor power settings,
        // m1 - m2.  If this is a positive number the robot will turn
        // to the left.  If it is a negative number, the robot will
        // turn to the right, and the magnitude of the number determines
        // the sharpness of the turn.
        power_difference = offset_from_center / 3;
        
        // Compute the actual motor settings.  We never set either motor
        // to a negative value.
        if(power_difference > SPEED)
          power_difference = SPEED;
        if(power_difference < -SPEED)
          power_difference = -SPEED;
     
        if(power_difference < 0)
          motors.setSpeeds(SPEED + power_difference, SPEED);
        else
          motors.setSpeeds(SPEED, SPEED - power_difference);
        
        // We use the inner four sensors (1, 2, 3, and 4) for
        // determining whether there is a line straight ahead, and the
        // sensors 0 and 5 for detecting lines going to the left and
        // right.
     
        if(!ABOVE_LINE(sensors[0]) && !ABOVE_LINE(sensors[1]) && !ABOVE_LINE(sensors[2]) && !ABOVE_LINE(sensors[3]) && !ABOVE_LINE(sensors[4]) && !ABOVE_LINE(sensors[5])) {
            // There is no line visible ahead, and we didn't see any
            // intersection.  Must be a dead end.            
            return;
        }
        else if(ABOVE_LINE(sensors[0]) || ABOVE_LINE(sensors[5])) {
            // Found an intersection.
            return;
        }        
    }
}

// The solveMaze() function works by applying a "left hand on the wall" strategy: 
// the robot follows a segment until it reaches an intersection, where it takes the 
// leftmost fork available to it. It records each turn it makes, and as long as the 
// maze has no loops, this strategy will eventually lead it to the finish. Afterwards, 
// the recorded path is simplified by removing dead ends. More information can be 
// found in the 3pi maze solving example.
void solveMaze()
{
    while(1) {
        // Navigate current line segment
        followSegment();
        
        // These variables record whether the robot has seen a line to the
        // left, straight ahead, and right, while examining the current
        // intersection.
        unsigned char found_left = 0;
        unsigned char found_straight = 0;
        unsigned char found_right = 0;
         
        // Now read the sensors and check the intersection type.
        unsigned int sensors[6];
        reflectances.readLine(sensors);
        
        // Check for left and right exits.
        if(ABOVE_LINE(sensors[0]))
          found_left = 1;
        if(ABOVE_LINE(sensors[5]))
          found_right = 1;
        
        // Drive straight a bit more, until we are
        // approximately in the middle of intersection.
        // This should help us better detect if we
        // have left or right segments.
        motors.setSpeeds(SPEED, SPEED);
        delay(OVERSHOOT(LINE_THICKNESS)/2);
        
        reflectances.readLine(sensors);
        
        // Check for left and right exits.
        if(ABOVE_LINE(sensors[0]))
          found_left = 1;
        if(ABOVE_LINE(sensors[5]))
          found_right = 1;
        
        // After driving a little further, we
        // should have passed the intersection
        // and can check to see if we've hit the
        // finish line or if there is a straight segment
        // ahead.   
        delay(OVERSHOOT(LINE_THICKNESS)/2);
        
        // Check for a straight exit.
        reflectances.readLine(sensors);
        
        // Check again to see if left or right segment has been found
        if(ABOVE_LINE(sensors[0]))
          found_left = 1;
        if(ABOVE_LINE(sensors[5]))
            found_right = 1;
        
        if(ABOVE_LINE(sensors[1]) || ABOVE_LINE(sensors[2]) || ABOVE_LINE(sensors[3]) || ABOVE_LINE(sensors[4]))
          found_straight = 1;
         
        // Check for the ending spot.
        // If all four middle sensors are on dark black, we have
        // solved the maze.
        if(ABOVE_LINE(sensors[1]) && ABOVE_LINE(sensors[2]) && ABOVE_LINE(sensors[3]) && ABOVE_LINE(sensors[4])) {
            motors.setSpeeds(0,0);
            break;
        }
        
        // Intersection identification is complete.
        unsigned char dir = selectTurn(found_left, found_straight, found_right);
        
        // Make the turn indicated by the path.
        turn(dir);
         
        // Store the intersection in the path variable.
        path[path_length] = dir;
        path_length++;
         
        // You should check to make sure that the path_length does not
        // exceed the bounds of the array.  We'll ignore that in this
        // example.
         
        // Simplify the learned path.
        simplifyPath();
         
    }
}

// Now enter an infinite loop - we can re-run the maze as many
// times as we want to.
void goToFinishLine()
{
  unsigned int sensors[6];
  int i = 0;

  // Turn around if the Zumo is facing the wrong direction.
  if(path[0] == 'B') {
    turn('B');
    i++;
  }
  
  for(;i<path_length;i++) {

    followSegment();
                  
    // Drive through the intersection. 
    motors.setSpeeds(SPEED, SPEED);
    delay(OVERSHOOT(LINE_THICKNESS));
                   
    // Make a turn according to the instruction stored in
    // path[i].
    turn(path[i]);
  }
    
  // Follow the last segment up to the finish.
  followSegment();
 
  // The finish line has been reached.
  // Return and wait for another button push to
  // restart the maze.         
  reflectances.readLine(sensors);
  motors.setSpeeds(0,0);
  
  return; 
} 


// simplifyPath analyzes the path[] array and reduces all the
// turns. For example: Right turn + Right turn = (1) Back turn.
void simplifyPath()
{
  
  // only simplify the path if the second-to-last turn was a 'B'
  if(path_length < 3 || path[path_length - 2] != 'B')
  return;
   
  int total_angle = 0;
  int i;
  
  for(i = 1; i <= 3; i++) {
    switch(path[path_length - i]) {
      case 'R':
        total_angle += 90;
        break;
      case 'L':
        total_angle += 270;
        break;
      case 'B':
        total_angle += 180;
        break;
    }
  }
   
  // Get the angle as a number between 0 and 360 degrees.
  total_angle = total_angle % 360;
   
  // Replace all of those turns with a single one.
  switch(total_angle) {
    case 0:
      path[path_length - 3] = 'S';
      break;
    case 90:
      path[path_length - 3] = 'R';
      break;
    case 180:
      path[path_length - 3] = 'B';
      break;
    case 270:
      path[path_length - 3] = 'L';
      break;
  }
   
  // The path is now two steps shorter.
  path_length -= 2;
}
#endif /* MAZESOLVER */
