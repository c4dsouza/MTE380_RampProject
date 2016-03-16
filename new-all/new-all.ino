#include <PID_v1.h>
#include <NewPing.h>
#include <Adafruit_MotorShield.h>
#include "I2Cdev.h"
#include "MPU6050.h"

// Arduino Wire library is required if I2Cdev I2CDEV_ARDUINO_WIRE implementation
// is used in I2Cdev.h
#if I2CDEV_IMPLEMENTATION == I2CDEV_ARDUINO_WIRE
    #include "Wire.h"
#endif

/*
 * Initializations
 * IMU, Motors, Ultrasonic Sensors, IR Sensors and PID
 */
#define GYRO_OFFSET -7.175
#define GYRO_OFFSET_Z 89
#define SCALE_CONSTANT  400000000000
#define DOWN_RAMP_THRESHOLD -500
#define OFF_RAMP_THRESHOLD 500
#define OUTPUT_READABLE_ACCELGYRO
MPU6050 accelgyro;
int16_t ax, ay, az;
int16_t gx, gy, gz;

#define motorMaxSpeed  255
#define normalSpeed   220
#define motorMinSpeed  60
Adafruit_MotorShield AFMS = Adafruit_MotorShield(); 
Adafruit_DCMotor * mr1 = AFMS.getMotor(1);
Adafruit_DCMotor * mr2 = AFMS.getMotor(3);
Adafruit_DCMotor * ml1 = AFMS.getMotor(2);
Adafruit_DCMotor * ml2 = AFMS.getMotor(4);

#define ULTRASONIC_PIN  22
#define MAX_DISTANCE 400
NewPing sonar(ULTRASONIC_PIN, ULTRASONIC_PIN, MAX_DISTANCE);

#define numIRSensors  4
static const uint8_t sensor_pin[] = {A11,A12,A13,A14,A15};
static const uint8_t trigger_pin[] = {51,49,47,45,43};
double IRSetpoint = 2500;

#define LEVEL_SHIFTER 32

#define RED_LED 28
#define GREEN_LED 26
#define BLUE_LED 24

/*
 * Motor drive code
 */
void setLeftMotors(int direction, int speed) {
  mr1->run(direction);
  mr2->run(direction);
  mr1->setSpeed(speed);
  mr2->setSpeed(speed);
}
void setRightMotors(int direction, int speed) {
  ml1->run(direction);
  ml2->run(direction);
  ml1->setSpeed(speed);
  ml2->setSpeed(speed);
}

void pivot(bool dir, int baseSpeed, int amount){
  double pos = 0, acc;
  unsigned long lastTime = micros();

  if (dir){
    setLeftMotors(BACKWARD, baseSpeed);
    setRightMotors(FORWARD, baseSpeed);
  } else {
    setLeftMotors(FORWARD, baseSpeed);
    setRightMotors(BACKWARD, baseSpeed);
  }
  
  unsigned long now = 0;
  while(abs(pos) < amount) {
    delay(2);
    acc = accelgyro.getRotationY() + GYRO_OFFSET;
    now = micros();
    pos += acc * (now - lastTime) * (now - lastTime) / SCALE_CONSTANT;
    lastTime = now;
    Serial.println(pos);
  }

  setLeftMotors(FORWARD, 0);
  setRightMotors(FORWARD, 0);
}

void steerSoft(bool dir, int baseSpeed, int amount) {
  setLeftMotors(FORWARD, baseSpeed);
  setRightMotors(FORWARD, baseSpeed);
  
  if (dir) {
    ml2->setSpeed(constrain(baseSpeed - abs(amount), motorMinSpeed, motorMaxSpeed));
  } else {  
    mr2->setSpeed(constrain(baseSpeed - abs(amount), motorMinSpeed, motorMaxSpeed));
  }
}

void steerHard (bool dir, int baseSpeed, int amount) {
  setLeftMotors(BACKWARD, constrain(baseSpeed - amount, motorMinSpeed, motorMaxSpeed));
  setRightMotors(BACKWARD, constrain(baseSpeed + amount, motorMinSpeed, motorMaxSpeed));
//  mr2->run(FORWARD);
//  ml2->run(FORWARD);
//  mr2->setSpeed(constrain(baseSpeed + amount, motorMinSpeed, motorMaxSpeed));
//  ml2->setSpeed(constrain(baseSpeed - amount, motorMinSpeed, motorMaxSpeed));
//  mr1->run(BACKWARD);
//  ml1->run(BACKWARD);
//  mr1->setSpeed(baseSpeed);
//  ml1->setSpeed(baseSpeed);
}

void drive(bool dir, int baseSpeed, long amount, bool stopWhenDone = 1){
  long angle = 0, dist = 0, ax = 0, gy = 0;
  unsigned long lastTime = micros();
  
  if (dir){
    setLeftMotors(FORWARD, baseSpeed);
    setRightMotors(FORWARD, baseSpeed);
  } else {
    setLeftMotors(BACKWARD, baseSpeed);
    setRightMotors(BACKWARD, baseSpeed);
  }
  
  if (stopWhenDone) {
    delay(amount);
    
    setLeftMotors(FORWARD, 0);
    setRightMotors(FORWARD, 0);
  }
}

void brake(){
  setLeftMotors(FORWARD, 0);
  setRightMotors(FORWARD, 0);
}

/*
 *  IR sensor code
 */
void setupIR(){
  for (int i = 0; i < numIRSensors; i++){
    pinMode(trigger_pin[i], OUTPUT);
    digitalWrite(trigger_pin[i], HIGH);
  }
}

int readIRSensor(uint8_t sensorNumber){
//  digitalWrite(trigger_pin[sensorNumber], HIGH);
  delayMicroseconds(20);
  int measured = analogRead(sensor_pin[sensorNumber]);
//  digitalWrite(trigger_pin[sensorNumber], LOW);
  delayMicroseconds(100);
  return (abs(measured));
}

double readIRSum(){
  double IRSum = 0;
  for (int i = 0; i < numIRSensors; i++){
    IRSum += readIRSensor(i);
  }
//  Serial.println(IRSum);
  return IRSum;
}

/*
 * Section Code
 */
void findRamp() {
  int rampTime = 750;
  int sonarDistance = 0;
    
  drive(1, normalSpeed-150, 0, 0);
  setLeftMotors(FORWARD, normalSpeed-153);
  setRightMotors(FORWARD, normalSpeed-150);
  
  while(true) {
//    sonarDistance = sonar.convert_cm(sonar.ping_median(3));
    sonarDistance = sonar.ping_cm();
    Serial.println(sonarDistance);
    if(sonarDistance != 0 && sonarDistance < 250) { //TODO: tune threshold
      delay(30);
      continue;
    }
    break;
  }
  //    delay(rampTime);

  drive(0, 70,0,0);
  drive(0, 70,0,0);
  drive(0, 70,0,0);
  drive(0, 70,0,0);
  drive(0, 70,0,0);
  drive(0, 70,0,0);
  drive(0, 70,0,0);
  blinkLED(BLUE_LED, 500);
//  delay(500);

  pivot(1, 150, 90);
  drive(0, normalSpeed, 0, 0);
  drive(0, normalSpeed, 0, 0);
  drive(0, normalSpeed, 0, 0);
  drive(0, normalSpeed, 0, 0);
  drive(0, normalSpeed, 0, 0);
  drive(0, normalSpeed, 0, 0);
  drive(0, normalSpeed, 0, 0);
  drive(0, normalSpeed, 0, 0);

  blinkLED(BLUE_LED, 500);
//  delay(1000);
}

void goUpRamp() {
  double Kp_u = 0.03; double Ki_u = 0.3; double Kd_u = 0.05;
  double input, output;
    
  PID upRampPid(&input, &output, &IRSetpoint, Kp_u,Ki_u,Kd_u, REVERSE);
  upRampPid.SetSampleTime(20);
  upRampPid.SetMode(AUTOMATIC);
  upRampPid.SetOutputLimits(-100, 100);

  int state = 0;
  
  int numReadings = 25;
  int readings[numReadings];
  int readingNum = 0;
  int doneReadings = 0;
  long sum = 0;
  double average;
  int PIDed = 0;

  for(int i = 0; i < numReadings; i++) {
    readings[i] = 0;
  }
  
  while(true) {
    input = readIRSum();
    PIDed = upRampPid.Compute();
//    upRampPid.Compute();
    steerHard(0, normalSpeed, output);

    int value;
    if(PIDed) {
      ledOn(BLUE_LED);
      value = accelgyro.getAccelerationX();
      ledOff(BLUE_LED);
      sum -= readings[readingNum];
      sum += value;
      readings[readingNum] = value;
      readingNum = (readingNum+1)%numReadings;
      
      Serial.print(value); Serial.print(',');
      Serial.println(average);
      
      if(doneReadings < numReadings) {
        doneReadings++;
      } else {
        average = sum*1.0/numReadings;
        if(state == 0) {
          if(average < -4000) {
            state = 1;
          }
        } else if(state == 1) {
          if(average > -3000) {
            break;
          }
        }
      }
    }
  }
}


double posZ;
unsigned long lastTimeZ;
void resetPositionZ() {
  posZ = 0;
  lastTimeZ = micros();
}

void updatePositionZ() {
  ledOn(BLUE_LED);
  double acc = accelgyro.getRotationZ() + GYRO_OFFSET_Z;
  ledOff(BLUE_LED);
  unsigned long now = micros();
  unsigned int dt = (unsigned int)(now - lastTimeZ);
  posZ += acc * dt * dt / SCALE_CONSTANT;
//  posZ += acc * (now - lastTimeZ) * (now - lastTimeZ) / SCALE_CONSTANT;
  lastTimeZ = now;
}


#define Z_ANGLE_THRESHOLD 8
void findPost(){
  unsigned long startTime = millis();
  int sonarPing = 0, sonarDist = 0; int postDetected = 175; int postThreshold = 1500; int inclineCount = 0;
  bool postFound = false; bool detected = false;

  int state = 0;
  
  int numReadings = 25;
  int readings[numReadings];
  int readingNum = 0;
  int doneReadings = 0;
  long sum = 0;
  double average;
  int PIDed = 0;

  for(int i = 0; i < numReadings; i++) {
    readings[i] = 0;
  }
  
  blinkLED(BLUE_LED,250);
  drive(0, normalSpeed, 400, 1);
  pivot(1, 80, 80);
  drive(1, normalSpeed, 0, 0);
  delay(1000);



  int readTimes = 0;
  
  while(!postFound){
    
    int value;
    if(true) {
      ledOn(BLUE_LED);
      value = accelgyro.getAccelerationX();
      ledOff(BLUE_LED);
      sum -= readings[readingNum];
      sum += value;
      readings[readingNum] = value;
      readingNum = (readingNum+1)%numReadings;

      Serial.print(value); Serial.print(',');
      Serial.println(average);
      
      if(doneReadings < numReadings) {
        doneReadings++;
      } else {
        average = sum*1.0/numReadings;
        if(average < -2000) {
            return;
        }
      }
    }    
//    if(posZ >= Z_ANGLE_THRESHOLD) {
//      return;
//    }
    if(readTimes++ < 6) {
      delay(5);
      continue;
    }
    readTimes = 0;
    sonarDist = sonar.convert_cm(sonar.ping_median(4));
    Serial.println(sonarDist);
    if (sonarDist < postDetected){
      if (!detected) {
        detected = true;
        startTime = millis();
      } else if ((millis() - startTime) > map(sonarDist, 0, 200, 0, 2)*postThreshold) { 
        postFound = true;
        break;
      }
    } else {
      detected = false;
    }
//    delay(30);
  }
  
  pivot(0, 80, 90);
  drive(1, normalSpeed, 0, 0);
  delay(500);
//  resetPositionZ();

  while(true){
    int value;
    if(true) {
      ledOn(BLUE_LED);
      value = accelgyro.getAccelerationX();
      ledOff(BLUE_LED);
      sum -= readings[readingNum];
      sum += value;
      readings[readingNum] = value;
      readingNum = (readingNum+1)%numReadings;

      Serial.print(value); Serial.print(',');
      Serial.println(average);
      
      if(doneReadings < numReadings) {
        doneReadings++;
      } else {
        average = sum*1.0/numReadings;
        if(average < -2000) {
            return;
        }
      }
    }
  }
  
  brake();
}

void setupLED() {
  pinMode(RED_LED, OUTPUT);
  pinMode(GREEN_LED, OUTPUT);
  pinMode(BLUE_LED, OUTPUT);
  digitalWrite(RED_LED, HIGH);
  digitalWrite(GREEN_LED, HIGH);
  digitalWrite(BLUE_LED, HIGH);
}

void blinkLED(int led, int timeOn) {
  digitalWrite(led, LOW);
  delay(timeOn);
  digitalWrite(led, HIGH);
}

void loopBlink(int led, int timeOn, int times) {
  for(int i = 0; i < times; i++) {
    blinkLED(led, timeOn);
    delay(timeOn);
  }
}

void ledOn(int led) {
  digitalWrite(RED_LED, HIGH);
  digitalWrite(GREEN_LED, HIGH);
  digitalWrite(BLUE_LED, HIGH);
  digitalWrite(led, LOW);
}

void ledOff(int led) {
  digitalWrite(led, HIGH);
}

void setup() {
  Serial.begin(38400);

  pinMode(30, OUTPUT);
  digitalWrite(30, HIGH);

  //IMU
  // join I2C bus (I2Cdev library doesn't do this automatically)
  #if I2CDEV_IMPLEMENTATION == I2CDEV_ARDUINO_WIRE
      Wire.begin();
  #elif I2CDEV_IMPLEMENTATION == I2CDEV_BUILTIN_FASTWIRE
      Fastwire::setup(400, true);
  #endif
  accelgyro.initialize();
  accelgyro.setYGyroOffset(6);

  //IR
  setupIR();

  //debug LED
  setupLED();

  //Motors
  AFMS.begin();

  //No setup needed for ultrasonic

  // Main run code
  blinkLED(RED_LED, 500);
  blinkLED(GREEN_LED, 500);
  blinkLED(BLUE_LED, 500);
  findRamp();
  goUpRamp();
  brake();
  findPost();
  brake();
  blinkLED(RED_LED, 500);
  blinkLED(GREEN_LED, 500);
  blinkLED(BLUE_LED, 500);
  blinkLED(RED_LED, 500);
  blinkLED(GREEN_LED, 500);
  blinkLED(BLUE_LED, 500);
  blinkLED(RED_LED, 500);
  blinkLED(GREEN_LED, 500);
  blinkLED(BLUE_LED, 500);
}

void loop() {

}
