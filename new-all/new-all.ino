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

#define RED_LED 44
#define GREEN_LED 46
#define BLUE_LED 48

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
  delay(500);

  pivot(1, 150, 90);
  drive(0, normalSpeed, 0, 0);
  drive(0, normalSpeed, 0, 0);
  drive(0, normalSpeed, 0, 0);
  drive(0, normalSpeed, 0, 0);
  drive(0, normalSpeed, 0, 0);
  drive(0, normalSpeed, 0, 0);
  drive(0, normalSpeed, 0, 0);
  drive(0, normalSpeed, 0, 0);

  delay(1000);
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
      value = accelgyro.getAccelerationX();
      sum -= readings[readingNum];
      sum += value;
      readings[readingNum] = value;
      readingNum = (readingNum+1)%numReadings;
    
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

void findPost(){
  unsigned long startTime = millis();
  int sonarPing = 0, sonarDist = 0; int postDetected = 130; int postThreshold = 1500; int inclineCount = 0;
  bool postFound = false; bool detected = false;
  double acc = 0; double pos = 0;
  unsigned long lastTime = micros();

  blinkLED(BLUE_LED,250);
  drive(0, normalSpeed, 400, 1);
  pivot(1, 80, 75);
  drive(1, normalSpeed, 0, 0); 

  while(!postFound){
    sonarDist = sonar.convert_cm(sonar.ping_median(4));
    Serial.println(sonarDist);
    if (sonarDist < postDetected){
      if (!detected) {
        detected = true;
        startTime = millis();
      } else if ((millis() - startTime) > postThreshold) { 
        postFound = true;
        break;
      }
    } else {
      detected = false;
    }
    delay(30);
  }
  
  pivot(0, 80, 90);
  drive(1, normalSpeed, 0, 0);
  delay(100);
  
  unsigned long now = 0;
  unsigned int dt = 0;
  lastTime = micros();
  while(pos < 3) {
    delay(2);
    acc = accelgyro.getRotationZ() + GYRO_OFFSET_Z;
    now = micros();
    dt = (unsigned int)(now - lastTime);
    pos += acc * dt * dt / SCALE_CONSTANT;
    lastTime = now;
    Serial.println(pos);
  }
  
  brake();
}

void setupLED() {
  pinMode(RED_LED, OUTPUT);
  pinMode(GREEN_LED, OUTPUT);
  pinMode(BLUE_LED, OUTPUT);
  digitalWrite(RED_LED, LOW);
  digitalWrite(GREEN_LED, LOW);
  digitalWrite(BLUE_LED, LOW);
}

void blinkLED(int led, int timeOn) {
  digitalWrite(led, HIGH);
  delay(timeOn);
  digitalWrite(led, LOW);
}

void loopBlink(int led, int timeOn, int times) {
  for(int i = 0; i < times; i++) {
    blinkLED(led, timeOn);
    delay(timeOn);
  }
}

void setup() {
  Serial.begin(38400);

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
  findRamp();
  goUpRamp();
  brake();
  findPost();
  loopBlink(BLUE_LED, 500, 5);
//  findPost();
}

void loop() {

}
