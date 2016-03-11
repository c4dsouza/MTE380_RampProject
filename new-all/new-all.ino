#include <PID_v1.h>
#include <NewPing.h>
#include <Adafruit_MotorShield.h>
#include "I2Cdev.h"
#include "MPU6050.h"
#include "Wire.h"

/*
 * Initializations
 * IMU, Motors, Ultrasonic Sensors, IR Sensors and PID
 */
#define GYRO_OFFSET -7.175
#define YAW_CONSTANT  400000000000
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

#define numIRSensors  5
static const uint8_t sensor_pin[] = {A11,A12,A13,A14,A15};
static const uint8_t trigger_pin[] = {51,49,47,45,43};
double IRSetpoint = 2500;

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
  double pos, acc;
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
    pos += acc * (now - lastTime) * (now - lastTime) / YAW_CONSTANT;
    lastTime = now;
//    Serial.println(pos);
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
  setLeftMotors(FORWARD, baseSpeed);
  setRightMotors(FORWARD, baseSpeed);
  
  mr2->setSpeed(constrain(baseSpeed + amount, motorMinSpeed, motorMaxSpeed));
  ml2->setSpeed(constrain(baseSpeed - amount, motorMinSpeed, motorMaxSpeed));
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
    digitalWrite(trigger_pin[i], LOW);
  }
}

int readIRSensor(uint8_t sensorNumber){
  digitalWrite(trigger_pin[sensorNumber], HIGH);
  delayMicroseconds(20);
  int measured = analogRead(sensor_pin[sensorNumber]);
  digitalWrite(trigger_pin[sensorNumber], LOW);
  delayMicroseconds(100);
  return (abs(measured));
}

double readIRSum(){
  double IRSum = 0;
  for (int i = 0; i < numIRSensors; i++){
    IRSum += readIRSensor(i);
  }
  return IRSum;
}

/*
 * Section Code
 */
void findRamp() {
  int ramp, rampTime = 0;
  int rampUpperThreshold = 220; //TODO: tune threshold
  int rampLowerThreshold = 100; //TODO: tune threshold
  int rampTimeThreshold = 200; //TODO: tune threshold
  int sonarDistance = 0;
    
  drive(1, normalSpeed, 0, 0);
  
  while(true) {
    sonarDistance = sonar.convert_cm(sonar.ping_median(5));
    if(sonarDistance > rampUpperThreshold && !ramp) { //TODO: tune threshold
      ramp = 1;
      rampTime = millis();
    }
    
    if(sonarDistance < rampLowerThreshold && ramp) { //TODO: tune threshold
      ramp = 0;
    }

    if(ramp && ((unsigned int)(millis() - rampTime)) >= rampTimeThreshold) { //TODO: tune threshold
      pivot(0, normalSpeed, 90);
      drive(1, normalSpeed, 0, 0);
      return;
    }
    
    delay(50);
  }
}

void goUpRamp() {
  double Kp_u = 0.03; double Ki_u = 0; double Kd_u = 0.05;
  double input, output;
    
  PID upRampPid(&input, &output, &IRSetpoint, Kp_u,Ki_u,Kd_u, REVERSE);
  upRampPid.SetSampleTime(10);
  upRampPid.SetMode(AUTOMATIC);
  upRampPid.SetOutputLimits(-100, 100);
  
  while(true) {
    input = readIRSum();
    upRampPid.Compute();
    steerHard(0, normalSpeed, output);

    if (accelgyro.getAccelerationX() > DOWN_RAMP_THRESHOLD) {
      brake();
      return;
    }
  }
}

void goDownRamp(){
  double Kp_d = 0.9; double Ki_d = 0.1; double Kd_d = 0.9; //TODO: tune k vals
  double input, output;
  PID downRampPid(&input, &output, &IRSetpoint, Kp_d,Ki_d,Kd_d, REVERSE);
  downRampPid.SetSampleTime(10);
  downRampPid.SetMode(AUTOMATIC);
  downRampPid.SetOutputLimits(-30, 30);
  
  while(true) {
    input = readIRSum();
    downRampPid.Compute();
    if (output < 0) {
      steerSoft(1, 60, output);
    } else {
      steerSoft(0, 60, output);
    }

    if (accelgyro.getAccelerationX() < OFF_RAMP_THRESHOLD) {
      drive(1, normalSpeed, 5, 1); //TODO: tune distance
      return;
    }
  }
}

void findPost(){
  unsigned long startTime = millis();
  int sonarPing = 0, sonarDist = 0; int postDetected = 130; int postThreshold = 2500; int inclineCount = 0;
  bool postFound = false; bool detected = false;
  double ax = 1; double axBase = 0;
double pos, acc;
  unsigned long lastTime = micros();
  
  pivot(0, 80, 90);
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
    pos += acc * (now - lastTime) * (now - lastTime) / YAW_CONSTANT;
    lastTime = now;
//    Serial.println(pos);
  }

  while(true){
    delay(10);
    double ax = accelgyro.getAccelerationX();
    Serial.println(ax);
    if (ax < -1000){
      if (inclineCount++ > 50){
        break;
      }
    }
  }
  brake();
}


void setup() {
  Serial.begin(38400);

  //IMU
  Wire.begin();
  accelgyro.initialize();
  accelgyro.setYGyroOffset(6);

  //IR
  setupIR();

  //Motors
  AFMS.begin();

  //No setup needed for ultrasonic

  // Main run code
//  findRamp();
//  goUpRamp();
//  goDownRamp();
  findPost();

}

void loop() {

}
