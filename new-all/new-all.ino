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
#define ACC_OFFSET  0
#define YAW_CONSTANT  400000000000
#define X_TRANSLATION_CONSTANT  400000000000
#define OUTPUT_READABLE_ACCELGYRO
MPU6050 accelgyro;
int16_t ax, ay, az;
int16_t gx, gy, gz;

#define motorMaxSpeed  255
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
    Serial.println(pos);
  }

  setLeftMotors(FORWARD, 0);
  setRightMotors(FORWARD, 0);
}

void steer(bool dir, int baseSpeed, int amount) {
  setLeftMotors(FORWARD, baseSpeed);
  setRightMotors(FORWARD, baseSpeed);
  
  if (dir) {
    ml2->setSpeed(constrain(baseSpeed - abs(amount), motorMinSpeed, motorMaxSpeed));
  } else {  
    mr2->setSpeed(constrain(baseSpeed - abs(amount), motorMinSpeed, motorMaxSpeed));
  }
}

void drive(bool dir, int baseSpeed, int amount, bool stopWhenDone = 0){
  double pos, acc;
  unsigned long lastTime = micros();
  
  if (dir){
    setLeftMotors(FORWARD, baseSpeed);
    setRightMotors(FORWARD, baseSpeed);
  } else {
    setLeftMotors(BACKWARD, baseSpeed);
    setRightMotors(BACKWARD, baseSpeed);
  }
  
  if (stopWhenDone) {
    unsigned long now = 0;
    while(abs(pos) < amount) {
      delay(2);
      acc = accelgyro.getAccelerationX() + ACC_OFFSET;
      now = micros();
      pos += acc * (now - lastTime) * (now - lastTime) / X_TRANSLATION_CONSTANT;
      lastTime = now;
      Serial.println(pos);
    }
  
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
  int rampThreshold = 220;
  int rampTimeThreshold = 200;
    
  drive(true, 200, 0, 0);
  
  int reading = 0;
  while(true) {
    reading = readUltrasonic();
    if(reading > rampThreshold && !ramp) {
      ramp = 1;
      rampTime = millis();
    }
    if(reading <= rampThreshold) {
      ramp = 0;
    }
    if(ramp && ((unsigned int)(millis() - rampTime)) >= rampTimeThreshold) {
      turnAngle(90, 0);
      setLeftMotors(FORWARD, 200);
      setRightMotors(FORWARD, 200);
      delay(1000);
      return;
    }
    Serial.println(reading);
    delay(50);
  }
}

void goUpRamp() {
  double Kp_u = 0.03; Ki_u = 0; Kd_u = 0.05;
  double input, output;
    
  PID upRampPid(&input, &output, &IRSetpoint, Kp_u,Ki_u,Kd_u, REVERSE);
  upRampPid.SetSampleTime(10);
  upRampPid.SetMode(AUTOMATIC);
  upRampPid.SetOutputLimits(-100, 100);
  
  while(true) {
    Input = readIRSum();
    upRampPid.Compute();
    pivot(output);
  }
}
void goUpRamp(){
  double input, output, setpoint;
  PID upRampPid(&input, &output, &setpoint, Kp_u,Ki_u,Kd_u, REVERSE);

  
}

void goDownRamp(){
  double input, output, setpoint;
  PID downRampPid(&input, &output, &setpoint, Kp_d,Ki_d,Kd_d, REVERSE);
}

void findPost(){
  
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

}

void loop() {

}
