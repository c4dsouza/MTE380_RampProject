#include <Adafruit_MotorShield.h>
#include <PID_v1.h>
#include "I2Cdev.h"
#include "MPU6050.h"
#include "Wire.h"

#define motorBaseSpeed 60
#define motorMaxSpeed 255
#define motorMinSpeed 0
#define MotorMax motorMaxSpeed

#define Kp  0.9
#define Ki  0.1
#define Kd  0.9

#define numSensors  5
#define numCalibrations 10

#define GYRO_OFFSET -7.175

#define OUTPUT_READABLE_ACCELGYRO

MPU6050 accelgyro;
int16_t ax, ay, az;
int16_t gx, gy, gz;

Adafruit_MotorShield AFMS = Adafruit_MotorShield(); 
Adafruit_DCMotor * mr1 = AFMS.getMotor(1);
Adafruit_DCMotor * mr2 = AFMS.getMotor(3);
Adafruit_DCMotor * ml1 = AFMS.getMotor(2);
Adafruit_DCMotor * ml2 = AFMS.getMotor(4);

static const uint8_t sensor_pin[] = {A11,A12,A13,A14,A15};
static const uint8_t trigger_pin[] = {51,49,47,45,43};

double Setpoint, Input, Output;
PID myPID(&Input, &Output, &Setpoint, Kp,Ki,Kd, REVERSE);

void turnAngle(int angle, int dir){
  double position = 0;
  double acc;
  unsigned long lastTime = micros();

  setLeftMotors(FORWARD, MotorMax);
  setRightMotors(FORWARD, MotorMax);

  if (dir){
    setLeftMotors(BACKWARD, MotorMax);
    setRightMotors(FORWARD, MotorMax);
  } else {
    setLeftMotors(FORWARD, MotorMax);
    setRightMotors(BACKWARD, MotorMax);
  }
  
  unsigned long now = 0;
  while(abs(position) < angle) {
    delay(2);
    acc = accelgyro.getRotationY() + GYRO_OFFSET;
    now = micros();
    position += acc * (now - lastTime) * (now - lastTime) / (400000000000);
    lastTime = now;
    Serial.println(position);
  }
  
  setLeftMotors(FORWARD, 0);
  setRightMotors(FORWARD, 0);
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
  for (int i = 0; i < numSensors; i++){
    IRSum += readIRSensor(i);
  }
  return IRSum;
}

void setupIR(){
  for (int i = 0; i < numSensors; i++){
    pinMode(trigger_pin[i], OUTPUT);
    digitalWrite(trigger_pin[i], LOW);
  }

  Setpoint = 0;
  for (int i = 0; i < numCalibrations; i++){
    Setpoint += readIRSum();
    Serial.println(readIRSum());
    delay(250);
  }
  Setpoint = Setpoint/numCalibrations;
  Serial.print("Calibrated: ");
  Serial.println(Setpoint);
}

void driveMotors(double drive){
  setLeftMotors(FORWARD, motorBaseSpeed);
  setRightMotors(FORWARD, motorBaseSpeed);
  if (drive < 0) {
    ml2->setSpeed(constrain(motorBaseSpeed - abs(drive), motorMinSpeed, motorMaxSpeed));
  } else {  
    mr2->setSpeed(constrain(motorBaseSpeed - abs(drive), motorMinSpeed, motorMaxSpeed));
  }
}

void followRamp() {
  setupIR();
  delay(1000);
  
  setLeftMotors(FORWARD, motorBaseSpeed);
  setRightMotors(FORWARD, motorBaseSpeed);
  Input = readIRSum();
  myPID.SetSampleTime(10);
  myPID.SetMode(AUTOMATIC);
  myPID.SetOutputLimits(-25, 25);
  
  while(true) {
    Input = readIRSum();
    myPID.Compute();
    driveMotors(Output);
  }
}

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

void setup() {
  Serial.begin(38400);
  Wire.begin();
  accelgyro.initialize();
  accelgyro.setYGyroOffset(6);
  AFMS.begin();
  followRamp();
}

void loop() {
}
