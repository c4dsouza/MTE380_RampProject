#include <Adafruit_MotorShield.h>
#include <PID_v1.h>
// I2Cdev and MPU6050 must be installed as libraries, or else the .cpp/.h files
// for both classes must be in the include path of your project
#include "I2Cdev.h"
#include "MPU6050.h"

#define leftMotorBase 128
#define leftMotorMax 200
#define leftMotorMin 40
#define rightMotorBase  128
#define rightMotorMax  200
#define rightMotorMin  40
#define MotorMax 150
#define Kp_cons  0.03
#define Ki_cons  0
#define Kd_cons  0.01
#define numSensors  5
#define numCalibrations 10

#define GYRO_OFFSET -7.175

#define OUTPUT_READABLE_ACCELGYRO

#if I2CDEV_IMPLEMENTATION == I2CDEV_ARDUINO_WIRE
    #include "Wire.h"
#endif

MPU6050 accelgyro;
//MPU6050 accelgyro(0x69); // <-- use for AD0 high

int16_t ax, ay, az;
int16_t gx, gy, gz;


Adafruit_MotorShield AFMS = Adafruit_MotorShield(); 

Adafruit_DCMotor * mr1 = AFMS.getMotor(1);
Adafruit_DCMotor * mr2 = AFMS.getMotor(3);
Adafruit_DCMotor * ml1 = AFMS.getMotor(2);
Adafruit_DCMotor * ml2 = AFMS.getMotor(4);

int scanTime = 30000;

int scanVals[10000];
int numScanVals = 0;

int ultrasonic = 22;

static const uint8_t sensor_pin[] = {A11,A12,A13,A14,A15};
static const uint8_t trigger_pin[] = {51,49,47,45,43};

double Setpoint, Input, Output;
PID myPID(&Input, &Output, &Setpoint, Kp_cons,Ki_cons,Kd_cons, REVERSE);

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
  
  delay(2000);
}

void driveMotors(double drive){
  int driveOutput = drive;
  Serial.println(driveOutput);
  setLeftMotors(FORWARD, constrain(leftMotorBase-driveOutput, leftMotorMin, leftMotorMax));
  setRightMotors(FORWARD, constrain(rightMotorBase+driveOutput, rightMotorMin, rightMotorMax));
}

double readUltrasonic() {
  pinMode(ultrasonic, OUTPUT);
  digitalWrite(ultrasonic, HIGH);
  delayMicroseconds(10);
  digitalWrite(ultrasonic, LOW);
  pinMode(ultrasonic, INPUT);
  double duration = 1.0*pulseIn(ultrasonic, HIGH);
  duration /= 58; //put into cm
  return duration;
}

double medianUltrasonic() {
  int numSamples = 3;
  int samples[3];
  double average = 0;
  for(int i = 0; i < numSamples; i++) {
    average += readUltrasonic()/numSamples;
  }
  return average;
}

void findRamp() {
  int ramp = 0;
  unsigned int rampTime = 0;
  int rampThreshold = 220;
  int rampTimeThreshold = 200;
  
  setLeftMotors(FORWARD, 200);
  setRightMotors(FORWARD, 200);
  
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
      Serial.println("TURNING");
      turnAngle(90, 0);
      Serial.println("STOPPING");
      setLeftMotors(FORWARD, 200);
      setRightMotors(FORWARD, 200);
      delay(1000);
      return;
    }
    Serial.println(reading);
    delay(50);
  }
}

void followRamp() {
  delay(1000);
  setupIR();
  setLeftMotors(FORWARD, 200);
  setRightMotors(FORWARD, 200);
  Input = readIRSum();
  myPID.SetSampleTime(10);
  myPID.SetMode(AUTOMATIC);
  myPID.SetOutputLimits(-100, 100);
  
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
  #if I2CDEV_IMPLEMENTATION == I2CDEV_ARDUINO_WIRE
      Wire.begin();
  #elif I2CDEV_IMPLEMENTATION == I2CDEV_BUILTIN_FASTWIRE
      Fastwire::setup(400, true);
  #endif
  accelgyro.initialize();
  Serial.println(accelgyro.testConnection() ? "MPU6050 connection successful" : "MPU6050 connection failed");
  accelgyro.setYGyroOffset(6);
  AFMS.begin();
  pinMode(ultrasonic, OUTPUT);
  digitalWrite(ultrasonic, LOW);
  turnAngle(90,0);
  delay(2000);
  turnAngle(90,0);
  delay(2000);
  turnAngle(90,0);
  delay(2000);
  turnAngle(90,0);
//  findRamp();
//  followRamp();
}

void loop() {
  
}
