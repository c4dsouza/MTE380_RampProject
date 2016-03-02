#include <Adafruit_MotorShield.h>

#define leftMotorBase 128
#define rightMotorBase  128
#define MotorMax 150
#define TIME_PER_DEGREE 9.3

Adafruit_MotorShield myDriveTrain = Adafruit_MotorShield();

Adafruit_DCMotor * leftMotor = myDriveTrain.getMotor(2);
Adafruit_DCMotor * rightMotor = myDriveTrain.getMotor(1);

void setup() {
  Serial.begin(9600);
  myDriveTrain.begin();
}

void loop() {
  turnAngle(90, 0);
  delay(1000);
  turnAngle(90, 1);
  delay(1000);
}

void turnAngle(uint8_t angle, bool dir){
  leftMotor->setSpeed(MotorMax);
  rightMotor->setSpeed(MotorMax);

  if (dir){
    leftMotor->run(FORWARD);
    rightMotor->run(BACKWARD);
  } else {
    leftMotor->run(BACKWARD);
    rightMotor->run(FORWARD);
  }

  delay(long(TIME_PER_DEGREE*angle));

  leftMotor->setSpeed(0);
  rightMotor->setSpeed(0);
}

