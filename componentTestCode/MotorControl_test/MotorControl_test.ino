#include <Adafruit_MotorShield.h>

#define leftMotorBase 128
#define rightMotorBase  128
#define MotorMax 200
#define TIME_PER_DEGREE 10 //Change me

Adafruit_MotorShield myDriveTrain = Adafruit_MotorShield();

Adafruit_DCMotor * leftMotor = myDriveTrain.getMotor(2);
Adafruit_DCMotor * rightMotor = myDriveTrain.getMotor(1);

void setup() {
  Serial.begin(9600);
  myDriveTrain.begin();
}

void loop() {

}

void turnAngle(uint8_t angle, bool dir){
  leftMotor->setSpeed(MotorMax);
  rightMotor->setSpeed(MotorMax);

  if (dir){
    leftMotor->run(FORWARD);
    rightMotor->run(REVERSE);
  } else {
    leftMotor->run(REVERSE);
    rightMotor->run(FORWARD);
  }

  delayMicroseconds(TIME_PER_DEGREE*angle);
}

void driveMotors(double drive){
  int driveOutput = drive;
  Serial.println(driveOutput);
  leftMotor->setSpeed(constrain(leftMotorBase+driveOutput, leftMotorMin, leftMotorMax));
  leftMotor->run(FORWARD);
  
  rightMotor->setSpeed(constrain(rightMotorBase-driveOutput, rightMotorMin, rightMotorMax));
  rightMotor->run(FORWARD);
}

