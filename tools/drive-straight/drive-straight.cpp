#include "drive-straight.h"

double Setpoint = 0, Input, Output;
PID drivePID(&Input, &Output, &Setpoint, Kp_cons,Ki_cons,Kd_cons, REVERSE);

Adafruit_MotorShield AFMS = Adafruit_MotorShield(); 

DriveStraight::DriveStraight(RobotContext *context) {
  rb = context;
  doneDriving = 1;
  driveTime = 0;
  aPosition = 0;
}

void DriveStraight::setup(int direction, int time, int speed) {
  if(!doneDriving) {
    return;
  }
  doneDriving = 0;
  aPosition = 0;
  driveDirection = direction;
  driveTime = time;
  startTime = millis();
  driveSpeed = speed;
  lastUpdate = micros();
}

/*
* Returns 1 if no problem
* Return 0 when done drivingTime
* Return -1 for special cases
*/
int DriveStraight::update() {
  unsigned int now = micros();
  unsigned int dt = (unsigned int)(now - lastUpdate);
  if(dt/1000 >= driveTime) {
    return 0;
  }
  if(dt < 1500000) {
    return 1;
  }
  updateAngularPosition();
  Input = aPosition;
  drivePID.Compute();
  lastUpdate = now;
  driveMotors(Output);
  return 1;
}

double DriveStraight::updateAngularPosition(unsigned int dt){
  
  double acc;
  
  acc = accelgyro.getRotationY() + GYRO_OFFSET;
  aPosition += acc * dt * dt / (400000000000);
  
}

void DriveStraight::driveMotors(double drive){
  int driveOutput = drive;
  Serial.println(driveOutput);
  rb->ml1->setSpeed(constrain(driveSpeed+drive, driveSpeed-50, driveSpeed+50));
  rb->ml2->setSpeed(constrain(driveSpeed+drive, driveSpeed-50, driveSpeed+50));
  rb->ml1->run(driveDirection);
  rb->ml2->run(driveDirection);
  
  rb->mr1->setSpeed(constrain(driveSpeed-drive, driveSpeed-50, driveSpeed+50));
  rb->mr2->setSpeed(constrain(driveSpeed-drive, driveSpeed-50, driveSpeed+50));
  rb->mr1->run(driveDirection);
  rb->mr2->run(driveDirection);
}

