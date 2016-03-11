#include <PID_v1.h>
#include <Adafruit_MotorShield.h>
#include "I2Cdev.h"
#include "MPU6050.h"
#if I2CDEV_IMPLEMENTATION == I2CDEV_ARDUINO_WIRE
    #include "Wire.h"
#endif

class RobotContext
{
  public:
    RobotContext(MPU6050 *gyro, Adafruit_DCMotor *mr1, Adafruit_DCMotor *mr2, Adafruit_DCMotor *ml1, Adafruit_DCMotor *ml2);
    MPU6050 *gyro;
    Adafruit_DCMotor *mr1;
    Adafruit_DCMotor *mr2;
    Adafruit_DCMotor *ml1;
    Adafruit_DCMotor *ml2;
  private:
};