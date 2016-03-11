#include "RobotContext.h"

RobotContext::RobotContext(MPU6050 *g, Adafruit_DCMotor *Mr1, Adafruit_DCMotor *Mr2, Adafruit_DCMotor *Ml1, Adafruit_DCMotor *Ml2)
{
  gyro = g;
  mr1 = Mr1;
  mr2 = Mr2;
  ml1 = Ml1;
  ml2 = Ml2;
}
