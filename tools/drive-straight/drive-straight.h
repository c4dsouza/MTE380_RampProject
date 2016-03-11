#include <PID_v1.h>
#include <Adafruit_MotorShield.h>

#define Kp_cons  0.03
#define Ki_cons  0
#define Kd_cons  0.01

#define GYRO_OFFSET -7.175

#if I2CDEV_IMPLEMENTATION == I2CDEV_ARDUINO_WIRE
    #include "Wire.h"
#endif

class DriveStraight
{
  public:
    DriveStraight(RobotContext *context);
    void setup(int time, int speed);
    int update();
  private:
    RobotContext *rb;
    int doneDriving;
    int driveTime;
    int driveSpeed;
    int driveDirection;
    unsigned int lastUpdate;
    unsigned int startTime;
    double aPosition;
    void driveMotors(double c);
    double updateAngularPosition();
};