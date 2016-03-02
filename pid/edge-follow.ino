// PID library
class PID
{
  public:
    PID();
    void tune(double p, double i, double d);
    void setTimeslice(int t);
    void getInputFunction(double (*a)());
    void getOutputFunction(double (*a)());
    void onControlSignal(void (*a)(double c));
    void delayRefresh();
    int refresh();
  private:
    double kp;
    double kd;
    double ki;
    double dt; // IN MICROS
    double prevError;
    double errorSum;
    int timestamp;
    unsigned int timeDifference(int now, int then);
    double (*input)();
    double (*output)();
    void (*control)(double c);
};

PID::PID()
{
  kp = 1;
  kd = 0;
  ki = 0;
  dt = 1000;
  timestamp = micros();
  prevError = 0;
  errorSum = 0;
}

void PID::tune(double p, double i, double d) {
  kp = p;
  kd = d;
  ki = i;
}

void PID::setTimeslice(int t) {
  dt = t;
}

int PID::refresh() {
  unsigned int now = micros();
  unsigned int delta = timeDifference(now, timestamp);
  if(delta < dt) {
    return 0;
  }

  double error = input() - output();
  double c = kp*error + kd*(error-prevError)/delta*1000000 + ki*error*delta/1000000;
  prevError = error;

  control(c);

  timestamp = now;

  return 1;
}

void PID::delayRefresh() {
  unsigned int delta = timeDifference(micros(), timestamp);
  delayMicroseconds(delta);
  while(!refresh()){};
}

unsigned int PID::timeDifference(int now, int then) {
  return ((unsigned int)now-then);
}

void PID::getInputFunction(double (*a)()) {
  input = a;
}

void PID::getOutputFunction(double (*a)()) {
  output = a;
}

void PID::onControlSignal(void (*a)(double c)) {
  control = a;
}
// END PID LIBRARY

#include <Adafruit_MotorShield.h>

Adafruit_MotorShield AFMS = Adafruit_MotorShield(); 

Adafruit_DCMotor *ml = AFMS.getMotor(2);
Adafruit_DCMotor *mr = AFMS.getMotor(1);

int leftSensor = A8;
int middleSensor = A9;
int rightSensor = A10;
int leftSensorLight = 50;
int middleSensorLight = 51;
int rightSensorLight = 52;

double targetVal = 0;

double targetValue() {
  return targetVal;
  //return 25 + 965 + (25+965)/2;
}

double readIRSensors() {
  digitalWrite(leftSensorLight, HIGH);
  float left = analogRead(leftSensor);
  digitalWrite(leftSensorLight, LOW);
  digitalWrite(middleSensorLight, HIGH);
  float middle = analogRead(middleSensor);
  digitalWrite(middleSensorLight, LOW);
  digitalWrite(rightSensorLight, HIGH);
  float right = analogRead(rightSensor);
  digitalWrite(rightSensorLight, LOW);
  
  left -= analogRead(leftSensor);
  middle -= analogRead(middleSensor);
  right -= analogRead(rightSensor);
  return left + middle + right;
}

void changeMotorSpeeds(double c) {
  int speedL = 128 + c;
  int speedR = 128 - c;
  if(speedL < 64) speedL = 64;
  if(speedL > 192) speedL = 192;
  if(speedR < 64) speedR = 64;
  if(speedR > 192) speedR = 192;

  ml->setSpeed(speedL);
  ml->run(FORWARD);
  mr->setSpeed(speedR);
  mr->run(FORWARD);
}

PID pid;

void setup() {
  Serial.begin(9600);

  pinMode(leftSensorLight, OUTPUT);
  pinMode(middleSensorLight, OUTPUT);
  pinMode(rightSensorLight, OUTPUT);

  digitalWrite(leftSensorLight, LOW);
  digitalWrite(middleSensorLight, LOW);
  digitalWrite(rightSensorLight, LOW);

  pid.getInputFunction(targetValue);
  pid.getOutputFunction(readIRSensors);
  pid.onControlSignal(changeMotorSpeeds);

  pid.setTimeslice(5000); // 5 Milliseconds
  pid.tune(0.15, 0.1, 0.2);

  targetVal = readIRSensors();
  Serial.println("CALIBRATED");
  delay(5000);

  AFMS.begin();
  changeMotorSpeeds(0);
  Serial.println("set motor speeds");

}

void loop() {
  pid.delayRefresh();
}

