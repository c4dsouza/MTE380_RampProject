#include <Adafruit_MotorShield.h>


#define leftMotorBase 128
#define rightMotorBase  128
#define MotorMax 150
#define TIME_PER_DEGREE 9.4

Adafruit_MotorShield AFMS = Adafruit_MotorShield(); 

Adafruit_DCMotor *ml = AFMS.getMotor(2);
Adafruit_DCMotor *mr = AFMS.getMotor(1);

int scanTime = 30000;

int scanVals[10000];
int numScanVals = 0;

int ultrasonic = 22;

void turnAngle(uint8_t angle, bool dir){
  ml->setSpeed(MotorMax);
  mr->setSpeed(MotorMax);

  if (dir){
    ml->run(FORWARD);
    mr->run(BACKWARD);
  } else {
    ml->run(BACKWARD);
    mr->run(FORWARD);
  }

  delay(long(TIME_PER_DEGREE*angle));

  ml->setSpeed(0);
  mr->setSpeed(0);
}

void setup() {
  Serial.begin(9600);
  AFMS.begin();
  pinMode(ultrasonic, OUTPUT);
  digitalWrite(ultrasonic, LOW);
  mr->run(FORWARD);
  mr->setSpeed(200);
  ml->run(FORWARD);
  ml->setSpeed(215);
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

int ramp = 0;
unsigned int rampTime = 0;
int rampThreshold = 220;
int rampTimeThreshold = 300;
void loop() {
  int reading = readUltrasonic();
  if(reading > rampThreshold && !ramp) {
    ramp = 1;
    rampTime = millis();
  }
  if(reading <= rampThreshold) {
    ramp = 0;
  }
  if(ramp && ((unsigned int)(millis() - rampTime)) >= rampTimeThreshold) {
    turnAngle(90, 0);
    ml->setSpeed(0);
    mr->setSpeed(0);
    while(true) {};
  }
  Serial.println(reading);
  delay(50);
}
