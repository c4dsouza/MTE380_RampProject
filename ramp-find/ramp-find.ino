#include <Adafruit_MotorShield.h>


#define TURN_TIME 1200

Adafruit_MotorShield AFMS = Adafruit_MotorShield(); 

Adafruit_DCMotor *ml = AFMS.getMotor(2);
Adafruit_DCMotor *mr = AFMS.getMotor(1);

int scanTime = 30000;

int scanVals[10000];
int numScanVals = 0;

int ultrasonic = 22;

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
int rampThreshold = 250;
int rampTimeThreshold = 500;
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
    ml->run(BACKWARD);
    mr->setSpeed(0);
    delay(TURN_TIME);
    ml->setSpeed(0);
    mr->setSpeed(0);
    while(true) {};
  }
  Serial.println(reading);
  delay(50);
}
