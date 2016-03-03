#include <Adafruit_MotorShield.h>
#include <PID_v1.h>

#define leftMotorBase 128
#define leftMotorMax 200
#define leftMotorMin 40
#define rightMotorBase  128
#define rightMotorMax  200
#define rightMotorMin  40
#define MotorMax 150
#define TIME_PER_DEGREE 9.4
#define Kp_cons  0.03
#define Ki_cons  0
#define Kd_cons  0.005
#define numSensors  5
#define numCalibrations 10

//STATES
#define FIND_RAMP_STATE 0
#define RAMP_PID_STATE 1

Adafruit_MotorShield AFMS = Adafruit_MotorShield(); 

Adafruit_DCMotor *ml = AFMS.getMotor(2);
Adafruit_DCMotor *mr = AFMS.getMotor(1);

int current_state = 0;

int scanTime = 30000;

int scanVals[10000];
int numScanVals = 0;

int ultrasonic = 22;

static const uint8_t sensor_pin[] = {A11,A12,A13,A14,A15};
static const uint8_t trigger_pin[] = {51,49,47,45,43};

double Setpoint, Input, Output;
PID myPID(&Input, &Output, &Setpoint, Kp_cons,Ki_cons,Kd_cons, REVERSE);

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

int readIRSensor(uint8_t sensorNumber){
  digitalWrite(trigger_pin[sensorNumber], LOW);
  int ambient = analogRead(sensor_pin[sensorNumber]);
  digitalWrite(trigger_pin[sensorNumber], HIGH);
  delayMicroseconds(20);
  int measured = analogRead(sensor_pin[sensorNumber]);
  digitalWrite(trigger_pin[sensorNumber], LOW);
  delayMicroseconds(100);
  return (abs(measured-ambient));
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

  Setpoint = 1500;
//  for (int i = 0; i < numCalibrations; i++){
//    Setpoint += readIRSum();
//    Serial.println(readIRSum());
//    delay(250);
//  }
//  Setpoint = Setpoint/numCalibrations;
  Serial.print("Calibrated: ");
  Serial.println(Setpoint);
  
//  delay(5000);
}

void driveMotors(double drive){
  int driveOutput = drive;
  Serial.println(driveOutput);
  ml->setSpeed(constrain(leftMotorBase+driveOutput, leftMotorMin, leftMotorMax));
  ml->run(FORWARD);
  
  mr->setSpeed(constrain(rightMotorBase-driveOutput, rightMotorMin, rightMotorMax));
  mr->run(FORWARD);
}

void start_state(int state) {
  Serial.println("STATE");
  Serial.println(state);
  current_state = state;
  switch(current_state) {
    case FIND_RAMP_STATE:
    
    mr->run(FORWARD);
    mr->setSpeed(200);
    ml->run(FORWARD);
    ml->setSpeed(215);
    
    break;
    case RAMP_PID_STATE:
    
    mr->run(FORWARD);
    mr->setSpeed(200);
    ml->run(FORWARD);
    ml->setSpeed(215);
    delay(1000);
    setupIR();
    Input = readIRSum();
    myPID.SetSampleTime(10);
    myPID.SetMode(AUTOMATIC);
    myPID.SetOutputLimits(-100, 100);
    
    break;
  }
}

void setup() {
  Serial.begin(9600);
  AFMS.begin();
  pinMode(ultrasonic, OUTPUT);
  digitalWrite(ultrasonic, LOW);
  start_state(FIND_RAMP_STATE);
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
int rampTimeThreshold = 200;

void loop() {
  int reading = 0;
  switch(current_state) {
    case FIND_RAMP_STATE: //@@@@@@@@@@@@@@@@@@@@@@@@@@@
    
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
      ml->setSpeed(0);
      mr->setSpeed(0);
      Serial.println("CHANGING STATE");
      start_state(RAMP_PID_STATE);
      return;
    }
    Serial.println(reading);
    delay(50);
    
    break;
    case RAMP_PID_STATE: //@@@@@@@@@@@@@@@@@@@@@@@@@@@
    
    Input = readIRSum();
    myPID.Compute();
    driveMotors(Output);
    
    break;
  }
  
}
