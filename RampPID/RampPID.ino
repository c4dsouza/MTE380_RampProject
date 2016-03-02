#include <PID_v1.h>
#include <Adafruit_MotorShield.h>

#define numSensors  5
#define numCalibrations 10

#define leftMotorBase 128
#define leftMotorMax 200
#define leftMotorMin 40
#define rightMotorBase  128
#define rightMotorMax  200
#define rightMotorMin  40

#define THRESHOLD 0.2
#define Kp_agg  2
#define Ki_agg  0.5
#define Kd_agg  1
#define Kp_cons  1
#define Ki_cons  0.1
#define Kd_cons  0.25

static const uint8_t sensor_pin[] = {A0,A1,A2,A3,A4};
static const uint8_t trigger_pin[] = {8,9,10,11,12};

double Setpoint, Input, Output;
PID myPID(&Input, &Output, &Setpoint, Kp_cons,Ki_cons,Kd_cons, DIRECT);
Adafruit_MotorShield myDriveTrain = Adafruit_MotorShield();

Adafruit_DCMotor * leftMotor = myDriveTrain.getMotor(2);
Adafruit_DCMotor * rightMotor = myDriveTrain.getMotor(1);

void setup() {
  Serial.begin(9600);
  
  setupIR();
  Input = readIRSum();
  myPID.SetSampleTime(10);
  myPID.SetMode(AUTOMATIC);

  myDriveTrain.begin();
}

void loop() {
  Input = readIRSum();
  myPID.Compute();
  driveMotors(Output);

  if (abs(Input/Setpoint) < THRESHOLD){
    myPID.SetTunings(Kp_cons, Ki_cons, Kd_cons);
  } else {
    myPID.SetTunings(Kp_agg, Ki_agg, Kd_agg);
  }
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

  Setpoint = 0;
  for (int i = 0; i < numCalibrations; i++){
    Setpoint += readIRSum();
  }
  Setpoint = Setpoint/numCalibrations;
  delay(5000);
}

void driveMotors(double driveOutput){
  leftMotor->setSpeed(constrain(leftMotorBase+driveOutput, leftMotorMax, leftMotorMin));
  leftMotor->run(FORWARD);
  
  rightMotor->setSpeed(constrain(rightMotorBase-driveOutput, rightMotorMax, rightMotorMin));
  rightMotor->run(FORWARD);
}

