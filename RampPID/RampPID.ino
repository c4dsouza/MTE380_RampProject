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

#define Kp_cons  0.03
#define Ki_cons  0
#define Kd_cons  0.01

static const uint8_t sensor_pin[] = {A11,A12,A13,A14,A15};
static const uint8_t trigger_pin[] = {51,49,47,45,43};

double Setpoint, Input, Output;
PID myPID(&Input, &Output, &Setpoint, Kp_cons,Ki_cons,Kd_cons, REVERSE);
Adafruit_MotorShield myDriveTrain = Adafruit_MotorShield();

Adafruit_DCMotor * leftMotor = myDriveTrain.getMotor(2);
Adafruit_DCMotor * rightMotor = myDriveTrain.getMotor(1);

void setup() {
  Serial.begin(9600);
  
  setupIR();
  Input = readIRSum();
  myPID.SetSampleTime(10);
  myPID.SetMode(AUTOMATIC);
  myPID.SetOutputLimits(-100, 100);

  myDriveTrain.begin();
}

void loop() {
  Input = readIRSum();
  myPID.Compute();
  driveMotors(Output);
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

  Setpoint = 2000;
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
  leftMotor->setSpeed(constrain(leftMotorBase+driveOutput, leftMotorMin, leftMotorMax));
  leftMotor->run(FORWARD);
  
  rightMotor->setSpeed(constrain(rightMotorBase-driveOutput, rightMotorMin, rightMotorMax));
  rightMotor->run(FORWARD);
}

