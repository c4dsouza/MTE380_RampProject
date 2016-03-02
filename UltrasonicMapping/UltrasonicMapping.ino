// MTE 380 - Group 1

#include <TimerOne.h>
#include <Adafruit_MotorShield.h>

#define SRF05_Pin 7
#define PULSE_LENGTH  10
#define DEG_PER_STEP  3
#define NUM_STEPS 180/DEG_PER_STEP //needs to be adjusted

volatile unsigned int numSamples = 0;

void setup() {
  // initialize timer1
  Timer1.initialize(); 

  // attaches callback() as a timer overflow interrupt with a 50 millisecond period
  Timer1.attachInterrupt(callback, 50000);  

  //SET MOTORS TO ROTATE
}

void loop() {

}

void callback(){
  if (numSamples++ < 50) {
    int distance_cm = readUltrasonic(); 
  } else {
    //STOP ROTATION
    Timer1.detachInterrupt();
  }
}

/*
 * Return SRF05 distance in cm
 */
int readUltrasonic() {
  //Set pin direction
  pinMode(SRF05_Pin, OUTPUT);
  digitalWrite(SRF05_Pin, LOW);
  delayMicroseconds(2);

  //Send chirp
  digitalWrite(SRF05_Pin, HIGH);
  delayMicroseconds(PULSE_LENGTH);
  digitalWrite(SRF05_Pin, LOW);

  //Return distance
  pinMode(SRF05_Pin, INPUT);
  int distance = pulseIn(SRF05_Pin, HIGH)/58;
  return distance;
}

/*
 * 
 */
void findBlock(){

}
 
