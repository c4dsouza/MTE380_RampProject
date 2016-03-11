// I2Cdev and MPU6050 must be installed as libraries, or else the .cpp/.h files
// for both classes must be in the include path of your project
#include "I2Cdev.h"
#include "MPU6050.h"


#if I2CDEV_IMPLEMENTATION == I2CDEV_ARDUINO_WIRE
    #include "Wire.h"
#endif

MPU6050 accelgyro;
//MPU6050 accelgyro(0x69); // <-- use for AD0 high
#define GYRO_OFFSET 89

int16_t ax, ay, az;
int16_t gx, gy, gz;

#define OUTPUT_READABLE_ACCELGYRO

// uncomment "OUTPUT_BINARY_ACCELGYRO" to send all 6 axes of data as 16-bit
// binary, one right after the other. This is very fast (as fast as possible
// without compression or data loss), and easy to parse, but impossible to read
// for a human.
// #define OUTPUT_BINARY_ACCELGYRO

#define LED_PIN 13
bool blinkState = false;

void setup() {
    #if I2CDEV_IMPLEMENTATION == I2CDEV_ARDUINO_WIRE
        Wire.begin();
    #elif I2CDEV_IMPLEMENTATION == I2CDEV_BUILTIN_FASTWIRE
        Fastwire::setup(400, true);
    #endif

    Serial.begin(38400);

    Serial.println("Initializing I2C devices...");
    accelgyro.initialize();

    Serial.println("Testing device connections...");
    Serial.println(accelgyro.testConnection() ? "MPU6050 connection successful" : "MPU6050 connection failed");

    Serial.println("Updating internal sensor offsets...");
//    Serial.print(accelgyro.getXAccelOffset()); Serial.print("\t"); // -76
//    Serial.print(accelgyro.getYAccelOffset()); Serial.print("\t"); // -2359
//    Serial.print(accelgyro.getZAccelOffset()); Serial.print("\t"); // 1688
//    Serial.print(accelgyro.getXGyroOffset()); Serial.print("\t"); // 0
    Serial.print(accelgyro.getYGyroOffset()); Serial.print("\t"); // 0
//    Serial.print(accelgyro.getZGyroOffset()); Serial.print("\t"); // 0
    Serial.print("\n");
//    accelgyro.setXGyroOffset(220);
//    accelgyro.setYGyroOffset(6);
//    accelgyro.setZGyroOffset(-85);
//    Serial.print(accelgyro.getXAccelOffset()); Serial.print("\t"); // -76
//    Serial.print(accelgyro.getYAccelOffset()); Serial.print("\t"); // -2359
//    Serial.print(accelgyro.getZAccelOffset()); Serial.print("\t"); // 1688
//    Serial.print(accelgyro.getXGyroOffset()); Serial.print("\t"); // 0
//    Serial.print(accelgyro.getYGyroOffset()); Serial.print("\t"); // 0
//    Serial.print(accelgyro.getZGyroOffset()); Serial.print("\t"); // 0
//    Serial.print("\n");

    pinMode(LED_PIN, OUTPUT);

}

void loop() {
//    accelgyro.getMotion6(&ax, &ay, &az, &gx, &gy, &gz);
//    Serial.print(ax);Serial.print(',');
//    Serial.print(ay);Serial.print(',');
//    Serial.print(az);Serial.print(',');
//    Serial.print(gx);Serial.print(',');
//    Serial.print(gy);Serial.print(',');
//    Serial.print(gz);Serial.println();

    getTurnAngle();

//    Serial.println(gy);
    blinkState = !blinkState;
    digitalWrite(LED_PIN, blinkState);
    
    delay(5);
}

int getTurnAngle(){
  unsigned long last_time_sampled = 0;
  unsigned long last_time_printed = 0;

  // The integral is in units of DN*us, 1 DN being the ADC step.
  float vel, acc, pos = 0;
  unsigned long now = micros();
  last_time_sampled = now;
  
  while (now - last_time_printed >= 1000000) {
    now = micros();
    acc = accelgyro.getRotationZ() + GYRO_OFFSET;

    pos += acc * (now - last_time_sampled) * (now - last_time_sampled) / (400000000000);

    Serial.println(pos);
    
    last_time_sampled = now;
    delay(2);
  }
  last_time_printed = now;
  
}

