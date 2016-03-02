const int numSensors = 5;
static const uint8_t sensor_pin[] = {A11,A12,A13,A14,A15};
static const uint8_t trigger_pin[] = {51,49,47,45,43};

void setup() {
  Serial.begin(9600);
  for (int i = 0; i < numSensors; i++){
    pinMode(trigger_pin[i], OUTPUT);
    digitalWrite(trigger_pin[i], HIGH);
  }
}

void loop() {
  int sum = 0;
  for (int i = 0; i < numSensors; i++) {
    sum += IR_Read(i);
//    Serial.print(IR_Read(i));
//    if (i == (numSensors-1)) {
//      Serial.println();
//    } else {
//      Serial.print(',');
//    }
    delay(1);
  }
  Serial.println(sum);
}

int IR_Read(uint8_t sensorNumber){
  
//  digitalWrite(trigger_pin[sensorNumber], LOW);
//  int ambient = analogRead(sensor_pin[sensorNumber]);
//  digitalWrite(trigger_pin[sensorNumber], HIGH);
//  delayMicroseconds(40);
//  int measured = analogRead(sensor_pin[sensorNumber]);
//  digitalWrite(trigger_pin[sensorNumber], LOW);
//  delayMicroseconds(150);
//  return (abs(measured-ambient));
  int measured = analogRead(sensor_pin[sensorNumber]);
  return measured;
}

