const int numSensors = 5;
static const uint8_t sensor_pin[] = {A0,A1,A2,A3,A4};
static const uint8_t trigger_pin[] = {8,9,10,11,12};

void setup() {
  Serial.begin(9600);
  for (int i = 0; i < numSensors; i++){
    pinMode(trigger_pin[i], OUTPUT);
    digitalWrite(trigger_pin[i], HIGH);
  }
}

void loop() {
  for (int i = 0; i < numSensors; i++) {
    Serial.print(IR_Read(trigger_pin[i] ,sensor_pin[i]));
    if (i == (numSensors-1)) {
      Serial.println();
    } else {
      Serial.print(',');
    }
    delay(1);
  }
}

int IR_Read(uint8_t trigger, uint8_t sensor){
  int ambient = analogRead(sensor);
  delay(1);
  digitalWrite(trigger, HIGH);
  delay(1);
  int measured = analogRead(sensor);
  delay(1);
  digitalWrite(trigger, LOW);
  return(ambient - measured);
//  return measured;
}

