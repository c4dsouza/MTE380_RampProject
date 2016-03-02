//SRF05 sample code//

int duration;                                //Stores duration of pulse in
int distance;                                // Stores distance
int sensorpin = 7;                           // Pin for SRF05

void setup()
{
  Serial.begin(9600);
}

void loop()
{
  pinMode(sensorpin, OUTPUT);
  digitalWrite(sensorpin, LOW);              // Make sure pin is low before sending a short high to trigger ranging
  delayMicroseconds(2);
  digitalWrite(sensorpin, HIGH);             // Send a short 10 microsecond high burst on pin to start ranging
  delayMicroseconds(10);
  digitalWrite(sensorpin, LOW);              // Send pin low again before waiting for pulse back in
  pinMode(sensorpin, INPUT);
  duration = pulseIn(sensorpin, HIGH);       // Reads echo pulse in from SRF05 in micro seconds
  distance = duration/58;                    // Dividing this by 58 gives us a distance in cm
  Serial.println(distance);                  // Wait before looping to do it again
  delay(100);   
}
