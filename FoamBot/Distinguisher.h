//functions used to differentiate the blocks
//parts copied from the US sensor example code

#define CUTOFF 6.7 //if the measured distance is larger than this, the block is fine

float detect() //activates ultrasonic and returns the measured distance
{
  Serial.println("Detecting block type");

  long duration; // variable for the duration of sound wave travel
  float distance; // variable for the distance measurement

  digitalWrite(USTRIG, LOW);
  delayMicroseconds(2);
  // Sets the trigPin HIGH (ACTIVE) for 10 microseconds
  digitalWrite(USTRIG, HIGH);
  delayMicroseconds(10);
  digitalWrite(USTRIG, LOW);
  // Reads the echoPin, returns the sound wave travel time in microseconds
  duration = pulseIn(USECHO, HIGH);
  // Calculating the distance
  distance = duration * 0.034 / 2; // Speed of sound wave divided by 2 (go and back)
  // Displays the distance on the Serial Monitor
  Serial.print("Distance: ");
  Serial.print(distance);
  Serial.println(" cm");

  return (distance);

}
