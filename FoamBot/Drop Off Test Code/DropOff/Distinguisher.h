#define CUTOFF 10

bool detect() //activates ultrasonic and returns true for fine block, false for coarse block
{
    Serial.println("Detecting block type");

    long duration; // variable for the duration of sound wave travel
    int distance; // variable for the distance measurement
    
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

    if (distance > CUTOFF) //true for fine
    {
        Serial.println("Fine");
        digitalWrite(GREENLED, HIGH);
        return true;
    }
    else
    {
        Serial.println("Coarse");
        digitalWrite(REDLED, HIGH);
        return false;
    }
}
