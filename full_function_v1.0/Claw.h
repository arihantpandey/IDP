#define POS_OPEN 130
#define POS_CLOSED 180 //Claw end states
#define TURNTIME 50 //time it takes to turn 1 deg (millisec)
#define THRESHOLD 15000 //(used to differentiate the blocks)
#define REDLED 4 
#define GREENLED 5 //LEDs used to show block type

bool grab(Servo servo) //closes the claw and returns true for fine block, false for coarse block
{
    Serial.println("Closing the claw");



    int integral = 0; // used for current measurement

    for (int pos = POS_OPEN; pos <= POS_CLOSED; pos++) //close the claw
    {
        servo.write(pos);
        integral += analogRead(A0);
        //Serial.println("Position: " + String(pos));
        delay(TURNTIME);
    }

    if (integral > THRESHOLD) //true for fine
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

void drop(Servo servo) //opens the claw
{

    Serial.println("Opening the claw");

    for (int pos = POS_CLOSED; pos >= POS_OPEN; pos--) //open the claw
    {
        servo.write(pos);
        //Serial.println("Position: " + String(pos));
        delay(TURNTIME);
    }

    digitalWrite(GREENLED, LOW);
    digitalWrite(REDLED, LOW); //turns off the blocktype LEDs

}
