#define POS_OPEN 120
#define POS_CLOSED 160 //Claw end states
#define POS_HALF 140

#define CLAWTIME 50 //time it takes to turn 1 deg (millisec)

bool grab(Servo servo) //closes the claw and returns true for fine block, false for coarse block
{
  Serial.println("Closing the claw");



  int integral = 0; // used for current measurement

  for (int pos = POS_OPEN; pos <= POS_CLOSED; pos++) //close the claw
  {

    servo.write(pos);
    //Serial.println("Position: " + String(pos));
    delay(CLAWTIME);
  }


}

void drop(Servo servo) //opens the claw
{

  Serial.println("Opening the claw");

  for (int pos = POS_HALF; pos >= POS_OPEN; pos--) //open the claw
  {
    servo.write(pos);
    //Serial.println("Position: " + String(pos));
    delay(CLAWTIME);
  }

  digitalWrite(GREENLED, LOW);
  digitalWrite(REDLED, LOW); //turns off the blocktype LEDs

}

void halfrelease(Servo servo)
{
  Serial.println("Half Opening servo");

  for (int pos = POS_CLOSED; pos >= POS_HALF; pos--) //open the claw
  {
    servo.write(pos);
    Serial.println("Position: " + String(pos));
    delay(CLAWTIME);
  }


}
