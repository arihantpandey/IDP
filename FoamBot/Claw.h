//functions used to control the claw

#define POS_OPEN 120
#define POS_CLOSED 160
#define POS_HALF 140 //Claw end states
#define CLAWTIME 20 //time it takes to turn 1 deg (millisec)

bool grab(Servo servo) //closes the claw
{
  Serial.println("Closing the claw");

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
  digitalWrite(REDLED, LOW); //turns off the blocktype LEDs after delivery

}

void halfRelease(Servo servo) //opens the claw halfway, used to push the block
{
  Serial.println("Half Opening servo");

  for (int pos = POS_CLOSED; pos >= POS_HALF; pos--) //open the claw
  {
    servo.write(pos);
    //Serial.println("Position: " + String(pos));
    delay(CLAWTIME);
  }
}
