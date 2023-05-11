#include "Claw.h"
#include <Servo.h>
void setup() {
  // put your setup code here, to run once:
  Servo myservo;
  myservo.attach(9);
}

void loop() {
  // put your main code here, to run repeatedly:
  grab(myservo);
  delay(10000);
  drop(myservo)
}
