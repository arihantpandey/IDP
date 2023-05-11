#include "Distinguisher.h"

void setup() {
  // put your setup code here, to run once:
  Serial.begin(9600);
  pinMode(A2, OUTPUT); // Sets the trigPin as an OUTPUT
  pinMode(A3, INPUT);
}

void loop() {
  // put your main code here, to run repeatedly:
  detect(A2,A3);
  delay(500);
}
