#include <Servo.h>
#include <Adafruit_MotorShield.h>
#include <SharpIR.h>
#include "Pinout.h"
#include "Claw.h"
#include "Movement.h"
#include "Distinguisher.h"


uint16_t MAXSPEED = 250;
#define SLOWSPEED 120
#define TURNTIME 5000

#define INTERSECTIONIGNORE 300
#define TURNDELAY 700
#define CREEPBACKTIME 2000
#define CREEPFORWARDTIME 1000

//volatile bool on = true; // True if the robot should be running, false otherwise
Servo servo; //the servo that controlls the claw
// Create the motor shield object with the default I2C address
Adafruit_MotorShield AFMS = Adafruit_MotorShield();
Adafruit_DCMotor *leftMotor = AFMS.getMotor(1);
Adafruit_DCMotor *rightMotor = AFMS.getMotor(4);
SharpIR IRSensor( SharpIR::GP2Y0A21YK0F, IRDISTANCE );

//bool state[4]; // The state of the line sensors
int state; //The state of the line sensors expressed as a number
int intersections = 0; //Counts the number of intersections passed
bool blocktype; //the type of the block grabbed, true for fine, false for coarse


void setup()
{
  Serial.begin(9600);
  /*
    leftMotor->setSpeed(0);
    rightMotor->setSpeed(0);
    leftMotor->run(FORWARD);
    rightMotor->run(FORWARD);
  */


  Serial.println("Adafruit Motorshield v2 - DC Motor test!");
  if (!AFMS.begin()) {         // create with the default frequency 1.6KHz
    // if (!AFMS.begin(1000)) {  // OR with a different frequency, say 1KHz
    Serial.println("Could not find Motor Shield. Check wiring.");
    while (1);
  }
  Serial.println("Motor Shield found.");

  Serial.println("Motor Shield found.");

}

void lowop()
{
  Serial.println("LOOp");

  moveForward(false);

  delay(10000);

}


void loop()
{
  //on = false;
  pinSetup();
  Serial.println("Waiting for start button");
  while (digitalRead(BUTTON) == HIGH ) {} //Wait for the button press to start

  //Debouncing to avoid turning off the robot by the bounce of the start button
  /*noInterrupts();
    delay(100);
    interrupts();*/

  Serial.println("Starting robot");

  Serial.println("Opening claw to starting position");
  //drop(servo); //open servo


  Serial.println("Moving to the edge of the box");
  moveForward(false); //Moves to edge of box

  Serial.println("Moving to the drop off area");
  moveForward(false); // Moves to drop off area

  Serial.println("Moving to the first block");
  moveForward(false); // Moves to first block

  Serial.println("Grabbing first block");
  grab(servo); //Grabs first block
  blocktype = detect(); //detects blocktypes

  Serial.println("Turning around");
  turnAround(); //turns back toward start area

  Serial.println("Half releasing servo");
  halfrelease(servo); //This is needed to avoid getting stuck on the ramp

  Serial.println("Moving block to the drop off area");
  moveForward(false); // Moves to drop off area
  moveForward(false);

  Serial.println("Dropping the first block");
  drop(servo);
  creepBackward();
  turnAround();

  //First block delivered

  Serial.println("Moving to other side of ramp");
  moveForward(false); //move to first block's location
  Serial.println("Searching for second block");
  moveForward(true); //move forward looking for the second block


  Serial.println("Grabbing second block");
  creepForward();
  grab(servo); //grabs second block
  blocktype = detect();

  Serial.println("Turning around");
  turnAround();

  Serial.print("Moving to first blocks location");
  moveForward(false); //moves to first block's location

  Serial.print("Moving to dropoff area");
  moveForward(false); // moves to dropoff area

  Serial.println("Dropping the second block");
  drop(servo);



  //Second block delivered
  Serial.println("Second block delivered, turning robot off");


}




void pinSetup() //Sets up the pins as input or output, configures the servo.
{
  pinMode(IRDISTANCE, INPUT);
  pinMode(SERVOCURRENT, INPUT);
  pinMode(USTRIG, OUTPUT);
  pinMode(USECHO, INPUT);
  pinMode(REAR, INPUT);
  pinMode(LEFT, INPUT);
  pinMode(MIDDLE, INPUT);
  pinMode(RIGHT, INPUT);
  pinMode(BUTTON, INPUT_PULLUP);
  pinMode(AMBERLED, OUTPUT);
  pinMode(GREENLED, OUTPUT);
  pinMode(REDLED, OUTPUT);
  pinMode(BLUELED1, OUTPUT);
  pinMode(BLUELED2, OUTPUT);
  pinMode(BLUELED3, OUTPUT);
  pinMode(BLUELED4, OUTPUT);

  servo.attach(SERVOCONTROL);
  servo.write(POS_OPEN);

  digitalWrite(GREENLED, LOW);
  
  digitalWrite(REDLED, LOW);

  //attachInterrupt(BUTTON, onButtonPress, FALLING);

  //leftMotor->run(RELEASE);
  //rightMotor->run(RELEASE);

}


void readLineSensors() //reads the inputs of the line sensors
{
  state = digitalRead(REAR) * 8 + digitalRead(LEFT) * 4 + digitalRead(MIDDLE) * 2 + digitalRead(RIGHT) * 1;
  //Serial.println("Line sensor state: " + String(state));
}


String moveForward(bool useIR) //controls the movement of the robot, detects initersections (always) and blocks (if useIR is true).
{
  long int starttime = millis();


  while ((  state != 0b1111 && state != 0b0111 && state != 0b1101 && state != 0b0101) || millis() - starttime < INTERSECTIONIGNORE) //Ignores intersections for a bit to avoid detecting it twice, exits when its at an intersection
  {

    readLineSensors();
    blinker();


    if (useIR && IRSensor.getDistance() == 9) //Stop moving if block is found
    {
      leftMotor->run(RELEASE);
      rightMotor->run(RELEASE);
      digitalWrite(AMBERLED, LOW);
      Serial.println("Block found");
      return "Block";
    }
    //Serial.println("Moving forward");

    leftMotor->setSpeed(MAXSPEED);
    rightMotor->setSpeed(MAXSPEED-5);
    leftMotor->run(FORWARD);
    rightMotor->run(FORWARD);

    delay(20);
  }
  leftMotor->run(RELEASE);
  rightMotor->run(RELEASE);
  digitalWrite(AMBERLED, LOW);
  Serial.println("Intersection found");
  return "Intersection";

  delay(20);


}



void turnAround()
{

  /*while ( state >= 8) //Move the rear sensor off the line
    {
    readLineSensors();
    blinker(); //blinks the amber LED
    leftMotor->run(BACKWARD);
    rightMotor->run(FORWARD);
    leftMotor->setSpeed(MAXSPEED / 2);
    rightMotor->setSpeed(MAXSPEED / 2);
    delay(50);
    }*/

  long int starttime = millis();

  while (state < 8 || millis() - starttime < TURNDELAY) //Turn until rear sensor gets on the line again, also avoid detecting the line again
  {
    readLineSensors();
    blinker(); //blinks the amber LED
    leftMotor->run(BACKWARD);
    rightMotor->run(FORWARD);
    leftMotor->setSpeed(MAXSPEED / 2);
    rightMotor->setSpeed(MAXSPEED / 2);
    delay(50);
  }
  
  //stop turning when the rear sensor is above the line
  //a delay here to get the robot pointing straight 
  delay(30);
  digitalWrite(AMBERLED, LOW);
  leftMotor->run(RELEASE);
  rightMotor->run(RELEASE);
}

void turnRight()
{
  Serial.println("Turning right");
  long int starttime = millis();

  while (millis() < starttime + TURNTIME)
  {
    blinker();
    leftMotor->run(FORWARD);
    rightMotor->run(BACKWARD);
    leftMotor->setSpeed(MAXSPEED / 2);
    rightMotor->setSpeed(MAXSPEED / 2);
    delay(70);
  }

  leftMotor->run(RELEASE);
  rightMotor->run(RELEASE);
  digitalWrite(AMBERLED, LOW);

}

void turnLeft()
{
  Serial.println("Turning left");
  long int starttime = millis();

  while (millis() - starttime <  TURNTIME)
  {
    blinker();
    leftMotor->run(BACKWARD);
    rightMotor->run(FORWARD);
    leftMotor->setSpeed(MAXSPEED / 2);
    rightMotor->setSpeed(MAXSPEED / 2);
    delay(50);
  }

  leftMotor->run(RELEASE);
  rightMotor->run(RELEASE);
  digitalWrite(AMBERLED, LOW);
}

void creepBackward()
{
  long int starttime = millis();

  while(millis()-starttime < CREEPBACKTIME)
  {
    blinker();
    leftMotor->run(BACKWARD);
    rightMotor->run(BACKWARD);
    leftMotor->setSpeed(MAXSPEED);
    rightMotor->setSpeed(MAXSPEED);
    delay(50);
    
  }

  leftMotor->run(RELEASE);
  rightMotor->run(RELEASE);
  digitalWrite(AMBERLED, LOW);
  
}

void creepForward()
{
  long int starttime = millis();

  while(millis()-starttime < CREEPFORWARDTIME)
  {
    blinker();
    leftMotor->run(FORWARD);
    rightMotor->run(FORWARD);
    leftMotor->setSpeed(MAXSPEED);
    rightMotor->setSpeed(MAXSPEED);
    delay(50);
    
  }

  leftMotor->run(RELEASE);
  rightMotor->run(RELEASE);
  digitalWrite(AMBERLED, LOW);
  
}





void blinker() // if this function is called often enough the amber LED will blink at 2Hz
{
  digitalWrite(AMBERLED, millis() % 500 < 250);
}
