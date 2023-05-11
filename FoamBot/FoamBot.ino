#include <Servo.h>
#include <Adafruit_MotorShield.h>
//#include "followLine.h" //not used
//#include <Wire.h> //not used
//#include <AccelStepper.h> //not used
#include <SharpIR.h>
#include "Pinout.h"
#include "Claw.h"
//#include "Movement.h" // not used
#include "Distinguisher.h"

uint16_t MAXSPEEDR = 246;
uint16_t MAXSPEEDL = 250; // normal speed of the motors, different because motor differences
uint16_t SLOWSPEED = 140; // reduced speed, used when corrections are made
#define TURNDELAY 3000 // used in turnAround(), for this amount of time is the line checking turned off
#define TURNTIME 4000 // used in turnRightTime(), turnLeftTime(), the time it takes to make a 90 deg turns
#define INTERSECTIONIGNORE 300 //used in moveForward(), intersections are ignored for this much time after start to avoid detecting the same intersection twice
#define BACKWARDINTERSECTIONIGNORE 1800 // used in moveBackward() when called from Foam_Delivery(), intersections are ignored for this much time after start
#define RAMPTIME 6500 //used in cross_ramp(), the time it takes to climb the ramp from the previous intersection
#define GRABCREEP 500 //after detecting a block, the robot moves forward for this amount of time to grab it
#define DROPOFFTIME 1700 // used in Drop_Off(), for this amount of time moves the robot forward before dropping the block
#define STRAIGHTENTIME 220 // used in turnAround(), turnRightAlign(), turnLeftAlign() the time it takes for the rear sensor to move to the centre of the line from the edge of the line while turning
#define SEARCHTIME 2800 // used in blockSearch(), the robot looks in one direction for this amount of time
#define THIRDBLOCKTIME 2200 // used in thirdBlock(), after finding the third block, the robot moves forward for this amount of time before grabbing it
//#define CREEPFORWARDTIME 2500 // not used
//#define CREEPBACKWARDTIME 2000 //not used
//#define CREEPTIME 500 //not used

bool block_location; //the location of the third block, true if right half, false if in left half from perspective of start square

Servo servo; //the servo that controlls the claw

// Create the motor shield object with the default I2C address and initialize the motors
Adafruit_MotorShield AFMS = Adafruit_MotorShield();
Adafruit_DCMotor *leftMotor = AFMS.getMotor(1);
Adafruit_DCMotor *rightMotor = AFMS.getMotor(4);

// Initialize the IR distance sensor
SharpIR IRSensor( SharpIR::GP2Y0A21YK0F, IRDISTANCE );

int state; //The state of the line sensors expressed as a four bit number (MSB: rear, then left, middle and LSB: right)
bool blocktype; //the type of the block grabbed, true for fine, false for coarse


void setup()
{
  Serial.begin(9600);

  Serial.println("Adafruit Motorshield v2");
  if (!AFMS.begin()) {         // create with the default frequency 1.6KHz
    Serial.println("Could not find Motor Shield. Check wiring.");
    while (1);
  }
  Serial.println("Motor Shield found.");

}



void looxp() //used for testing
{

  pinSetup();
  Serial.println("Waiting for start button");

  while (digitalRead(BUTTON) == HIGH ) {} //Wait for the button press to start


  digitalWrite(GREENLED, LOW);
  digitalWrite(REDLED, LOW);

  grab(servo);
  halfRelease(servo);
  drop(servo);

  //delay(100);
  Serial.println(tripleCheck());
  //delay(200);


}


void loop() //main loop
{

  pinSetup(); //initialize the pins of the arduino
  Serial.println("Waiting for start button");
  while (digitalRead(BUTTON) == HIGH ) {} //Wait for the button press to start

  Serial.println("Starting robot");

  Serial.println("Moving to the edge of the box");
  moveForward(false); //Moves to edge of box

  Serial.println("Moving to the drop off area");
  moveForward(false); // Moves to drop off area

  Serial.println("Moving to the first block");
  Serial.println("Climbing ramp");
  cross_ramp(); // move forward with reduced navigation as the line sensors dont work reliably on the ramp

  Serial.println("Slowing down");  //Slows down to make downhill navigation more reliable
  MAXSPEEDR -= 100;
  MAXSPEEDL -= 100;
  moveForward(false); // Moves to first block
  MAXSPEEDR += 100;
  MAXSPEEDL += 100;

  Serial.println("Grabbing first block");

  grab(servo); //Grabs first block

  blocktype = tripleCheck(); //detects blocktypes

  Serial.println("Creeping back");
  creepBackward(900); //Moves backward to avoid hitting second block during the 180deg turn

  Serial.println("Turning around");
  turnAround(); //turns back towards start area

  halfRelease(servo); // releases the claw halfway to avoid problems with the ramp

  Serial.println("Moving block to the drop off area");
  Serial.println("Climbing ramp");
  cross_ramp();  // move forward with reduced navigation as the line sensors dont work reliably on the ramp
  Serial.println("Slowing down");
  MAXSPEEDR -= 100;
  MAXSPEEDL -= 100; //slow down robot across and down bridge
  moveForward(false); // Moves to drop off area
  MAXSPEEDR += 100;
  MAXSPEEDL += 100; //restore previous speed
  Serial.println("Dropping the first block");

  Foam_Delivery(blocktype); //drops block into correct box

  Serial.println("First block delivered");

  //First block delivered


  Serial.println("Climbing ramp");
  cross_ramp(); // move forward with reduced navigation as the line sensors dont work reliably on the ramp
  Serial.println("Slowing down");
  MAXSPEEDR -= 100;
  MAXSPEEDL -= 100;
  moveForward(false); // Moves to first block's location
  MAXSPEEDR += 100;
  MAXSPEEDL += 100;

  Serial.println("Searching for second block");
  moveForward(false); //move forward to edge of 3rd block's box

  Serial.println("Grabbing second block");

  grab(servo); //grabs second block
  blocktype = tripleCheck(); //detects blocktypes

  align(); //align the rear of the robot to the line to make turning around more reliable

  Serial.println("Moving backward");
  creepBackward(3000); // moves backward to avoid detecting the box with the rear sensor while turning around

  Serial.println("Turning around");
  turnAround(); //turns around
  halfRelease(servo); // releases the claw halfway to avoid problems with the ramp

  Serial.println("Moving to first block's location");
  moveForward(false); //moves to first block's location
  Serial.println("Moving to drop off area");
  moveForward(false); // moves to dropoff area

  Serial.println("Dropping the second block");
  Foam_Delivery(blocktype); //drops block into correct box

  //Second block delivered
  Serial.println("Second block delivered");

  cross_ramp(); // move forward with reduced navigation as the line sensors dont work reliably on the ramp
  moveForward(false); // moces to the first block's location
  moveForward(false); // moves to edge of third block's square

  thirdBlock(); //scans the square to look for third block, grabs it then returns to starting position
  blocktype = tripleCheck(); //detects blocktypes

  Serial.println("Moving backward");
  creepBackward(3000); // moves backward to avoid detecting the box with the rear sensor while turning around

  align(); //aligns rear sensor to line to make turning around more reliable

  Serial.println("Turning around");
  turnAround(); //turns around
  halfRelease(servo); // releases the claw halfway to avoid problems with the ramp

  Serial.println("Moving to first block's location");
  moveForward(false); //moves to first block's location
  Serial.println("Moving to drop off area");
  moveForward(false); // moves to dropoff area

  Serial.println("Dropping the third block");
  Foam_Delivery(blocktype); // drops block into correct box

  //Third block delivered
  Serial.println("Third block delivered");

  back_to_start(); //from drop off location, move back to starting square

  //All three blocks delivered and robot returned to starting position

}




void pinSetup() //Sets up the pins as input or output, configures the servo. stops the motors
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

  leftMotor->run(RELEASE);
  rightMotor->run(RELEASE);

}


void readLineSensors() //reads the inputs of the line sensors and stores them in an integer, each bit of the integer corresponds to one sensor
{
  state = digitalRead(REAR) * 8 + digitalRead(LEFT) * 4 + digitalRead(MIDDLE) * 2 + digitalRead(RIGHT) * 1;
  //Serial.println("Line sensor state: " + String(state));

  //Show the state of the line sensors on the feedback board
  digitalWrite(BLUELED1, digitalRead(REAR));
  digitalWrite(BLUELED2, digitalRead(LEFT));
  digitalWrite(BLUELED3, digitalRead(MIDDLE));
  digitalWrite(BLUELED4, digitalRead(RIGHT));
}


String moveForward(bool useIR) //controls the movement of the robot, detects initersections (always) and blocks (if useIR is true). The case  (useIR = true) was not used
{
  long int starttime = millis();

  //At an intersection the right and the left sensors are on (other sensors can take any value)
  while (( state & 0b0101) != 0b0101 || millis() - starttime < INTERSECTIONIGNORE) //Ignores intersections for a bit to avoid detecting it twice, exits when it's at an intersection
  {
    readLineSensors(); //update state
    blinker(); //blink amber LED

    if (useIR && IRSensor.getDistance() == 9) //Stop moving if block is found (was not used)
    {
      leftMotor->run(RELEASE);
      rightMotor->run(RELEASE);
      digitalWrite(AMBERLED, LOW);
      Serial.println("Block found");
      return "Block";
    }
    else if (state == 0b0001 || state == 0b1001) { //just right or right and rear
      adjust_right();
    }

    else if (state == 0b0100 || state == 0b1100) { //just left or left and rear
      adjust_left();
    }

    else if (state == 0b0110 || state == 0b1110) { //left and middle or left, middle and rear
      slight_left();
    }

    else if (state == 0b0011 || state == 0b1011) { //right and middle or right, middle and rear
      slight_right();
    }
    else // move forward and hope it's on the line for every other state
    {
      Serial.println("No adjustment");
      leftMotor->setSpeed(MAXSPEEDL);
      rightMotor->setSpeed(MAXSPEEDR);
      leftMotor->run(FORWARD);
      rightMotor->run(FORWARD);
    }

    delay(5);
  }

  //Stop at the intersection and turn of the blinking LED
  leftMotor->run(RELEASE);
  rightMotor->run(RELEASE);
  digitalWrite(AMBERLED, LOW);
  Serial.println("Intersection found");
  return "Intersection";

  delay(20);
}


void turnAround() // does a turn until the rear sensor gets back on the line again
{

  long int starttime = millis();
  //while(rear sensor low || still in the ignore time)
  while ( (state & 0b1000) != 0b1000 || millis() - starttime < TURNDELAY) //Turn until rear sensor gets on the line again, avoid checking for a bit to make sure it clears the original line
  {
    readLineSensors();
    blinker(); //blinks the amber LED
    leftMotor->run(BACKWARD);
    rightMotor->run(FORWARD);
    leftMotor->setSpeed(MAXSPEEDL / 2);
    rightMotor->setSpeed(MAXSPEEDR / 2);
    delay(50);
  }

  //a delay here to get the robot pointing straight
  delay(STRAIGHTENTIME);
  //stop turning and turn the blinking LED off
  digitalWrite(AMBERLED, LOW);
  leftMotor->run(RELEASE);
  rightMotor->run(RELEASE);
}


void turnRightTime() //90deg rotation, time based
{
  Serial.println("Turning right, time based");
  long int starttime = millis();
  while (millis() - starttime < TURNTIME)
  {
    blinker();
    leftMotor->run(FORWARD);
    rightMotor->run(BACKWARD);
    leftMotor->setSpeed(MAXSPEEDL / 2);
    rightMotor->setSpeed(MAXSPEEDR / 2);
    delay(50);
  }

  //digitalWrite(BLUELED1, HIGH);
  delay(30);
  //digitalWrite(BLUELED1, LOW);
  //Stop after specified time and turn LED off
  leftMotor->run(RELEASE);
  rightMotor->run(RELEASE);
  digitalWrite(AMBERLED, LOW);
}


void turnRightJunction() // turns right until the rear line sensor detects the second line
{
  Serial.println("Turning right, junction based");
  long int starttime = millis();

  int rear_detection_count = 0; //number of times the rear sensor detects a line
  bool rear_detection_check = false; //true if the rear sensor is above the line
  while (rear_detection_count < 2) //stop when the rear sensor detects second line
  {
    readLineSensors();
    rear_detection_check = (state >= 8);
    if (rear_detection_check == true) {
      rear_detection_count += 1;
      //digitalWrite(BLUELED1, HIGH);
      rear_detection_check = false;
      delay(800); //to avoid detecting the same line twice
    }
    //digitalWrite(BLUELED1, LOW);

    blinker();
    leftMotor->run(FORWARD);
    rightMotor->run(BACKWARD);
    leftMotor->setSpeed(MAXSPEEDL / 2);
    rightMotor->setSpeed(MAXSPEEDR / 2);
    delay(50);
  }

  //Stop and turn off LED
  leftMotor->run(RELEASE);
  rightMotor->run(RELEASE);
  digitalWrite(AMBERLED, LOW);
}

void turnRightAlign() { //turns right until the rear line sensor gets on the line
  Serial.println("Turning right, aligning to line");
  while (state < 8) //Turn until rear sensor gets on the line again
  {
    readLineSensors();
    blinker(); //blinks the amber LED
    leftMotor->run(FORWARD);
    rightMotor->run(BACKWARD);
    leftMotor->setSpeed(MAXSPEEDL / 2);
    rightMotor->setSpeed(MAXSPEEDR / 2);
    delay(20);
  }

  //maybe a delay here to get the robot pointing straight
  delay(STRAIGHTENTIME);
  //stop turning when the rear sensor is above the line
  digitalWrite(AMBERLED, LOW);
  leftMotor->run(RELEASE);
  rightMotor->run(RELEASE);
}

void turnLeftTime() //90deg rotation, time based
{
  Serial.println("Turning left, time based");
  long int starttime = millis();

  while (millis() - starttime < TURNTIME)
  {
    blinker();
    leftMotor->run(BACKWARD);
    rightMotor->run(FORWARD);
    leftMotor->setSpeed(MAXSPEEDL / 2);
    rightMotor->setSpeed(MAXSPEEDR / 2);
    delay(50);
  }
  //digitalWrite(BLUELED1, HIGH);
  delay(30);
  //Stop after specified time and turn off LED
  //digitalWrite(BLUELED1, LOW);
  leftMotor->run(RELEASE);
  rightMotor->run(RELEASE);
  digitalWrite(AMBERLED, LOW);
}


void turnLeftJunction() // turns left until the rear line sensor detects the second line
{
  Serial.println("Turning left, junction based");
  long int starttime = millis();

  int rear_detection_count = 0; //number of times the rear sensor detects a line
  bool rear_detection_check = false; //true if the rear sensor is above the line
  while (rear_detection_count < 2) //stop when the rear sensor detects second line
  {
    readLineSensors();
    rear_detection_check = (state >= 8);
    if (rear_detection_check == true) {
      rear_detection_count += 1;
      //digitalWrite(BLUELED1, HIGH);
      rear_detection_check = false;
      delay(800); // to avoid detecting the same line twice
    }
    //digitalWrite(BLUELED1, LOW);

    blinker();
    leftMotor->run(BACKWARD);
    rightMotor->run(FORWARD);
    leftMotor->setSpeed(MAXSPEEDL / 2);
    rightMotor->setSpeed(MAXSPEEDR / 2);
    delay(50);
  }

  //Stop and turn off LED
  leftMotor->run(RELEASE);
  rightMotor->run(RELEASE);
  digitalWrite(AMBERLED, LOW);
}

void turnLeftAlign() { //turns left until the rear line sensor gets on the line
  Serial.println("Turning left, aligning to line");
  while (state < 8) //Turn until rear sensor gets on the line again
  {
    readLineSensors();
    blinker(); //blinks the amber LED
    leftMotor->run(BACKWARD);
    rightMotor->run(FORWARD);
    leftMotor->setSpeed(MAXSPEEDL / 2);
    rightMotor->setSpeed(MAXSPEEDR / 2);
    delay(20);
  }

  //a delay here to get the robot pointing straight
  delay(STRAIGHTENTIME);
  //stop turning when the rear sensor is above the line
  digitalWrite(AMBERLED, LOW);
  leftMotor->run(RELEASE);
  rightMotor->run(RELEASE);
}

void creepForward(long int creepTime) // move forward for the time specified as creepTime
{
  long int starttime = millis();
  while (millis() - starttime < creepTime)
  {
    readLineSensors();
    blinker();
    leftMotor->run(FORWARD);
    rightMotor->run(FORWARD);
    leftMotor->setSpeed(MAXSPEEDL);
    rightMotor->setSpeed(MAXSPEEDR);
    delay(20);

  }
  //Stop after specified time and turn off LED
  leftMotor->run(RELEASE);
  rightMotor->run(RELEASE);
  digitalWrite(AMBERLED, LOW);

}


void moveBackward(int ignoreTime) //move backwards until junction detected, ignores junctions until ignoreTime has passed
{
  Serial.println("Moving backward until intersection is detected");
  long int starttime = millis();

  while ( ( state & 0b0101) != 0b0101 ||  millis() - starttime < ignoreTime) //move backward until junction is detected (right and left on the line) and ignoreTime has passed
  {
    blinker();
    readLineSensors();
    leftMotor->run(BACKWARD);
    rightMotor->run(BACKWARD);
    leftMotor->setSpeed(MAXSPEEDL);
    rightMotor->setSpeed(MAXSPEEDR);
    delay(20);
  }
  //stop moving when intersection is detected
  leftMotor->run(RELEASE);
  rightMotor->run(RELEASE);
  digitalWrite(AMBERLED, LOW);


}

void moveBackwardMiddle() { //move backwards until the middle sensor is on the white line to prepare for line following
  Serial.println("Moving back until middle sensor is on the line");
  while ((state & 0b0010) != 0b0010) {
    readLineSensors();
    blinker();
    leftMotor->run(BACKWARD);
    rightMotor->run(BACKWARD);
    leftMotor->setSpeed(MAXSPEEDL);
    rightMotor->setSpeed(MAXSPEEDR);
    delay(20);
  }
  //Stop moving and turn off LED
  leftMotor->run(RELEASE);
  rightMotor->run(RELEASE);
  digitalWrite(AMBERLED, LOW);
}

void creepBackward(int delay_time) { //move backward for a specified time

  Serial.println("Creeping back for " + String(delay_time) + " ms");
  long int starttime = millis();

  while (millis() - starttime < delay_time) // move backwards for delay_time
  {
    blinker();
    leftMotor->run(BACKWARD);
    rightMotor->run(BACKWARD);
    leftMotor->setSpeed(MAXSPEEDL);
    rightMotor->setSpeed(MAXSPEEDR);
    delay(20);
  }

  //stop after the specified time
  leftMotor->run(RELEASE);
  rightMotor->run(RELEASE);
  digitalWrite(AMBERLED, LOW);

}

void blinker() // if this function is called often enough the amber LED will blink at 2Hz
{
  digitalWrite(AMBERLED, millis() % 500 < 250);
}


//function used to deliver the block to the blue or red box from the junction between the boxes
//the previous function should be 'moveforward(false)'
void Foam_Delivery(bool blocktype) { //deliver the block to the blue or red box from the junction between the boxes then return to the junction

  if (blocktype == true) { //fine to blue area
    turnLeftTime(); //turn 90 deg to face the box
    //digitalWrite(BLUELED4, LOW);
    creepForward(DROPOFFTIME); // move into the box
    //digitalWrite(BLUELED4, HIGH);
    drop(servo); //drop block
    moveBackward(BACKWARDINTERSECTIONIGNORE); //move back to intersection
    //digitalWrite(BLUELED4, LOW);
    turnLeftJunction(); //turn back towards next block
    moveBackwardMiddle(); //put front of robot on white line
    turnRightAlign(); //align rear to line

  }
  else if (blocktype == false) { //coarse to red area
    turnRightTime(); //turn 90 deg to face the box
    //digitalWrite(BLUELED4, LOW);
    creepForward(DROPOFFTIME); // move into the box
    //digitalWrite(BLUELED4, HIGH);
    drop(servo);
    moveBackward(BACKWARDINTERSECTIONIGNORE); //move back to intersection
    //digitalWrite(BLUELED4, LOW);
    turnRightJunction(); //turn back towards next block
    moveBackwardMiddle(); //put front of robot on white line
    turnLeftAlign(); //align rear to line
  }
}



void adjust_right() { //large adjustment to the right
  Serial.println("Adjusting right");
  //Serial.println(state);
  leftMotor->setSpeed(250);
  rightMotor->setSpeed(SLOWSPEED);
  leftMotor->run(FORWARD);
  rightMotor->run(FORWARD);
  delay(30);

}


void adjust_left() { //large adjustment to the left
  Serial.println("Adjusting left");
  //Serial.println(state);
  leftMotor->setSpeed(SLOWSPEED);
  rightMotor->setSpeed(250);
  leftMotor->run(FORWARD);
  rightMotor->run(FORWARD);
  delay(30);

}

void slight_right() { //small adjustment to the right
  Serial.println("slight right");
  leftMotor->setSpeed(250);
  rightMotor->setSpeed(SLOWSPEED);
  leftMotor->run(FORWARD);
  rightMotor->run(FORWARD);
  delay(20);
}

void slight_left() { //small adjustment to the left
  Serial.println("slight left");
  leftMotor->setSpeed(SLOWSPEED);
  rightMotor->setSpeed(250);
  leftMotor->run(FORWARD);
  rightMotor->run(FORWARD);
  delay(20);
}

void cross_ramp() { //moves forward for a set amount of time using reduced line following
  Serial.println("Crossing ramp");
  long int starttime = millis();

  SLOWSPEED += 70; //Increase the speed of the slower wheel during adjustments, thus make the adjustments smaller

  while ((millis() - starttime < RAMPTIME)) //Exits the loop after a specified amount of time
  {
    readLineSensors(); //update state
    blinker(); //blink amber LED

    if (state == 0b0001 || state == 0b1001) { //just right or right and rear
      adjust_right();
    }

    else if (state == 0b0100 || state == 0b1100) { //just left or left and rear
      adjust_left();
    }

    else if (state == 0b0110 || state == 0b1110) { //left and middle or left, middle and rear
      slight_left();
    }

    else if (state == 0b0011 || state == 0b1011) { //right and middle or right, middle and rear
      slight_right();
    }
    else // move forward and hope it's on the line for every other state
    {
      Serial.println("No adjustment");
      leftMotor->setSpeed(MAXSPEEDL);
      rightMotor->setSpeed(MAXSPEEDR);
      leftMotor->run(FORWARD);
      rightMotor->run(FORWARD);
    }

    delay(5);
  }

  SLOWSPEED -= 70; //change SLOWSPEED back to use normal line following after this point

  //Stop and turn of the blinking LED
  leftMotor->run(RELEASE);
  rightMotor->run(RELEASE);
  digitalWrite(AMBERLED, LOW);
  Serial.println("Ramp has been climbed");

  delay(20);
}

bool blockSearch() //searches for the third block
{

  int distance = 0; //to store the previous reading of the IR sensor
  long int starttime = millis();
  Serial.println("look right");
  //look right
  while ( millis() - starttime < SEARCHTIME )
  {
    readLineSensors();
    blinker();

    //Stop when block is found
    if (IRSensor.getDistance() < 18 && distance < 18) // use the previous value as well to avoid errors caused by noise
    {
      delay(200 - (distance - 9) * 10); //turn a little more to face the centre of the block
      Serial.println("Block found");
      leftMotor->run(RELEASE);
      rightMotor->run(RELEASE);
      digitalWrite(AMBERLED, LOW);
      return true; //block is in right half of square

    } else {
      Serial.println("rotating!");
      rightMotor->run(BACKWARD);
      leftMotor->run(FORWARD);
      rightMotor->setSpeed(MAXSPEEDR / 2);
      leftMotor->setSpeed(MAXSPEEDL / 2);
      delay(10);
    }
    Serial.print("Distance " + String(distance));
    distance = IRSensor.getDistance(); //update previous value
    delay(5);
  }

  //turn back to starting position
  turnLeftAlign();

  Serial.println("look left");
  //look left
  starttime = millis();
  while ( (millis() - starttime < SEARCHTIME ))
  {
    readLineSensors();
    blinker();

    if (IRSensor.getDistance() < 18 && distance < 18) // use the previous value as well to avoid errors caused by noise
    {
      delay(200 - (distance - 9) * 10); //turn a little more to face the centre of the block
      Serial.println("Block found");
      leftMotor->run(RELEASE);
      rightMotor->run(RELEASE);
      digitalWrite(AMBERLED, LOW);
      return false;  //block is in left half of square

    } else {
      //Serial.println("rotating!");
      rightMotor->run(FORWARD);
      leftMotor->run(BACKWARD);
      rightMotor->setSpeed(MAXSPEEDR / 2);
      leftMotor->setSpeed(MAXSPEEDL / 2);
      delay(10);
    }
    distance = IRSensor.getDistance();
    //Serial.println("distance: " + String(IRSensor.getDistance()));
    delay(5);
  }

  turnRightAlign(); //if the block is not found, return to starting position


}


void align() //function used to align the rear sensor with the line if the direction of the deviation is not known, starts turning clockwise, then makes larger and larger rotations to find the line
{
  long int timeLimit = 0;
  long int timeStep = 500;
  bool clockwise = false;
  long int starttime;

  while ((state & 0b1000 ) != 0b1000)
  {
    timeLimit += timeStep; //increase time of sweep
    clockwise = !clockwise; //change direction
    starttime = millis();
    while ((state & 0b1000 ) != 0b1000 && millis() - starttime < timeLimit )
    {

      blinker();
      readLineSensors();

      if (clockwise)
      {
        rightMotor->run(BACKWARD);
        leftMotor->run(FORWARD);
        rightMotor->setSpeed(MAXSPEEDR / 2);
        leftMotor->setSpeed(MAXSPEEDL / 2);
      }
      else
      {
        rightMotor->run(FORWARD);
        leftMotor->run(BACKWARD);
        rightMotor->setSpeed(MAXSPEEDR / 2);
        leftMotor->setSpeed(MAXSPEEDL / 2);
      }
      delay(20);


    }
  }

  //stop when line has been found
  leftMotor->run(RELEASE);
  rightMotor->run(RELEASE);
  digitalWrite(AMBERLED, LOW);


}

bool tripleCheck() //checks the blocktype 50 times (originally 3)
{

  creepForward(100); // to get the block into the optimal position
  drop(servo); //release servo all the way, otherwise it interferes with the measurements
  delay(100); //wait for the robot to stop
  float sum = 0; //the sum of distances returned
  for (int i = 0; i < 50; i++) //do the test 50 times
  {
    sum += detect();
    delay(50);
  }

  if (sum  >= CUTOFF * 50) //if average is above CUTOFF, the block is fine
    digitalWrite(GREENLED, HIGH);
  else
    digitalWrite(REDLED, HIGH);

  Serial.println("average = " + String(sum / 50));


  grab(servo); //grab the block again


  return ((sum >= CUTOFF * 50)); //if average above cutoff, return true

}


void thirdBlock() //function that searches for and grabs the third block then returns to starting position
{
  creepForward(200); //to push the block out of the corner of the box
  moveBackward(0);  //move back to intersection

  block_location = blockSearch(); //find the third block

  creepForward(THIRDBLOCKTIME); //move forward to the other edge of the box
  grab(servo); //grab the block

  Serial.println("block location" + String(block_location));
  creepBackward(1500);
  moveBackwardMiddle(); //move backward to starting position, ignoring the state of middle sensor since it can be on the other edge of the box

  if (block_location) { //turn back parallell to the line
    turnLeftAlign();
  } else {
    turnRightAlign();
  }

  moveBackward(0); //move back to the starting position
  moveBackwardMiddle();

  align(); //find the line with the rear sensor to make reversing more reliable

}



void back_to_start() { //return to starting box after the last delivery

  Serial.println("return to startpoint");

  moveBackward(BACKWARDINTERSECTIONIGNORE); //move back to egde of starting box

  creepBackward(600); //move back to centre of the box

}
