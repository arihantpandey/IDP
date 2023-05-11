#include <Servo.h>
#include <Adafruit_MotorShield.h>
#include <Wire.h>
#include <AccelStepper.h>
#include <SharpIR.h>
#include "Pinout.h"
#include "Claw.h"
#include "Movement.h" // not used
#include "Distinguisher.h"
#include "Sensors.h"
#include "Settings.h"

uint16_t MAXSPEEDL = 248;
uint16_t MAXSPEEDR = 250;
Servo servo; //the servo that controlls the claw

// Create the motor shield object with the default I2C address
SharpIR IRSensor( SharpIR::GP2Y0A21YK0F, IRDISTANCE );

int state; //The state of the line sensors expressed as a number
bool blocktype; //the type of the block grabbed, true for fine, false for coarse


void setup()
{
  wheelSpeedGroup[0].leftV = 0;
  wheelSpeedGroup[0].rightV = 0;    // fullyStop
  wheelSpeedGroup[1].leftV = 1;
  wheelSpeedGroup[1].rightV = 1;    // straitForward
  wheelSpeedGroup[2].leftV = 1;
  wheelSpeedGroup[2].rightV = -1;   // middle
  wheelSpeedGroup[3].leftV = 0.5;
  wheelSpeedGroup[3].rightV = -1;   // near left
  wheelSpeedGroup[4].leftV = 1;
  wheelSpeedGroup[4].rightV = -0.5; // near right
  wheelSpeedGroup[5].leftV = -0.1;
  wheelSpeedGroup[5].rightV = -1;   // far left
  wheelSpeedGroup[6].leftV = 1;
  wheelSpeedGroup[6].rightV = 0.1;    // far right
  wheelSpeedGroup[7].leftV = -0.56;
  wheelSpeedGroup[7].rightV = -1;   // super far left
  wheelSpeedGroup[8].leftV = 1;
  wheelSpeedGroup[8].rightV = 0.56;  // super far right

  wheelSpeedGroup[5].delay = 30;
  wheelSpeedGroup[6].delay = 30;
  wheelSpeedGroup[7].delay = 20;
  wheelSpeedGroup[8].delay = 20;

  Serial.begin(9600);

  Serial.println("Adafruit Motorshield v2 - DC Motor test!");
  if (!AFMS.begin()) {         // create with the default frequency 1.6KHz
    Serial.println("Could not find Motor Shield. Check wiring.");
    while (1);
  }
  Serial.println("Motor Shield found.");

}



void loopj() //used for testing
{
  pinSetup();
  Serial.println("Waiting for start button");
  while (digitalRead(BUTTON) == HIGH ) {} //Wait for the button press to start
  grab(servo);
  halfRelease(servo);
  digitalWrite(BLUELED4, HIGH);
  moveForward(false, false);
  Foam_Delivery(false); //right turn for coarse block
  digitalWrite(BLUELED4, LOW);

}



void loop() //main loop
{

  pinSetup();
  Serial.println("Waiting for start button");
  while (digitalRead(BUTTON) == HIGH ) {} //Wait for the button press to start

  Serial.println("Starting robot");

  Serial.println("Moving to the edge of the box");
  moveForward(false, false); //Moves to edge of box

  Serial.println("Moving to the drop off area");
  moveForward(false, false); // Moves to drop off area

  Serial.println("Moving to the first block");
  Serial.println("Climbing ramp");
  cross_ramp(); // move forward without navigation as the line sensors dont work on the ramp

  Serial.println("Slowing down");  //Slows down to make downhill navigation more reliable
  MAXSPEEDR -= 100;
  MAXSPEEDL -= 100;
  moveForward(false, true); // Moves to first block
  MAXSPEEDR += 100;
  MAXSPEEDL += 100;

  Serial.println("Grabbing first block");
  grab(servo); //Grabs first block
  blocktype = detect(); //detects blocktypes

  Serial.println("Creeping back");
  creepBackward(1000); //Moves backward to avoid hitting second block during the 180deg turn

  Serial.println("Turning around");
  turnAround(); //turns back towards start area

  halfRelease(servo); // releases the claw halfway to avoid problems with the ramp

  Serial.println("Moving block to the drop off area");
  Serial.println("Climbing ramp");
  cross_ramp();
  Serial.println("Slowing down");
  MAXSPEEDR -= 100;
  MAXSPEEDL -= 100; //slow down robot across and down bridge
  moveForward(false, true); // Moves to drop off area
  MAXSPEEDR += 100;
  MAXSPEEDL += 100; //restore prev speed
  Serial.println("Dropping the first block");

  Foam_Delivery(blocktype);

  Serial.println("First block delivered");

  //First block delivered


  Serial.println("Climbing ramp");
  cross_ramp();
  Serial.println("Slowing down");
  MAXSPEEDR -= 100;
  MAXSPEEDL -= 100;
  moveForward(false, true); // Moves to first block's location
  MAXSPEEDR += 100;
  MAXSPEEDL += 100;

  Serial.println("Searching for second block");
  moveForward(true, false); //move forward looking for the second block

  Serial.println("Grabbing second block");
  creepForward(); //moves farther forward bc the block is too far to grab at this point
  creepForward();
  grab(servo); //grabs second block
  blocktype = detect();

  Serial.println("Moving backward");
  creepBackward(3000); // moves backward to avoid detecting the box with the rear sensor while turning around

  Serial.println("Turning around");
  turnAround();
  halfRelease(servo); // releases the claw halfway to avoid problems with the ramp

  Serial.println("Moving to first block's location");
  moveForward(false, false); //moves to first block's location
  Serial.println("Moving to drop off area");
  moveForward(false, false); // moves to dropoff area

  Serial.println("Dropping the second block");
  Foam_Delivery(blocktype);

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

  stop_movement();

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


String moveForward( bool useIR, bool rash )
{
    if (rash)
    {
        return moveForwardB( useIR );
    }
    else
    {
        return moveForwardA( useIR );
    }
}

String moveForwardB( bool useIR ) //controls the movement of the robot, detects initersections (always) and blocks (if useIR is true).
{
  long int starttime = millis();

  //At an intersection the right and the left sensors are on (other sensors can take any value)
  while (( state & 0b0101) != 0b0101 || millis() - starttime < INTERSECTIONIGNORE) //Ignores intersections for a bit to avoid detecting it twice, exits when it's at an intersection
  {
    readLineSensors(); //update state
    blinker(); //blink amber LED

    if (useIR && IRSensor.getDistance() == 9) //Stop moving if block is found
    {
      stop_movement();
      digitalWrite(AMBERLED, LOW);
      Serial.println("Block found");
      return "Block";
    }
    else if (state == 0b0001) { //just right
      adjust_right();
    }

    else if (state == 0b0100) { //just left
      adjust_left();
    }

    else if (state == 0b0110) { //left and middle
      slight_right();
    }

    else if (state == 0b0011) { //right and middle
      slight_left();
    }
    else// move forward and hope it's on the line for every other state
    {
      Serial.println("No adjustment");
      strait_move(true, 1);
    }

    delay(5);
  }

  //Stop at the intersection and turn of the blinking LED
  stop_movement();
  digitalWrite(AMBERLED, LOW);
  Serial.println("Intersection found");
  return "Intersection";

  delay(20);
}

String moveForwardA( bool useIR ) //controls the movement of the robot, detects initersections (always) and blocks (if useIR is true).
{
  unsigned long int starttime = millis();
  int robotState = 0;
  bool rush_to_ramp = false;
  int ramp_dis = 100;
  int count_dis = 0;

  while ( ( state & 0b0101) != 0b0101 || millis() - starttime < INTERSECTIONIGNORE ) //Ignores intersections for a bit to avoid detecting it twice, exits when its at an intersection
  {
    if ( rush_to_ramp )
    // find the ramp
    {
      strait_move(true, 1);
      digitalWrite(REDLED, HIGH);
      Serial.println("Ramp");
      delay(3000);
      digitalWrite(REDLED, LOW);
      rush_to_ramp = false;
      ramp_dis = 100;
      count_dis = 0;
    }
    else
    {
      robotState = follow_command( GOFORWARD );
    }

    if (millis() % 100 == 0)
    {
      Serial.println(IRSensor.getDistance());
    }
    if (IRSensor.getDistance() <= ramp_dis)
    {
      ramp_dis = IRSensor.getDistance();
    }
    else
    {
      // bouncing
      ramp_dis = 100;
    }

    if (ramp_dis <= 14)
    {
      count_dis ++;
    }
    else
    {
      count_dis = 0;
    }

    if( count_dis >= 2 )
    {
      rush_to_ramp = true;
    }

    blinker();

    if ( useIR && IRSensor.getDistance() == 9 ) //Stop moving if block is found
    {
      stop_movement();
      digitalWrite( AMBERLED, LOW );
      Serial.println( "Block found" );
      return "Block";
    }

    delay( 20 );
  }
  stop_movement();
  digitalWrite( AMBERLED, LOW );
  Serial.println( "Intersection found" );
  return "Intersection";
}

void turnAround() // does a turn until the rear sensor gets back on the line again
{

  long int starttime = millis();
  //while(rear sensor low || still in the ignore time)
  while ( (state & 0b1000) != 0b1000 || millis() - starttime < TURNDELAY) //Turn until rear sensor gets on the line again, avoid checking for a bit to make sure it clears the original line
  {
    readLineSensors();
    blinker(); //blinks the amber LED
    rotate(false, 0.5);
    delay(50);
  }

  //a delay here to get the robot pointing straight
  delay(300);
  //stop turning and turn the blinking LED off
  digitalWrite(AMBERLED, LOW);
  stop_movement();
}


void turnRightTime() //first rotation to face red box, fully time based function
{
  Serial.println("Turning right, time based");
  long int starttime = millis();
  while (millis() - starttime < TURNTIME)
  {
    blinker();
    rotate(true, 0.5);
    delay(50);
  }

  //digitalWrite(BLUELED1, HIGH);
  delay(30);
  //digitalWrite(BLUELED1, LOW);
  stop_movement();
  digitalWrite(AMBERLED, LOW);
}


void turnRightJunction() // after the block has been delivered turn around to collect second block, uses junction counting
{
  Serial.println("Turning right, junction based");
  long int starttime = millis();

  int rear_detection_count = 0;
  bool rear_detection_check = false;
  while (rear_detection_count < 2)
  {
    readLineSensors();
    rear_detection_check = (state >= 8);
    if (rear_detection_check == true) {
      rear_detection_count += 1;
      digitalWrite(BLUELED1, HIGH);
      rear_detection_check = false;
      delay(800);
    }
    digitalWrite(BLUELED1, LOW);

    blinker();
    rotate(true, 0.5);
    delay(50);
  }

  stop_movement();
  digitalWrite(AMBERLED, LOW);
}

void turnRightAllign() {
  Serial.println("Turning right, alligning to line");
  while (state < 8) //Turn until rear sensor gets on the line again
  {
    readLineSensors();
    blinker(); //blinks the amber LED
    rotate(true, 0.5);
    delay(50);
  }

  //maybe a delay here to get the robot pointing straight
  delay(300);
  //stop turning when the rear sensor is above the line
  digitalWrite(AMBERLED, LOW);
  stop_movement();
}

void turnLeftTime() //first rotation to face blue box, fully time based function
{
  Serial.println("Turning left, time based");
  long int starttime = millis();

  while (millis() - starttime < TURNTIME)
  {
    blinker();
    rotate(false, 0.5);
    delay(50);
  }
  //digitalWrite(BLUELED1, HIGH);
  delay(30);
  //digitalWrite(BLUELED1, LOW);
  stop_movement();
  digitalWrite(AMBERLED, LOW);
}


void turnLeftJunction() // after the block has been delivered turn around to collect second block, uses junction counting
{
  Serial.println("Turning left, junction based");
  long int starttime = millis();

  /*might add a delay so that doesn't detect junction twice*/
  /*delay(20)*/

  int rear_detection_count = 0;
  bool rear_detection_check = false;
  while (rear_detection_count < 2)
  {
    readLineSensors();
    rear_detection_check = (state >= 8);
    if (rear_detection_check == true) {
      rear_detection_count += 1;
      digitalWrite(BLUELED1, HIGH);
      rear_detection_check = false;
      delay(800);
    }
    digitalWrite(BLUELED1, LOW);

    blinker();
    rotate(false, 0.5);
    delay(50);
  }


  stop_movement();
  digitalWrite(AMBERLED, LOW);
}

void turnLeftAllign() {
  Serial.println("Turning left, alligning to line");
  while (state < 8) //Turn until rear sensor gets on the line again
  {
    readLineSensors();
    blinker(); //blinks the amber LED
    rotate(false, 0.5);
    delay(50);
  }

  //a delay here to get the robot pointing straight
  delay(300);
  //stop turning when the rear sensor is above the line
  digitalWrite(AMBERLED, LOW);
  stop_movement();
}

void creepForward() // move forward for the time specified as CREEPFORWARDTIME
{
  long int starttime = millis();
  while (millis() - starttime < CREEPFORWARDTIME)
  {
    readLineSensors();
    blinker();
    strait_move(true, 1);
    delay(20);

  }

  stop_movement();
  digitalWrite(AMBERLED, LOW);

}


void moveBackward() //move backwards until junction detected
{
  Serial.println("Moving backward until intersection is detected");
  long int starttime = millis();
  /*int junction_count = 0;
    bool intersection_check = false;

    while (millis() - starttime < 2000) // move backwards half a second to ignore the junction
    {
    blinker();
    leftMotor->run(BACKWARD);
    rightMotor->run(BACKWARD);
    leftMotor->setSpeed(MAXSPEEDL);
    rightMotor->setSpeed(MAXSPEEDR);
    delay(20);
    }
    while (junction_count < 1 )// keep moving backwards until the junction is detected again
    {
    readLineSensors();
    intersection_check = (state == 0b0111) or (state == 0b1111);
    if (intersection_check == true) {
      junction_count += 1;
      digitalWrite(BLUELED2, HIGH);
      intersection_check = false;
      delay(50);
    }
    digitalWrite(BLUELED2, LOW);

    blinker();
    leftMotor->run(BACKWARD);
    rightMotor->run(BACKWARD);
    leftMotor->setSpeed(MAXSPEEDL);
    rightMotor->setSpeed(MAXSPEEDR);
    delay(20);

    }

    leftMotor->run(RELEASE);
    rightMotor->run(RELEASE);
    digitalWrite(AMBERLED, LOW);
  */
  while ( ( state & 0b0101) != 0b0101 ||  millis() - starttime < BACKWARDINTERSECTIONIGNORE) // move backwards half a second to ignore the junction
  {
    blinker();
    readLineSensors();
    strait_move(false, 1);
    delay(20);
  }
  //stop moving when intersection is detected
  stop_movement();
  digitalWrite(AMBERLED, LOW);


}

void moveBackwardMiddle() { //move backwards until the middle sensor is on the white line to prepare for line following
  Serial.println("Moving back until middle sensor is on the line");
  while ((state & 0b0010) != 0b0010) {
    readLineSensors();
    blinker();
    strait_move(false, 1);
    delay(20);
  }

  stop_movement();
  digitalWrite(AMBERLED, LOW);
}

void creepBackward(int delay_time) { //move backward for a specified time

  Serial.println("Creeping back for " + String(delay_time) + " ms");
  long int starttime = millis();

  while (millis() - starttime < delay_time) // move backwards for delay_time
  {
    blinker();
    strait_move(false, 1);
    delay(20);
  }

  //stop after the specified time
  stop_movement();
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
    turnLeftTime();
    //digitalWrite(BLUELED4, LOW);
    creepForward();
    //digitalWrite(BLUELED4, HIGH);
    drop(servo);
    moveBackward();
    //digitalWrite(BLUELED4, LOW);
    turnLeftJunction();
    moveBackwardMiddle();
    turnRightAllign();

  }
  else if (blocktype == false) { //coarse to red area
    turnRightTime();
    //digitalWrite(BLUELED4, LOW);
    creepForward();
    //digitalWrite(BLUELED4, HIGH);
    drop(servo);
    moveBackward();
    //digitalWrite(BLUELED4, LOW);
    turnRightJunction();
    moveBackwardMiddle();
    turnLeftAllign();
  }
}



void adjust_right() {
  Serial.println("Adjusting right");
  Serial.println(state);
  /*while(state != 0b0100 || state != 0b0110) { //adjust until left or left,middle are active
    readLineSensors();
    blinker()*/;
  move_with(1, 0.56);
  delay(30);

}


void adjust_left() { //adjust until right or right,middle are active
  Serial.println("Adjusting left");
  //Serial.println(state);
  /*while(state != 0b0001 || state != 0b0011) {
    readLineSensors();
    Serial.println(state);
    blinker();  */

  move_with(1,0.56);
  delay(30);

}

void slight_right() {
  Serial.println("slight right");
  move_with(1,0.56);
  delay(20);
}

void slight_left() {
  Serial.println("slight left");
  move_with(0.56, 1);
  delay(20);
}

void cross_ramp() {
  long int starttime = millis();


  while (millis() - starttime < RAMPTIME) //exits after time taken to go up ramp
  {
    readLineSensors();
    blinker();
    strait_move(true, 1);

    delay(20);

  }
  stop_movement();
  digitalWrite(AMBERLED, LOW);
  Serial.println("ramp has been climbed");
  delay(20);
}

/*void blockSearch() //searches for the third block
{
  //TODO: check if block is on line

  int distance = 0;
  long int starttime = millis();

  //look right
  while ( (state & 0b1000 != 0b1000 || millis() - starttime < TURNDELAY ))
  {
  readLineSensors();
    blinker();

    rightMotor->run(BACKWARD);
    leftMotor->run(FORWARD);
    rightMotor->run(MAXSPEEDR / 2);
    leftMotor->run(MAXSPEEDL / 2);

    if (IRSensor.getDistance() < distance - 10) //if there is a large change, the block is found
    {
      Serial.println("Block found");
      leftMotor->run(RELEASE);
      rightMotor->run(RELEASE);
      digitalWrite(AMBERLED, LOW);
      return;

    }

    Serial.println("distance: " + String(IRSensor.getDistance()));
                   delay(20);
  }

  //turn back to starting position
  turnAround();

  //look left
  while ( (state & 0b1000 != 0b1000 || millis() - starttime < TURNDELAY ))
  {
  readLineSensors();
    blinker();

    rightMotor->run(FORWARD);
    leftMotor->run(BACKWARD);
    rightMotor->run(MAXSPEEDR / 2);
    leftMotor->run(MAXSPEEDL / 2);

    if (IRSensor.getDistance() < distance - 10) //if there is a large change, the block is found
    {
      Serial.println("Block found");
      leftMotor->run(RELEASE);
      rightMotor->run(RELEASE);
      digitalWrite(AMBERLED, LOW);
      return;

    }

    Serial.println("distance: " + String(IRSensor.getDistance()));
    delay(20);
  }



}*/


/*void intersection_align() { //not used atm
  Serial.println("aligning to intersection");
  while ((state & 1010) != 1010) {
    blinker();
    leftMotor->run(BACKWARD);
    rightMotor->run(FORWARD);
    leftMotor->setSpeed(MAXSPEEDL / 2);
    rightMotor->setSpeed(MAXSPEEDR / 2);
    readLineSensors();
    delay(50);
  }
}*/
