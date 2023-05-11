#include <Servo.h>
#include <Adafruit_MotorShield.h>
//#include "followLine.h"
#include <Wire.h>
#include <AccelStepper.h>
#include <SharpIR.h>
#include "Pinout.h"
#include "Claw.h"
#include "Movement.h"
#include "Distinguisher.h"

uint16_t MAXSPEED = 250;
#define SLOWSPEED 140
#define RAMPTIME 5700
#define TURNTIME 5000 
#define CREEPTIME 500
#define CREEPFORWARDTIME 1000
#define CREEPBACKWARDTIME 1000 
#define INTERSECTIONIGNORE 300
#define TURNDELAY 1500

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
long int rotation_time; 


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


void loop() {
   Serial.println("LOOp");
  /*pinSetup();
  grab(servo);
  halfrelease(servo);
  digitalWrite(BLUELED4, HIGH);
  moveForward(false);
  turnLeft();
  digitalWrite(BLUELED4, LOW);
  creepForward();
  digitalWrite(BLUELED4, HIGH);
  drop(servo);
  creepBackward();
  digitalWrite(BLUELED4, LOW);
  turnLeft2();
  delay(10000);*/

  pinSetup();
Serial.println("Waiting for start button");
while (digitalRead(BUTTON) == HIGH ) {} //Wait for the button press to start
grab(servo);
  halfrelease(servo);
  digitalWrite(BLUELED4, HIGH);
  moveForward(false);
  turnLeft();
  digitalWrite(BLUELED4, LOW);
  creepForward();
  digitalWrite(BLUELED4, HIGH);
  drop(servo);
  creepBackward();
  digitalWrite(BLUELED4, LOW);
  turnLeft2();
/*creepForward();
creepBackward();
turnLeft2();*/

/*pinSetup();
turnAround();*/
}
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

}

void readLineSensors() //reads the inputs of the line sensors
{
  state = digitalRead(REAR) * 8 + digitalRead(LEFT) * 4 + digitalRead(MIDDLE) * 2 + digitalRead(RIGHT) * 1;
  //Serial.println("Line sensor state: " + String(state));
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

  //at this point the rear sensor is just off the line, the robot still points in the original direction


  while (state < 8 || millis() - starttime < TURNDELAY) //Turn until rear sensor gets on the line again
  {
    readLineSensors();
    blinker(); //blinks the amber LED
    leftMotor->run(BACKWARD);
    rightMotor->run(FORWARD);
    leftMotor->setSpeed(MAXSPEED / 2);
    rightMotor->setSpeed(MAXSPEED / 2);
    delay(50);
  }

rotation_time = millis() - starttime;
  Serial.println("rotation_time for 180 degrees is: " + String(rotation_time));
  //maybe a delay here to get the robot pointing straight
  delay(300);
  //stop turning when the rear sensor is above the line
  digitalWrite(AMBERLED, LOW);
  leftMotor->run(RELEASE);
  rightMotor->run(RELEASE);
}


void turnRight()
{
  Serial.println("Turning right");
  long int starttime = millis();
  while((state != 0b0111 and state!= 0b1111) or (millis() - starttime < TURNTIME))
  {
    readLineSensors();
    blinker();
    leftMotor->run(FORWARD);
    rightMotor->run(BACKWARD);
    leftMotor->setSpeed(MAXSPEED / 2);
    rightMotor->setSpeed(MAXSPEED / 2);
    delay(50);
  }

  leftMotor->run(RELEASE);
  rightMotor->run(RELEASE);
  digitalWrite(AMBERLED, LOW);

}

void turnLeft()
{
  Serial.println("Turning left");
  long int starttime = millis();
  while (millis() - starttime < 4000)
  {
   leftMotor->run(BACKWARD);
    rightMotor->run(FORWARD);
    leftMotor->setSpeed(MAXSPEED / 2);
    rightMotor->setSpeed(MAXSPEED / 2);
    delay(50);
  }
    
  while ((state&0b0110) == 0b0110)
  {
    readLineSensors();
    blinker();
    leftMotor->run(BACKWARD);
    rightMotor->run(FORWARD);
    leftMotor->setSpeed(MAXSPEED / 2);
    rightMotor->setSpeed(MAXSPEED / 2);
    delay(50);
  }
  digitalWrite(BLUELED1,HIGH);
  delay(30);
  digitalWrite(BLUELED1,LOW);
  leftMotor->run(RELEASE);
  rightMotor->run(RELEASE);
  digitalWrite(AMBERLED, LOW);
}


void turnLeft2()
{
  Serial.println("Turning left");
  long int starttime = millis();
  int rear_detection_count = 0;
  bool rear_detection_check = false;
  rotation_time = 8000;
 while(rear_detection_count <1) //and (millis()-starttime < rotation_time/2))

  {
    readLineSensors();
    rear_detection_check = (state >= 8);
    if(rear_detection_check == true){
   rear_detection_count += 1;
   digitalWrite(BLUELED1, HIGH);
    rear_detection_check = false;
    delay(300);}
    digitalWrite(BLUELED1, LOW);

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

void creepForward()
{
  long int starttime = millis();
  /*while((millis()-starttime < CREEPFORWARDTIME) or (state != 0b0111 and state!= 0b1111))*/
  while(millis()-starttime < CREEPFORWARDTIME)
  /*check the creep backward time*/
  {
    readLineSensors();
    blinker();
    leftMotor->run(FORWARD);
    rightMotor->run(FORWARD);
    leftMotor->setSpeed(MAXSPEED);
    rightMotor->setSpeed(MAXSPEED);
    delay(20);
    
  }

  leftMotor->run(RELEASE);
  rightMotor->run(RELEASE);
  digitalWrite(AMBERLED, LOW);
  
}

void creepBackward()
{
  long int starttime = millis();
  int junction_count = 0;
  bool intersection_check = false;

    while(junction_count < 3)
  {
    readLineSensors();
    intersection_check = (state == 0b0111) or (state == 0b1111);
    if(intersection_check == true){
    junction_count += 1;
    digitalWrite(BLUELED2, HIGH);
    intersection_check = false;
    delay(50);
    }
    digitalWrite(BLUELED2, LOW);
    
    blinker();
    leftMotor->run(BACKWARD);
    rightMotor->run(BACKWARD);
    leftMotor->setSpeed(MAXSPEED);
    rightMotor->setSpeed(MAXSPEED);
    delay(20);
    
  }

  leftMotor->run(RELEASE);
  rightMotor->run(RELEASE);
  digitalWrite(AMBERLED, LOW);
  
}


void blinker() // if this function is called often enough the amber LED will blink at 2Hz
{
  digitalWrite(AMBERLED, millis() % 500 < 250);
}
