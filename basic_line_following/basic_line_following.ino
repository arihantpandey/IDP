#include "followLine.h"
#include <Wire.h>
#include <AccelStepper.h>
#include <Adafruit_MotorShield.h>


/** Prepare motors **/
Adafruit_MotorShield AFMS(0x60);

Adafruit_StepperMotor *stepperLeftInfo = AFMS.getStepper(200, 1);
Adafruit_StepperMotor *stepperRightInfo = AFMS.getStepper(200, 2);

void forwardStepL()
{
  stepperLeftInfo -> onestep(FORWARD, SINGLE);
}
void backwardStepL()
{
  stepperLeftInfo -> onestep(BACKWARD, SINGLE);
}
void forwardStepR()
{
  stepperLeftInfo -> onestep(FORWARD, SINGLE);
}
void backwardStepR()
{
  stepperLeftInfo -> onestep(BACKWARD, SINGLE);
}

AccelStepper stepperL(forwardStepL, backwardStepL);
AccelStepper stepperR(forwardStepR, backwardStepR);

const int velocityExpansion = 1;


/** Prepare sensor for line following **/
// to be initialized by electrical members
const int lineSensorL = 1;  // left
const int lineSensorR = 3;  // right
const int lineSensorF = 2;  // front
const int lineSensorB = 0;  // back

/* use two slightly different thresholds to avoid bouncing is necessary
const int thresholdW2B = ...; // white to black
const int thresholdB2W = ...; // black to white*/

// to be initialized again in setup(), current values for reference only
bool lineSensorLastStateL = false;
bool lineSensorLastStateR = false;
bool lineSensorLastStateF = true;
bool lineSensorLastStateB = true;

SSensors getLastState()
{
  SSensors lastState(lineSensorLastStateL, lineSensorLastStateR, lineSensorLastStateF, lineSensorLastStateB);
  return lastState;
}

SSensors getCurrentState()
{
  SSensors thisState(
    digitalRead(lineSensorL),
    digitalRead(lineSensorR),
    digitalRead(lineSensorF),
    digitalRead(lineSensorB));

  return thisState;
}

void setLastState(SSensors state)
{
  lineSensorLastStateL = state.left;
  lineSensorLastStateR = state.right;
  lineSensorLastStateF = state.middle;
  lineSensorLastStateB = state.tail;
}


/** main loop **/
void setup() 
{
  Serial.begin(9600);
  
  wheelSpeedGroup[0].leftV = 0;   wheelSpeedGroup[0].rightV = 0;
  wheelSpeedGroup[1].leftV = 10;  wheelSpeedGroup[1].rightV = 10;
  wheelSpeedGroup[2].leftV = -10; wheelSpeedGroup[2].rightV = -10;
  wheelSpeedGroup[3].leftV = 10;  wheelSpeedGroup[3].rightV = -10;
  wheelSpeedGroup[4].leftV = 10;  wheelSpeedGroup[4].rightV = -5;
  wheelSpeedGroup[5].leftV = -5;   wheelSpeedGroup[5].rightV = 10;
  wheelSpeedGroup[6].leftV = 10; wheelSpeedGroup[6].rightV = 0;
  wheelSpeedGroup[7].leftV = 0; wheelSpeedGroup[7].rightV = 10;
  wheelSpeedGroup[8].leftV = 10; wheelSpeedGroup[8].rightV = 7;
  wheelSpeedGroup[9].leftV = 7; wheelSpeedGroup[9].rightV = 10;

  setLastState(getCurrentState());

  AFMS.begin();

  // find the best values in a real robot
  stepperL.setMaxSpeed(100);
  stepperL.setAcceleration(100);

  stepperR.setMaxSpeed(100);
  stepperR.setAcceleration(100);

  Serial.println("Set up");
}

void loop() {
  SSensors sensorThisState = getCurrentState();
  SWheels wheelMovement = simplestFollow(sensorThisState, getLastState());

  Serial.print(sensorThisState.state());
  Serial.print(" ");
  Serial.print(getLastState().state());
  Serial.print(" ");
  Serial.print(wheelSpeedGroup[straitForward].leftV);
  Serial.print(" ");
  Serial.println(wheelMovement.rightV);
  delay(500);

  stepperL.moveTo(stepperL.currentPosition() + (int)(wheelSpeedGroup[straitForward].leftV * velocityExpansion));
  stepperL.moveTo(stepperR.currentPosition() - (int)(wheelSpeedGroup[straitForward].rightV * velocityExpansion));

  stepperL.run();
  stepperR.run();
}
