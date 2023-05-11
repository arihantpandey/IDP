#include "followLine.h"
#include <Wire.h>
#include <Adafruit_MotorShield.h>
#include <AceRoutine.h>
#include <Claw.h>
#include <Distinguisher.h>
#include <Servo.h>

/** Prepare motors **/
Adafruit_MotorShield AFMS( 0x60 );

Adafruit_DCMotor *motorLeft = AFMS.getMotor( 1 );
Adafruit_DCMotor *motorRight = AFMS.getMotor( 4 );

const double basicVelocity = 150;
SWheels wheelSpeedGroup[11];


/** Prepare sensor for line following **/
// to be initialized by electrical members
const int lineSensorL = 1;  // left
const int lineSensorR = 3;  // right
const int lineSensorF = 2;  // front
const int lineSensorB = 0;  // back

// to be initialized again in setup(), current values for reference only
static bool lineSensorLastStateL = false;
static bool lineSensorLastStateR = false;
static bool lineSensorLastStateF = true;
static bool lineSensorLastStateB = true;

//intialize servo
Servo myservo;

bool held_block = false;
bool fine_block = false;

SSensors getLastState()
{
	SSensors lastState( lineSensorLastStateL, lineSensorLastStateR, lineSensorLastStateF, lineSensorLastStateB );
	return lastState;
}

SSensors getCurrentState()
{
	SSensors thisState(
	    digitalRead( lineSensorL ),
	    digitalRead( lineSensorR ),
	    digitalRead( lineSensorF ),
	    digitalRead( lineSensorB ) );

	return thisState;
}

void setLastState( SSensors *state )
{
	lineSensorLastStateL = (*state).left;
	lineSensorLastStateR = (*state).right;
	lineSensorLastStateF = (*state).middle;
	lineSensorLastStateB = (*state).tail;
}

//runtime movement functions for when block is secure and identified

void turnAround() {
  motorLeft -> setSpeed( basicVelocity );
  motorRight -> setSpeed( basicVelocity );
  
  motorLeft -> run( FORWARD );
  motorRight -> run( BACKWARD );
  delay(50);
  
  while(sensorThisState.state() != 0b1100) {
    motorLeft -> run( FORWARD );
    motorRight -> run( BACKWARD );
  }
}

//block is fine, robot is turned around, go back to first intersection and turn left
void goToBlue() {
  
}

//block is coarse, robot is turned around, go back to first intersection and turn right
void goToRed() {
  
}


/** prepare front sensor **/
const int front_sensor_pin = A0;
int get_cube_distance
{
  return analogRead( front_sensor_pin );
}


/** prepare LED **/
const int LED1_pin = 5;
const int blink_time = 100;
const int reset_time = 60000;
static bool do_blink = false;
COROUTINE(blink_LED1)
{
  COROUTINE_LOOP()
  {
    if (do_blink)
    {
      digitalWrite(LED1_pin, HIGH);
      COROUTINE_DELAY(blink_time);
    }
    digitalWrite(LED1_pin, LOW);
    COUTINE_DELAY(blink_time);
  }
}

COROUTINE(button) {
  static int buttonState = HIGH;
  static int prevButtonState = HIGH;

  COROUTINE_LOOP() {
    buttonState = digitalRead(BUTTON_PIN);
    if (prevButtonState == HIGH && buttonState == LOW) {
      // primitive debouncing
      COROUTINE_DELAY(20);
      buttonState = digitalRead(BUTTON_PIN);
      if (prevButtonState == HIGH && buttonState == LOW) {
        if (!do_blink)
        {
          do_blink = true;
          COROUTINE_DELAY(reset_time);
        }
        else
        {
          do_blink = false;
        }
      }
    }
    prevButtonState = buttonState;
    COROUTINE_DELAY(20);
  }
}

COROUTINE(grabBlock) {
  COROUTINE_LOOP() {
    if(get_cube_distance < 9) {
      if(Claw.grab(myservo)) {
        if(Distinguisher.detect()) { 
          held_block = true;
          fine_block = true;
          break;
        } else {
          held_block = true;
          fine_block = false;
          break;
        }
      } 
    }
  }
}





/** main loop **/
void setup()
{
	Serial.begin( 9600 );

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
	wheelSpeedGroup[5].leftV = 0;
	wheelSpeedGroup[5].rightV = -1;   // far left
	wheelSpeedGroup[6].leftV = 1;
	wheelSpeedGroup[6].rightV = 0;    // far right
	wheelSpeedGroup[7].leftV = -0.5;
	wheelSpeedGroup[7].rightV = -1;   // super far left
	wheelSpeedGroup[8].leftV = 1;
	wheelSpeedGroup[8].rightV = 0.5;  // super far right

  SSensors sensorInit = getCurrentState();
	setLastState( &sensorInit );

	AFMS.begin();

	// find the best values in a real robot
	motorLeft -> setSpeed( basicVelocity );
	motorRight -> setSpeed( basicVelocity );

	motorLeft -> run( FORWARD );
	motorRight -> run( FORWARD );

  // pin init
  pinMode(LED1_pin, OUTPUT);

  // coroutine init
  CoroutineScheduler::setup();
}

void loop() {
  CoroutineScheduler::loop();
  
	static int lineFollowInstruction = 0;
	static int wheelMovementNumber = 0;
	static int stateNumber = 0;
	static double meanVelocity = 0;
  
	SSensors sensorThisState = getCurrentState();
	lineFollowInstruction = advancedFollow( sensorThisState.state(), getLastState().state(), meanVelocity >= 0, 0, GOFORWARD );
  wheelMovementNumber = ( lineFollowInstruction % 256 ) - ( ( lineFollowInstruction % 256 > 127 ) ? 256 : 0 );
  stateNumber = lineFollowInstruction - wheelMovementNumber;

  // for debugging
  if ( sensorThisState.state() != getLastState().state() )
  {
      Serial.println( "A change" );
      Serial.print( "\tmovement number: " );
      Serial.println( wheelMovementNumber );
      Serial.print( "\tstate number: " );
      Serial.println( stateNumber>>8 );
      Serial.print( "\tsensor state: \n\t\tnow" );
      Serial.println( sensorThisState.state() );
      Serial.print( "\t\tlast" );
      Serial.println( getLastState().state() );
      Serial.print( "\tcentral speed: " );
      Serial.println( meanVelocity );
      if ( stateNumber == AtCross )
        Serial.println( "\tEntering cross." );
      if ( stateNumber == LeaveCross )
        Serial.println( "\tLeaving cross" );
  }

  setLastState( &sensorThisState );

  if ( stateNumber == UNEXPECTED_ERROR || stateNumber == REDUNDANT_ERROR )
  {
    motorLeft -> run( RELEASE );
    motorRight -> run( RELEASE );
    meanVelocity = 0;
  }
	else if ( stateNumber == unchanged )
	{
    if (sensorThisState.state() == 0b1100 && meanVelocity < 0.5)
    {
      motorLeft -> setSpeed( basicVelocity );
      motorRight -> setSpeed( basicVelocity );
    
      motorLeft -> run( FORWARD );
      motorRight -> run( FORWARD );
    }
	}
	else if ( stateNumber == nothing || stateNumber == AtCross || stateNumber == LeaveCross )
		/// later on cross states will be separated from this
	{
		double leftV = wheelMovementNumber > 0 ? wheelSpeedGroup[wheelMovementNumber].leftV : -wheelSpeedGroup[-wheelMovementNumber].leftV;
		double rightV = wheelMovementNumber > 0 ? wheelSpeedGroup[wheelMovementNumber].rightV : -wheelSpeedGroup[-wheelMovementNumber].rightV;
		meanVelocity = ( leftV + rightV ) / 2;

		if ( leftV > 0 )
		{
			motorLeft -> setSpeed( ( uint8_t )( basicVelocity * leftV ) );
			motorLeft -> run( FORWARD );
		}
		else
		{
			motorLeft -> setSpeed( ( uint8_t )( -basicVelocity * leftV ) );
			motorLeft -> run( BACKWARD );
		}

		// right wheel is not reversed
		if ( rightV > 0 )
		{
			motorRight -> setSpeed( ( uint8_t )( basicVelocity * rightV ) );
			motorRight -> run( FORWARD );
		}
		else
		{
			motorRight -> setSpeed( ( uint8_t )( -basicVelocity * rightV ) );
			motorRight -> run( BACKWARD );
		}
	}
	else
		/// there is an error
	{
		motorLeft -> run( RELEASE );
		motorRight -> run( RELEASE );
	}

	delay( 10 );

	//motorLeft -> run( RELEASE );
	//motorRight -> run( RELEASE );

	//delay( 10 );

  if(fine_block and held_block) {
    turnAround();
    goToBlue();
  }
  if(fine_block and held_block) {
    turnAround();
    goToRed();
  }
  

  
}
