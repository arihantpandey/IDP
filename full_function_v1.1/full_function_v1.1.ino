#include "followLine.h"
#include <Wire.h>
#include <Servo.h>
#include <Adafruit_MotorShield.h>


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


/** prepare claw **/
#define POS_OPEN 130
#define POS_CLOSED 180 //Claw end states
#define TURNTIME 50 //time it takes to turn 1 deg (millisec)
#define THRESHOLD 15000 //(used to differentiate the blocks)
#define REDLED 4 
#define GREENLED 5 //LEDs used to show block type
Servo servo;

bool grab() //closes the claw and returns true for fine block, false for coarse block
{
    Serial.println("Closing the claw");

    int integral = 0; // used for current measurement

    for (int pos = POS_OPEN; pos <= POS_CLOSED; pos++) //close the claw
    {
        servo.write(pos);
        integral += analogRead(A0);
        //Serial.println("Position: " + String(pos));
        delay(TURNTIME);
    }

    if (integral > THRESHOLD) //true for fine
    {
        Serial.println("Fine");
        digitalWrite(GREENLED, HIGH);
        return true;
    }
    else
    {
        Serial.println("Coarse");
        digitalWrite(REDLED, HIGH);
        return false;
    }
}

void drop() //opens the claw
{
    Serial.println("Opening the claw");

    for (int pos = POS_CLOSED; pos >= POS_OPEN; pos--) //open the claw
    {
        servo.write(pos);
        //Serial.println("Position: " + String(pos));
        delay(TURNTIME);
    }

    digitalWrite(GREENLED, LOW);
    digitalWrite(REDLED, LOW); //turns off the blocktype LEDs
}


/** prepare front sensor **/
const int front_sensor_pin = A0;
int get_cube_distance()
{
  return analogRead( front_sensor_pin );
}


/** prepare LED **/
const int LED1_pin = 5;
const int BUTTON_PIN = 4;
const int blink_time = 100;
const int reset_time = 60000;
static bool do_blink = false;
void blink_LED1()
{
  static bool LED1_on = false;
  static unsigned int last_called = millis();

  if (do_blink)
  {
    if (millis() - last_called >= blink_time)
    {
      LED1_on = !LED1_on;
      digitalWrite(LED1_pin, LED1_on ? HIGH : LOW);
    }
  }
}

void button()
{
  static int last_called = -1;

  if (last_called < -1)
  {
    if (millis() + last_called > 100)
    {
      last_called = -1;
    }
    else
    {
      return;
    }
  }
  
  if (digitalRead(BUTTON_PIN) == LOW) 
  {
    // primitive debouncing
    if (last_called == -1)
    {
      last_called = millis();
    }
    else if (millis() - last_called > 20)
    {
      if (!do_blink)
      {
        do_blink = true;
        last_called = -millis();
      }
      else
      {
        do_blink = false;
        last_called = -millis();
      }
    }
  }
}

void co_routines()
{
  blink_LED1();
  button();
}


/** modulus **/
int follow_command( int cmd )
{
  static int lineFollowInstruction = 0;
  static int wheelMovementNumber = 0;
  static int stateNumber = 0;
  static double meanVelocity = 0;
  
  SSensors sensorThisState = getCurrentState();
  lineFollowInstruction = advancedFollow( sensorThisState.state(), getLastState().state(), meanVelocity >= 0, 0, cmd );
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

  if ( stateNumber == UNEXPECTED_ERROR || stateNumber == REDUNDANT_ERROR || stateNumber == ROTATE_CONFLICT_ERROR )
  {
    motorLeft -> run( RELEASE );
    motorRight -> run( RELEASE );
    meanVelocity = 0;
  }
  else if ( stateNumber == unchanged )
  {
    if (sensorThisState.state() == 0b1100 && meanVelocity < 0.5)
      // restart, hoping problem 
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
  else if ( stateNumber == RotateEnd )
  {
    motorLeft -> run( RELEASE );
    motorRight -> run( RELEASE );
  }
  return stateNumber;
}

void rotate(bool clockwise, double relative_speed)
{
  motorLeft -> setSpeed( ( uint8_t )( basicVelocity * relative_speed ) );
  motorRight -> setSpeed( ( uint8_t )( basicVelocity * relative_speed ) );
  
  if (clockwise)
  {
    motorLeft -> run( FORWARD );
    motorRight -> run( BACKWARD );
  }
  else
  {
    motorLeft -> run( BACKWARD );
    motorRight -> run( FORWARD );
  }
}

void rotate_until_tail_on_line(unsigned int basic_test_time, double relative_speed)
{
  static bool clockwise = true;
  static unsigned int test_time = basic_test_time;
  static unsigned int last_called = millis();

  if (millis() - last_called > test_time)
  {
    clockwise = !clockwise;
    test_time += basic_test_time;
  }
  rotate(clockwise, relative_speed);
}

/** main loop **/
static int check_point_number;
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
  servo.attach(10);

  // other init
  check_point_number = 0;
}

void loop() {
  co_routines();
  static bool cube_type;
  static unsigned int last_called = millis();
  
	switch(check_point_number)
  {
    case 0:
    // go forward until meet the first cube
    {
      if ( get_cube_distance() < 9 )
      {
        check_point_number ++;
        follow_command( GRAB );
        break;
      }
      else
      {
        follow_command( GOFORWARD );
      }
      break;
    }
    case 1:
    // grab the cube and get its property
    {
      cube_type = grab();
      check_point_number ++;
      break;
    }
    case 2:
    // prepare to rotate 180 degrees
    {
      if (getCurrentState().tail == false)
      {
        rotate_until_tail_on_line(150, 0.6);
      }
      else
      {
        check_point_number ++;
        motorLeft -> run( RELEASE );
        motorRight -> run( RELEASE );
      }
      break;
    }
    case 3:
    // rotate 180 degrees
    {
      if( follow_command( ROTATE_C ) == RotateEnd )
      {
        check_point_number ++;
        motorLeft -> run( RELEASE );
        motorRight -> run( RELEASE );
        last_called = millis();
      }
      break;
    }
    case 4:
    // go back until enter the first cross
    {
      int state = follow_command( GOFORWARD );
      if (millis() - last_called > 5000 && state == AtCross)
      // first condition is for avoiding stopping at the cross where the cube is grabbed
      {
        check_point_number ++;
        motorLeft -> run( RELEASE );
        motorRight -> run( RELEASE );
      }
      break;
    }
    case 5:
    // prapare to rotate to the right direction
    {
      if (getCurrentState().tail == false)
      {
        rotate_until_tail_on_line(150, 0.6);
      }
      else
      {
        check_point_number ++;
        motorLeft -> run( RELEASE );
        motorRight -> run( RELEASE );
      }
      break;
    }
    case 6:
    // rotate to the right direction
    {
      if( follow_command( cube_type ? ROTATE_C : ROTATE_A ) == RotateEnd ) // !!may need to reverse direction
      {
        check_point_number ++;
        motorLeft -> run( RELEASE );
        motorRight -> run( RELEASE );
        last_called = millis();
      }
      break;
    }
    case 7:
    // go forward until meed the boundary lines
    {
      int state = follow_command( GOFORWARD );
      if (millis() - last_called > 1000 && state == AtCross)
      // first condition is for avoiding stopping at the cross where the rotation is finished
      {
        check_point_number ++;
        motorLeft -> run( RELEASE );
        motorRight -> run( RELEASE );
      }
      break;
    }
    case 8:
    {
      drop();
      check_point_number ++;
      break;
    }
    default:
    // the first cube is delivered, other tasks to be done later
    // need a backward-direction movement, because normal rotation will fail here.
    {
      motorLeft -> run( RELEASE );
      motorRight -> run( RELEASE );
    }
  }

	delay( 10 );
}
