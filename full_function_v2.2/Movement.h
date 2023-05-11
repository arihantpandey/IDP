#include "FollowLine.h"
#include "Sensors.h"
#include "Settings.h"

#define basicSpeedL 245
#define basicSpeedR 250
Adafruit_MotorShield AFMS = Adafruit_MotorShield();
Adafruit_DCMotor *motorLeft = AFMS.getMotor( 1 );
Adafruit_DCMotor *motorRight = AFMS.getMotor( 4 );

SWheels wheelSpeedGroup[11];

void move_with( double lv, double rv )
{
  if (lv < 0)
  {
    lv *= -1;
    motorLeft -> run(BACKWARD);
  }
  else
  {
    motorLeft -> run(FORWARD);
  }

  if (rv < 0)
  {
    rv *= -1;
    motorRight -> run(BACKWARD);
  }
  else
  {
    motorRight -> run(FORWARD);
  }
  
  motorLeft -> setSpeed( ( uint8_t )( basicSpeedL * lv ) );
  motorRight -> setSpeed( ( uint8_t )( basicSpeedR * rv ) );
}

void stop_movement()
{
    motorLeft -> run( RELEASE );
	motorRight -> run( RELEASE );
}

void rotate( bool clockwise, double relative_speed )
{
	motorLeft -> setSpeed( ( uint8_t )( basicSpeedL * relative_speed ) );
	motorRight -> setSpeed( ( uint8_t )( basicSpeedR * relative_speed ) );

	if ( clockwise )
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

void strait_move( bool forwards, double relative_speed )
{
	motorLeft -> setSpeed( ( uint8_t )( basicSpeedL * relative_speed ) );
	motorRight -> setSpeed( ( uint8_t )( basicSpeedR * relative_speed ) );

	if ( forwards )
	{
		motorLeft -> run( FORWARD );
		motorRight -> run( FORWARD );
	}
	else
	{
		motorLeft -> run( BACKWARD );
		motorRight -> run( BACKWARD );
	}
}

static bool to_init_rutol = false;
void rotate_until_tail_on_line( unsigned int basic_test_time, double relative_speed )
{
	Serial.println( "tail correcting working" );
	static bool clockwise;
	static unsigned int test_time;
	static unsigned int last_called;

	if ( to_init_rutol )
	{
		clockwise = true;
		test_time = basic_test_time;
		last_called = millis();
		to_init_rutol = false;
	}

	if ( millis() - last_called > test_time )
	{
		clockwise = !clockwise;
		test_time += basic_test_time;
   last_called = millis();
	}
	rotate( clockwise, relative_speed );
}

static bool to_init_muw = false;
void move_until_white( unsigned int basic_test_time, double relative_speed )
{
	Serial.println( "sensor correcting working" );
	static bool forwards;
	static unsigned int test_time;
	static unsigned int last_called;

	if ( to_init_muw )
	{
		forwards = true;
		test_time = basic_test_time;
		last_called = millis();
		to_init_muw = false;
	}

	if ( millis() - last_called > test_time )
	{
		forwards = !forwards;
		test_time += basic_test_time;
   last_called = millis();
	}
	strait_move( forwards, relative_speed );
}

int follow_command( int cmd )
{
	static int lineFollowInstruction = 0;
	static int wheelMovementNumber = 0;
	static int stateNumber = 0;
	static double meanVelocity = 0;
	static int continus_instructions = _nothing_;

	Serial.print( "\tcontinus instructions: " );
	Serial.println( continus_instructions );

	SSensors sensorThisState = getCurrentState();

	digitalWrite( 8, sensorThisState.left ? HIGH : LOW );
	digitalWrite( 9, sensorThisState.right ? HIGH : LOW );
	digitalWrite( 11, sensorThisState.middle ? HIGH : LOW );
	digitalWrite( 12, sensorThisState.tail ? HIGH : LOW );

	// routine check
	if ( sensorThisState.tail && continus_instructions == _get_tail_in_ )
	{
		continus_instructions = _nothing_; // task finished
		delay(10); // allow full in
		motorLeft -> run( RELEASE );
		motorRight -> run( RELEASE );
		meanVelocity = 0;
	}
	if ( sensorThisState.state() != 0 && continus_instructions == _get_sensor_in_ )
	{
		continus_instructions = _nothing_; // task finished
		delay(10); // allow full in
		motorLeft -> run( RELEASE );
		motorRight -> run( RELEASE );
		meanVelocity = 0;
	}


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
		Serial.println( stateNumber >> 8 );
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

	if ( stateNumber == UNEXPECTED_ERROR || stateNumber == ROTATE_CONFLICT_ERROR )
	{
		Serial.println( "ERROR" );
        strait_move(true, 0.9);

		meanVelocity = 0;
	}
	else if ( stateNumber == REDUNDANT_ERROR )
    {
        Serial.println( "Redundant" );
        strait_move(true, 0.9);
        meanVelocity = 0.9;
    }
	else if ( stateNumber == unchanged )
	{
		if ( continus_instructions == _nothing_ )
		{
			if ( sensorThisState.state() == 0b1100 && meanVelocity < 0.5 && meanVelocity > -0.5 )
				// restart, hoping problem solved
			{
				strait_move(true, 1);
				meanVelocity = 1;
			}
		}
		else if ( continus_instructions == _get_tail_in_ )
		{
			rotate_until_tail_on_line( 250, 0.6 );
		}
		else if ( continus_instructions == _get_sensor_in_ )
		{
			move_until_white( 250, 0.6 );
		}
	}
	else if ( stateNumber == GetTailIn )
	{
		if ( continus_instructions != _get_tail_in_ )
		{
			continus_instructions = _get_tail_in_;
			to_init_rutol = true;
		}
        rotate_until_tail_on_line( 250, 0.6 );
	}
	else if ( stateNumber == GetOneSensorIn )
	{
		if ( continus_instructions != _get_sensor_in_ )
		{
			continus_instructions = _get_sensor_in_;
			to_init_muw = true;
		}
        move_until_white( 250, 0.6 );
	}
	else if ( stateNumber == nothing || stateNumber == AtCross || stateNumber == LeaveCross )
		/// later on cross states will be separated from this
	{
		double leftV = wheelMovementNumber > 0 ? wheelSpeedGroup[wheelMovementNumber].leftV : -wheelSpeedGroup[-wheelMovementNumber].leftV;
		double rightV = wheelMovementNumber > 0 ? wheelSpeedGroup[wheelMovementNumber].rightV : -wheelSpeedGroup[-wheelMovementNumber].rightV;
		int delay_time = wheelMovementNumber > 0 ? wheelSpeedGroup[wheelMovementNumber].delay : wheelSpeedGroup[-wheelMovementNumber].delay;
		meanVelocity = ( leftV + rightV ) / 2;

		if ( leftV > 0 )
		{
			motorLeft -> setSpeed( ( uint8_t )( basicSpeedL * leftV * (meanVelocity > 0 ? 1 : 0.7) ) );
			motorLeft -> run( FORWARD );
		}
		else
		{
			motorLeft -> setSpeed( ( uint8_t )( -basicSpeedL * leftV * (meanVelocity > 0 ? 1 : 0.7) ) );
			motorLeft -> run( BACKWARD );
		}

		// right wheel is not reversed
		if ( rightV > 0 )
		{
			motorRight -> setSpeed( ( uint8_t )( basicSpeedR * rightV * (meanVelocity > 0 ? 1 : 0.7) ) );
			motorRight -> run( FORWARD );
		}
		else
		{
			motorRight -> setSpeed( ( uint8_t )( -basicSpeedR * rightV * (meanVelocity > 0 ? 1 : 0.7) ) );
			motorRight -> run( BACKWARD );
		}

		if (delay_time != 0)
            delay(delay_time);
	}
	else if ( stateNumber == RotateEnd )
	{
		motorLeft -> run( RELEASE );
		motorRight -> run( RELEASE );
		meanVelocity = 0;
	}
	return stateNumber;
}
