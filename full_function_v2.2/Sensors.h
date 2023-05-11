#ifndef _SENSORS_H_
#define _SENSORS_H_
#include "Pinout.h"

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
	    digitalRead( SENSOR_LEFT ),
	    digitalRead( SENSOR_RIGHT ),
	    digitalRead( SENSOR_MIDDLE ),
	    digitalRead( SENSOR_REAR ) );

	return thisState;
}

bool getCurrentState( int sensor_num )
{
    return digitalRead( sensor_num );
}

void setLastState( SSensors *state )
{
	lineSensorLastStateL = ( *state ).left;
	lineSensorLastStateR = ( *state ).right;
	lineSensorLastStateF = ( *state ).middle;
	lineSensorLastStateB = ( *state ).tail;
}

#endif
