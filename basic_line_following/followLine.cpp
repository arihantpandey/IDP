#ifndef _FOLLOW_LINE_CPP_
#define _FOLLOW_LINE_CPP_
#include "followLine.h"

SWheels simplestFollow(SSensors thisState, SSensors lastState)
{
    if(lastState.state() == 0x1100 | lastState.state() == 0x1111)
    {
        if(thisState.state() == 0x1101)
        /// left wheel drove too much
        /// drive back around near right center
        {
            return wheelSpeedGroup[nRight];
        }
        else if(thisState.state() == 0x1110)
        {
            return wheelSpeedGroup[nLeft];
        }

        else
        {
            return wheelSpeedGroup[straitForward];
        }
    }
    else if(lastState.state() == 0x1101 | lastState.state() == 0x1110)
    {
        if(thisState.state() == 0x1100)
        {
            return wheelSpeedGroup[straitForward];
        }
        else if(thisState.state() == 0x1111)
        {
            return wheelSpeedGroup[straitForward];
        }
    }

    else
    {
        return wheelSpeedGroup[straitForward];
    }
}

#endif
