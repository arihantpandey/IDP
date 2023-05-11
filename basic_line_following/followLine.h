#ifndef _FOLLOW_LINE_H_
#define _FOLLOW_LINE_H_

/**About wheel control, the rotating center uses three words. Their meanings:
    middle: between the left-right sensors & two wheels same speed different direction;
    near right / left: beside the left-right sensors & two wheels different direction;
    far right / left: beside the wheels & one steady;
    super far right / left & two wheels same direction different speed.
**/

struct SWheels
{
    double leftV;
    double rightV;
};

static SWheels wheelSpeedGroup[10];
enum rotationCenter {
    fullyStop,
    straitForward, straitBackward,
    middle,
    nLeft, nRight,
    fLeft, fRight,
    sLeft, sRight
};

struct SSensors
/**True for light, False for dim
  *state: t m r l
  *standard state: 1100
**/
{
    bool left, right, middle, tail;
    SSensors(bool l, bool r, bool m, bool t)
    {
        left = l;
        right = r;
        middle = m;
        tail = t;
    }
    short state()
    {
        return ((((((short)tail << 1) + middle) << 1) + right) << 1) + left;
    }
};

/**Strait line without cross**/
SWheels simplestFollow(SSensors thisState, SSensors lastState);

#endif // _FOLLOW_LINE_H_
