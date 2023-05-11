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

/// All Clockwise (negative for reverse direction)
enum rotationCenter {
	fullyStop,
	straitForward,
	middle,
	nLeft, nRight,
	fLeft, fRight,
	sLeft, sRight
};

enum specialState {
	nothing = 0,
	unchanged = 1 << 8,
	AtCross = 2 << 8,
	LeaveCross = 3 << 8,
	UNEXPECTED_ERROR = 4 << 8,
	REDUNDANT_ERROR = 5 << 8,
	RotateEnd = 6 << 8,
	ROTATE_CONFLICT_ERROR = 7 << 8
};

enum commands {
	GOFORWARD,
	GRAB,
	ROTATE_C,
	ROTATE_A
};

struct SSensors
/**True for light, False for dim
  *state: t m r l
  *standard state: 1100
**/
{
	bool left, right, middle, tail;
	SSensors( bool l, bool r, bool m, bool t )
	{
		left = l;
		right = r;
		middle = m;
		tail = t;
	}
	short state()
	{
		return ( ( ( ( ( ( short )tail << 1 ) + middle ) << 1 ) + right ) << 1 ) + left;
	}
};

/**Strait line without cross**/
int simplestFollow( SSensors thisState, SSensors lastState );
int advancedFollow( int thisState, int lastState, bool is_moving_forward, int grab_distance, int cmd );

#endif // _FOLLOW_LINE_H_
