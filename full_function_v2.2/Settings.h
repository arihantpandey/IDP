#define SLOWSPEED 140 // reduced speed, used when corrections are made
#define TURNDELAY 1500 // used in turnAround(), for this amount of time is the line checking turned off
#define TURNTIME 4000 // used in turnRightTime(), turnLeftTime(), the time it takes to make a 90 deg turns
#define INTERSECTIONIGNORE 300 //used in moveForward(), intersections are ignored for this much time after start
#define BACKWARDINTERSECTIONIGNORE 2000 // used in moveBackward(), intersections are ignored for this much time after start
#define RAMPTIME 5700 //used in cross_ramp(), the time it takes to climb the ramp from the previous intersection
#define CREEPFORWARDTIME 2500 // used in creepForward(), it moves forward for this time
//#define CREEPBACKWARDTIME 2000 //not used
//#define CREEPTIME 500
