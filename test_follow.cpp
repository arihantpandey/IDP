#include<bits/stdc++.h>
using namespace std;

struct SWheels
{
	double leftV;
	double rightV;
} wheelSpeedGroup[9];

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


int advancedFollow( int thisState, int lastState, bool is_moving_forward, int grab_distance, string cmd )
{
	const int L = 1;
	const int R = 2;
	const int M = 4;
	const int T = 8;

	/// 0 change
	const int good = 0b1100;

	/// 1 change
	const int tail_out = 0b0100;
	const int left_in = 0b1101;
	const int right_in = 0b1110;
	// impossible middle_out = 0b1000;

	/// 2 changes
	const int tOut_lIn = 0b0101;
	const int tOut_rIn = 0b0110;
	const int at_cross = 0b1111;
	const int mOut_lIn = 0b1001;
	const int mOut_rIn = 0b1010;
	// impossible mOut_tOut = 0b0000;

	/// 3 changes
	// impossible tail_correct = 0b1011;
	const int middle_correct = 0b0111; // at cross wrong bearing
	const int left_correct = 0b0001; // very bad
	const int right_correct = 0b0010; // very bad

	/// 4 changes
	const int all_wrong = 0b0011; // at cross wrong position

	if ( thisState == lastState )
	{
		return unchanged;
	}

	if ( cmd == "GOFORWARD" )
	{
		if ( thisState == good )
			/// it's on the simple strait line
		{
			return straitForward;
		}

		if ( lastState == good )
		{
			if ( thisState ^ lastState == L )
			{
				/// bearing: almost correct; position: to the right
				return -sLeft;
			}
			else if ( thisState ^ lastState == R )
			{
				return sRight;
			}
			else if ( thisState ^ lastState == M )
			{
				/// this cannot happen, unless sensors are broken
				return UNEXPECTED_ERROR;
			}
			else if ( thisState ^ lastState == T )
			{
				/// bearing: not correct; position: almost correct
				/// move until L / R changes
				return straitForward;
			}
			else
			{
				/// more than one sensor changes
				return REDUNDANT_ERROR;
			}
		}
		else if ( lastState == tail_out )
		{
			if ( thisState ^ lastState == L )
			{
				/// bearing: to the right; position: a bit to the right
				return -fRight;
			}
			else if ( thisState ^ lastState == R )
			{
				return fLeft;
			}
			else if ( thisState ^ lastState == M )
			{
				/// cannot go all black with L / R sensors on both sides
				return UNEXPECTED_ERROR;
			}
			else if ( thisState ^ lastState == T )
			{
				/// tail in automatically, good
				return straitForward;
			}
			else
			{
				return REDUNDANT_ERROR;
			}
		}
		else if ( lastState == left_in )
		{
			if ( thisState ^ lastState == L )
			{
				/// left out automatically, good
				return straitForward;
			}
			else if ( thisState ^ lastState == R )
			{
				/// must meet the cross
				/// bearing: to the right
				return -sLeft + AtCross;
			}
			else if ( thisState ^ lastState == M )
			{
				/// bearing: to the right; position: far to the right
				return -sRight;
			}
			else if ( thisState ^ lastState == T )
			{
				/// move forward until middle out / left out / tail in
				return straitForward;
			}
			else
			{
				return REDUNDANT_ERROR;
			}
		}
		else if ( lastState == right_in ) /// symmetric with above
		{
			if ( thisState ^ lastState == L )
			{
				return sRight + AtCross;
			}
			else if ( thisState ^ lastState == R )
			{
				return straitForward;
			}
			else if ( thisState ^ lastState == M )
			{
				return sLeft;
			}
			else if ( thisState ^ lastState == T )
			{
				return straitForward;
			}
			else
			{
				return REDUNDANT_ERROR;
			}
		}
		else if ( lastState == tOut_lIn )
		{
			if ( thisState ^ lastState == L )
			{
				if ( is_moving_forward )
				{
					/// bearing: to the left; position: a bit to the right
					return nRight;
				}
				else
				{
					/// bearing: to the right; position: a bit to the right
					return -nRight;
				}
			}
			else if ( thisState ^ lastState == R )
			{
				/// go into a cross and is not strait
				/// bearing: to the right; position: about correct
				return -middle + AtCross;
			}
			else if ( thisState ^ lastState == M )
			{
				if ( is_moving_forward )
				{
					/// bearing: to the right; position: to the right
					return -fRight;
				}
				else
				{
					/// bearing: to the left; position: to the right
					return nRight;
				}
			}
			else if ( thisState ^ lastState == T )
			{
				/// tail in automatically, good
				/// bearing: to the right; position: to the right
				return -sRight;
			}
			else
			{
				return REDUNDANT_ERROR;
			}
		}
		else if ( lastState == tOut_rIn ) /// symmetric with above
		{
			if ( thisState ^ lastState == L )
			{
				return middle + AtCross;
			}
			else if ( thisState ^ lastState == R )
			{
				if ( is_moving_forward )
				{
					return -nLeft;
				}
				else
				{
					return nLeft;
				}
			}
			else if ( thisState ^ lastState == M )
			{
				if ( is_moving_forward )
				{
					return fLeft;
				}
				else
				{
					return -nLeft;
				}
			}
			else if ( thisState ^ lastState == T )
			{
				return sLeft;
			}
			else
			{
				return REDUNDANT_ERROR;
			}
		}
		else if ( lastState == at_cross )
		{
			if ( thisState ^ lastState == L )
			{
				/// bearing: to the right; position: unknown
				return -sLeft + LeaveCross;
			}
			else if ( thisState ^ lastState == R )
			{
				return sRight + LeaveCross;
			}
			else if ( thisState ^ lastState == M )
			{
				return UNEXPECTED_ERROR;
			}
			else if ( thisState ^ lastState == T )
			{
				/// bearing: unknown; position: unknown
				return straitForward;
			}
			else
			{
				return REDUNDANT_ERROR;
			}
		}
		else if ( lastState == mOut_lIn )
		{
			if ( thisState ^ lastState == L )
			{
				/// bearing: to the right; position: far to the right
				return -sRight;
			}
			else if ( thisState ^ lastState == R )
			{
				return UNEXPECTED_ERROR;
			}
			else if ( thisState ^ lastState == M )
			{
				/// middle enter, good
				/// bearing: to the right; position: a bit to the right
				return -nRight;
			}
			else if ( thisState ^ lastState == T )
			{
				if ( is_moving_forward )
				{
					/// bearing: almost correct; position: to the right
					return fLeft;
				}
				else
				{
					/// bearing: far to the right; position: to the right
					return -sRight;
				}
			}
			else
			{
				return REDUNDANT_ERROR;
			}
		}
		else if ( lastState == mOut_rIn ) /// symmetric with above
		{
			if ( thisState ^ lastState == L )
			{
				return UNEXPECTED_ERROR;
			}
			else if ( thisState ^ lastState == R )
			{
				return sLeft;
			}
			else if ( thisState ^ lastState == M )
			{
				return nLeft;
			}
			else if ( thisState ^ lastState == T )
			{
				if ( is_moving_forward )
				{
					/// bearing: almost correct; position: to the right
					return -fRight;
				}
				else
				{
					/// bearing: far to the right; position: to the right
					return sLeft;
				}
			}
		}
		else if ( lastState == middle_correct )
		{
			if ( thisState ^ lastState == L )
			{
				/// bearing: to the right; position: unknown
				return -nLeft + LeaveCross;
			}
			else if ( thisState ^ lastState == R )
			{
				return nRight + LeaveCross;
			}
			else if ( thisState ^ lastState == M )
			{
				return UNEXPECTED_ERROR;
			}
			else if ( thisState ^ lastState == T )
			{
				/// tail enters automatically, good
				return straitForward;
			}
			else
			{
				return REDUNDANT_ERROR;
			}
		}
		else if ( lastState == left_correct )
		{
			if ( thisState ^ lastState == L )
			{
				/// entering cross
				/// bearing: to the left; position: far to the left
				return sLeft;
			}
			else if ( thisState ^ lastState == R )
			{
				/// all black, try to go back
				if ( is_moving_forward )
				{
					return -straitForward;
				}
				else
				{
					return straitForward;
				}
			}
			else if ( thisState ^ lastState == M )
			{
				/// middle enters, good
				/// bearing: to the left; position: a bit to the left
				return nLeft;
			}
			else if ( thisState ^ lastState == T )
			{
				/// bearing: to the left; position: to the left
				return sLeft;
			}
			else
			{
				return REDUNDANT_ERROR;
			}
		}
		else if ( lastState == right_correct )
		{
			if ( thisState ^ lastState == L )
			{
				if ( is_moving_forward )
				{
					return -straitForward;
				}
				else
				{
					return straitForward;
				}
			}
			else if ( thisState ^ lastState == R )
			{
				return -sRight;
			}
			else if ( thisState ^ lastState == M )
			{
				return -nRight;
			}
			else if ( thisState ^ lastState == T )
			{
				return -sRight;
			}
			else
			{
				return REDUNDANT_ERROR;
			}
		}
		else if ( lastState == all_wrong )
		{
			if ( thisState ^ lastState == L )
			{
				if ( is_moving_forward )
				{
					return UNEXPECTED_ERROR;
				}
				else
				{
					/// bearing: to the left; position: at cross but unknown
					return nRight;
				}
			}
			else if ( thisState ^ lastState == R )
			{
				if ( is_moving_forward )
				{
					return UNEXPECTED_ERROR;
				}
				else
				{
					return -nLeft;
				}
			}
			else if ( thisState ^ lastState == M )
			{
				/// at cross, details unknown
				return straitForward;
			}
			else if ( thisState ^ lastState == T )
			{
				return UNEXPECTED_ERROR;
			}
			else
			{
				return REDUNDANT_ERROR;
			}
		}
		else
		{
			return UNEXPECTED_ERROR;
		}
	}
}

struct Sposition
{
	double x;
	double y;
	double angle;

	Sposition( double defX, double defY, double defA )
	{
		x = defX;
		y = defY;
		angle = defA;
	}

	Sposition operator + ( const Sposition& delta ) const
	{
		Sposition res( this->x, this->y, this->angle );
		res.x += delta.x;
		res.y += delta.y;
		res.angle += delta.angle;
		return res;
	}
};

class vector2D
{
public:
	double x;
	double y;

	vector2D( double defX, double defY )
	{
		x = defX;
		y = defY;
	}

	vector2D operator + ( const vector2D& b ) const
	{
		vector2D res( this->x, this->y );
		res.x += b.x;
		res.y += b.y;
		return res;
	}

	vector2D rotation(double theta) const
	{
	    vector2D res(0, 0);
        res.x = x * cos( theta ) + y * sin( theta );
        res.y = x * -sin( theta ) + y * cos( theta );
        return res;
	}
};

Sposition disp( double v1, double v2, double delta_t ) {
	/* returns the estimated displacement of the robot*/
	/* distance between 2 wheels = 0.237m*/
    const double left_wheel_mult = 1.00;
    const double right_wheel_mult = 1.10;
    v1 *= left_wheel_mult;
    v2 *= right_wheel_mult;


	double diff, v_centre, d_centre;
	diff = ( v1 * delta_t ) - ( v2 * delta_t );
	v_centre = ( v1 + v2 ) / 2;
	d_centre = v_centre * delta_t;

	Sposition res( d_centre * sin( res.angle ), d_centre * cos( res.angle ), atan( diff / 0.237 ) );
	return res;
}


int sensor_check( vector2D pos_1 ) {
	/*check that the sensor position has been changed to cross onto the white line*/

	double t = 0.016;
	double a = t / 2;

	if ( ( ( pos_1.x + a ) * ( pos_1.x - a ) ) < 0 ) {
		return 1;
	}
	else {
		return 0;
	}
}


int sensor_sim( vector2D L_i, vector2D R_i, vector2D M_i, vector2D T_i )
/*sensor position relative to centre of white line (or wheels at t =0) */
{
	int res = 0;
	/*sensor check returns 0 or 1 value depending on whether white line crossed*/
	res += 1 * sensor_check( L_i );
	res += 2 * sensor_check( R_i );
	res += 4 * sensor_check( M_i );
	res += 8 * sensor_check( T_i );
	//cout << "error " << res << bitset<sizeof(char)*4>(res) << endl;
	return res;
}

vector2D update_pos( vector2D pos, Sposition displacement ) {
	double theta = displacement.angle;
	//cout << "displacement ( " << displacement.x << " , " << displacement.y<< " )" << endl;
	double x_pos = pos.x * cos( theta ) + pos.y * sin( theta );
	double y_pos = pos.x * -sin( theta ) + pos.y * cos( theta );
	vector2D s( x_pos, y_pos );
	return s;
}


int main()
{
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

	const double delta_t = 0.001;
	/*position of sensors relative to wheel centre*/

	int sensorval_1 = 0b1100;
	int sensorval_2 = 0b1100;

	vector2D L_pos( -0.015, 0.0 );
	vector2D R_pos( 0.017, 0.0 );
	vector2D M_pos( 0.0, 0.013 );
	vector2D T_pos( 0.0, 0.198 );

	const vector2D Lc( -0.015, 0.0 );
	const vector2D Rc( 0.017, 0.0 );
	const vector2D Mc( 0.0, 0.013 );
	const vector2D Tc( 0.0, 0.198 );

	int v1 = 1;
	int v2 = 1;

	static double center_bearing = 0;
	vector2D center_position(0, 0);

	while ( true )
	{
		bool forwardcheck = false;
		if ( v1 + v2 > 0 ) {
			forwardcheck = true;
		}

		int v_and_ins = advancedFollow( sensorval_1, sensorval_2, forwardcheck, 0, "GOFORWARD" );
		int v_num = ( v_and_ins % 256 ) - ( ( v_and_ins % 256 > 127 ) ? 256 : 0 );
		int inssructions = v_and_ins - v_num;
		if(inssructions != unchanged)
        {
            v1 = wheelSpeedGroup[v_num].leftV;
            v2 = wheelSpeedGroup[v_num].rightV;
        }
		cout << "v_num: " << v_num << endl;
		cout << "wheel speed left  = " << v1 << endl;
		cout << "wheel speed right = " << v2 << endl;

		Sposition displacement = disp( v1, v2, delta_t );
		cout << "displacement ( " << displacement.x << " , " << displacement.y << " )" << endl;
		vector2D center_disp(displacement.x, displacement.y);
		center_position = center_position + center_disp;
		center_bearing += displacement.angle;
		vector2D L_pos = Lc.rotation(center_bearing) + center_position;
		vector2D R_pos = Rc.rotation(center_bearing) + center_position;
		vector2D M_pos = Mc.rotation(center_bearing) + center_position;
		vector2D T_pos = Tc.rotation(center_bearing) + center_position;

		//cout << "left sensor position: ( " << L_pos.x << " , " << L_pos.y<< " )" << endl;


		sensorval_1 = sensorval_2;
		sensorval_2 = sensor_sim( L_pos, R_pos, M_pos, T_pos );

		cout << "current sensor value:" << bitset<sizeof( char ) * 4>( sensorval_1 ) << endl;
		cout << "current sensor value:" << bitset<sizeof( char ) * 4>( sensorval_2 ) << endl;
		cout << "position of wheel centre: ( " << center_position.x << " , " << center_position.y << " )" << endl;
		cout << "\tleft sensor position: ( " << L_pos.x << " , " << L_pos.y << " )" << endl;
		cout << "\tright sensor position: ( " << R_pos.x << " , " << R_pos.y << " )" << endl;
		cout << "\tmiddle sensor position: ( " << M_pos.x << " , " << M_pos.y << " )" << endl;
		cout << "\ttail sensor position: ( " << T_pos.x << " , " << T_pos.y << " )" << endl;


		if (sensorval_1 != sensorval_2)
        {
            system( "pause" );
        }
	}
}
