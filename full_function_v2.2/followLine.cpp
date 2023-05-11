#include "followLine.h"

int advancedFollow( int thisState, int lastState, bool is_moving_forward, int grab_distance, int cmd )
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
	const int middle_out = 0b1000;

	/// 2 changes
	const int tOut_lIn = 0b0101;
	const int tOut_rIn = 0b0110;
	const int at_cross = 0b1111;
	const int mOut_lIn = 0b1001;
	const int mOut_rIn = 0b1010;
	const int mOut_tOut = 0b0000;

	/// 3 changes
	// impossible tail_correct = 0b1011;
	const int middle_correct = 0b0111; // at cross wrong bearing
	const int right_correct = 0b0001; // very bad
	const int left_correct = 0b0010; // very bad

	/// 4 changes
	const int all_wrong = 0b0011; // at cross wrong position

	static bool rotation_started = false;

	if ( cmd == GOFORWARD )
	{
		if ( thisState == lastState )
		{
			return unchanged;
		}

		if ( thisState == good )
			/// it's on the simple strait line
		{
			return straitForward;
		}

		if ( lastState == good )
		{
			if ( (thisState ^ lastState) == L )
			{
				/// bearing: almost correct; position: to the right
				return -sLeft;
			}
			else if ( (thisState ^ lastState) == R )
			{
				return sRight;
			}
			else if ( (thisState ^ lastState) == M )
			{
				/// this cannot happen, unless sensors are broken
				return UNEXPECTED_ERROR + lastState;
			}
			else if ( (thisState ^ lastState) == T )
			{
				/// bearing: not correct; position: almost correct
				/// move until L / R changes
				return GetTailIn;
			}
			else
			{
				/// more than one sensor changes
				return REDUNDANT_ERROR;
			}
		}
		else if ( lastState == tail_out )
		{
			if ( (thisState ^ lastState) == L )
			{
				/// bearing: to the right; position: a bit to the right
				return -fLeft;
			}
			else if ( (thisState ^ lastState) == R )
			{
				return fRight;
			}
			else if ( (thisState ^ lastState) == M )
			{
				/// cannot go all black with L / R sensors on both sides
				return GetTailIn + lastState;
			}
			else if ( (thisState ^ lastState) == T )
			{
				/// tail in automatically, good
				return straitForward;
			}
			else
			{
				return REDUNDANT_ERROR;
			}
		}
		else if ( lastState == middle_out )
		{
			if ( (thisState ^ lastState) == L )
			{
				/// bearing: to the right; position: far the right
				return -nLeft;
			}
			else if ( (thisState ^ lastState) == R )
			{
				/// symmetric
				return nRight;
			}
			else if ( (thisState ^ lastState) == M )
			{
				/// middle in again, good
				return straitForward;
			}
			else if ( (thisState ^ lastState) == T )
			{
			    /*
				if ( is_moving_forward )
				{
					/// fully loose track, go back
					return -straitForward;
				}
				else
				{
					/// probably tail and other three on both sides of the line
					return -straitForward;
				}*/
				return GetOneSensorIn;
			}
			else
			{
				return REDUNDANT_ERROR;
			}
		}
		else if ( lastState == left_in )
		{
			if ( (thisState ^ lastState) == L )
			{
				/// left out automatically, good
				return straitForward;
			}
			else if ( (thisState ^ lastState) == R )
			{
				/// must meet the cross
				/// bearing: to the right
				return -sLeft + AtCross;
			}
			else if ( (thisState ^ lastState) == M )
			{
				/// bearing: to the right; position: far to the right
				return -sRight;
			}
			else if ( (thisState ^ lastState) == T )
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
			if ( (thisState ^ lastState) == L )
			{
				return sRight + AtCross;
			}
			else if ( (thisState ^ lastState) == R )
			{
				return straitForward;
			}
			else if ( (thisState ^ lastState) == M )
			{
				return sLeft;
			}
			else if ( (thisState ^ lastState) == T )
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
			if ( (thisState ^ lastState) == L )
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
			else if ( (thisState ^ lastState) == R )
			{
				/// go into a cross and is not strait
				/// bearing: to the right; position: about correct
				return -middle + AtCross;
			}
			else if ( (thisState ^ lastState) == M )
			{
			    /*
				if ( is_moving_forward )
				{
					/// bearing: to the right; position: to the right
					return -fRight;
				}
				else
				{
					/// bearing: to the left; position: to the right
					return nRight;
				}*/
				return -nLeft;
			}
			else if ( (thisState ^ lastState) == T )
			{
				/// tail in automatically, good
				/// bearing: to the right; position: to the right
				return -sLeft;
			}
			else
			{
				return REDUNDANT_ERROR;
			}
		}
		else if ( lastState == tOut_rIn ) /// symmetric with above
		{
			if ( (thisState ^ lastState) == L )
			{
				return middle + AtCross;
			}
			else if ( (thisState ^ lastState) == R )
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
			else if ( (thisState ^ lastState) == M )
			{
			    /*
				if ( is_moving_forward )
				{
					return fLeft;
				}
				else
				{
					return -nLeft;
				}*/
				return nRight;
			}
			else if ( (thisState ^ lastState) == T )
			{
				return sRight;
			}
			else
			{
				return REDUNDANT_ERROR;
			}
		}
		else if ( lastState == at_cross )
		{
			if ( (thisState ^ lastState) == L )
			{
				/// bearing: to the right; position: unknown
				return -sLeft + LeaveCross;
			}
			else if ( (thisState ^ lastState) == R )
			{
				return sRight + LeaveCross;
			}
			else if ( (thisState ^ lastState) == M )
			{
				return UNEXPECTED_ERROR + lastState;
			}
			else if ( (thisState ^ lastState) == T )
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
			if ( (thisState ^ lastState) == L )
			{
				/// bearing: to the right; position: far to the right
				return -sRight;
			}
			else if ( (thisState ^ lastState) == R )
			{
				return UNEXPECTED_ERROR + lastState;
			}
			else if ( (thisState ^ lastState) == M )
			{
				/// middle enter, good
				/// bearing: to the right; position: a bit to the right
				return -nRight;
			}
			else if ( (thisState ^ lastState) == T )
			{
				/*
                if ( is_moving_forward )
                {
                    /// bearing: almost correct; position: to the right
                    return fLeft;
                }
                else
                {
                    /// bearing: far to the right; position: to the right
                    return -sRight;
                }*/
				return GetTailIn;
			}
			else
			{
				return REDUNDANT_ERROR;
			}
		}
		else if ( lastState == mOut_rIn ) /// symmetric with above
		{
			if ( (thisState ^ lastState) == L )
			{
				return UNEXPECTED_ERROR + lastState;
			}
			else if ( (thisState ^ lastState) == R )
			{
				return sLeft;
			}
			else if ( (thisState ^ lastState) == M )
			{
				return nLeft;
			}
			else if ( (thisState ^ lastState) == T )
			{
				/*
                if ( is_moving_forward )
                {
                    /// bearing: almost correct; position: to the right
                    return -fRight;
                }
                else
                {
                    /// bearing: far to the right; position: to the right
                    return sLeft;
                }*/
				return GetTailIn;
			}
		}
		else if ( lastState == mOut_tOut )
		{
			if ( (thisState ^ lastState) == L )
			{
				// unknown
				return GetTailIn;
			}
			else if ( (thisState ^ lastState) == R )
			{
				return GetTailIn;
			}
			else if ( (thisState ^ lastState) == M )
			{
				return GetTailIn;
			}
			else if ( (thisState ^ lastState) == T )
			{
				return -straitForward;
			}
		}
		else if ( lastState == middle_correct )
		{
			if ( (thisState ^ lastState) == L )
			{
				/// bearing: to the right; position: unknown
				return -nLeft + LeaveCross;
			}
			else if ( (thisState ^ lastState) == R )
			{
				return nRight + LeaveCross;
			}
			else if ( (thisState ^ lastState) == M )
			{
				return UNEXPECTED_ERROR + lastState;
			}
			else if ( (thisState ^ lastState) == T )
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
      if ( (thisState ^ lastState) == L )
      {
        /// entering cross
        /// bearing: to the left; position: far to the left
        return sLeft;
      }
      else if ( (thisState ^ lastState) == R )
      {
        /// all black, try to go back
        /*if ( is_moving_forward )
        {
          return -straitForward;
        }
        else
        {
          return straitForward;
        }*/
        return GetOneSensorIn;
      }
      else if ( (thisState ^ lastState) == M )
      {
        /// middle enters, good
        /// bearing: to the left; position: a bit to the left
        return sRight;
      }
      else if ( (thisState ^ lastState) == T )
      {
        /// bearing: to the left; position: to the left
        return fRight;
      }
      else
      {
        return REDUNDANT_ERROR;
      }
    }
    else if ( lastState == right_correct )
    {
      if ( (thisState ^ lastState) == L )
      {
        /*if ( is_moving_forward )
        {
          return -straitForward;
        }
        else
        {
          return straitForward;
        }*/
        return GetOneSensorIn;
      }
      else if ( (thisState ^ lastState) == R )
      {
        return -sRight;
      }
      else if ( (thisState ^ lastState) == M )
      {
        return -sLeft;
      }
      else if ( (thisState ^ lastState) == T )
      {
        return -fLeft;
      }
      else
      {
        return REDUNDANT_ERROR;
      }
    }
		else if ( lastState == all_wrong )
		{
			if ( (thisState ^ lastState) == L )
			{
				if ( is_moving_forward )
				{
					return UNEXPECTED_ERROR + lastState;
				}
				else
				{
					/// bearing: to the left; position: at cross but unknown
					return nRight;
				}
			}
			else if ( (thisState ^ lastState) == R )
			{
				if ( is_moving_forward )
				{
					return UNEXPECTED_ERROR + lastState;
				}
				else
				{
					return -nLeft;
				}
			}
			else if ( (thisState ^ lastState) == M )
			{
				/// at cross, details unknown
				return straitForward;
			}
			else if ( (thisState ^ lastState) == T )
			{
				return UNEXPECTED_ERROR + lastState;
			}
			else
			{
				return REDUNDANT_ERROR;
			}
		}
		else
		{
			return UNEXPECTED_ERROR + lastState;
		}
	}
	else if ( cmd == GRAB )
	{
		return fullyStop;
	}
	else if ( cmd == ROTATE_C || cmd == ROTATE_A )
	{
		if ( !rotation_started && ( thisState & T ) == 0 )
		{
			return ROTATE_CONFLICT_ERROR;
		}

		// should only enter this with T = true
		if ( rotation_started && ( thisState & T ) != 0 )
			// during rotation, tail find white line again
		{
			rotation_started = false;
			return RotateEnd;
		}

		if ( !rotation_started && ( thisState & T ) == 0 )
			// tail leaves white line, marking the true start of rotation
		{
			rotation_started = true;
		}

		return middle * ( cmd == ROTATE_C ? 1 : -1 );
	}
}
