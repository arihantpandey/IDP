Sposition disp(v1, v2, delta_t) {
/* returns the estimated displacment of the robot*/
/* distance between 2 wheels = 0.237m*/

	double diff, v_centre, d_centre;
	diff = (v1 * delta_t) - (v2 * delta_t);
	v_centre = (v1 + v2)/2;
	d_centre = v_centre * delta_t;

	Sposition res;
	res.angle = atan(diff/0.237);
	res.x = d_centre * sin(res.angle);
	res.y = d_centre * cos(res.angle);

	return res;
}


int sensor_check(pos_1, pos_2) { 
	/*check that the sensor position has been changed to cross onto the white line*/

	double t = 0.016;
	double a = t / 2;
	double test_1;
	double test_2;

	test_1 = (pos_1.x - a) * (pos_2.x - a);
	test_2 = (pos_1.x - a) * (pos_2.x - a);

	if (test_1 < 0){
		return 1;
	}
	else if (test_2 < 0){ 
		return 1;
		 }
	else{
		return 0;
	 }


int sensor_sim(displacement, L_pos, R_pos, M_pos, T_pos)
/*sensor position relative to centre of white line (or wheels at t =0) */
{
	Vector2D d(displacement.x, displacement.y);

	const int L = 1;
	const int R = 2;
	const int M = 4;
	const int T = 8;

	int res = 0;
	/*sensor check returns 0 or 1 value depending on whether white line crossed*/
	/* if L true = L, if L false = 0*/
	res += sensor_check(L_pos, L_pos + d) ? L : 0;
	res += sensor_check(R_pos, R_pos + d) ? R : 0;
	res += sensor_check(M_pos, M_pos + d) ? M : 0;
	res += sensor_check(T_pos, T_pos + d) ? T : 0;
	
	return res;
}

/*estimated displacement at time t*/
Sposition displacement = disp(v1, v2, delta_t);
Vector2D d(displacement.x, displacement.y);

/*position of sensors relative to wheel centre*/
vector2D L_pos(-0.015, 0.0);
vector2D R_pos(0.017, 0.0);
vector2D M_pos(0.0, 0.013);
vector2D T_pos(0.0, 0.198);

/*estimated sensor value at time t*/
int sensor_value = sensor_sim(displacement, L_pos + d, R_pos + d, M_pos_d, T_pos + d);

