#include "PID.h"
#include <numeric> // std::accumulate

using namespace std;

/*
	// P: steer in proportion to the crosstrack error

	// I: steer more when there is sustained error to counter the systematic bias we have from e.g. misaligned wheels.

	// D: When the car has turned enough to reduce CTE, it counter-steers to avoid overshooting

	// We can also use TWIDDLE algorith for finding good values for (Kp, Ki, Kd).
*/

PID::PID() {}

PID::~PID() {}

void PID::Init(double Kp, double Ki, double Kd) {
	this->Kp = Kp;
	this->Ki = Ki;
	this->Kd = Kd;

	max_steering_angle = M_PI/4.0;
	i_history_len = 100;

	p_error = 0.0;
	i_error = 0.0;
	d_error = 0.0;

	// private members
	previous_cte = 0.0;
	total_cte = 0.0;

	// keep running sum of N cte
	cte_vec.resize(i_history_len);
}


void PID::UpdateError(double cte) {
	if(cte_vec.size() <= i_history_len){
		cte_vec.push_back(cte);
	}
	else{
		cte_vec.pop_front();
	}

    total_cte += std::accumulate(cte_vec.begin(), cte_vec.end(), 0);

    p_error = - Kp * cte;
    i_error = - Ki * total_cte;
    d_error = - Kd * (cte - previous_cte);

    previous_cte = cte;	
}

double PID::AttenuateSteer(double steer_value){
	steer_value = fmin(steer_value, max_steering_angle);
	steer_value = fmax(steer_value, -1*max_steering_angle);
	return steer_value;
}

// return steer_value
double PID::TotalError() {
	if(cte_vec.size() >= i_history_len){
		return AttenuateSteer(p_error + i_error + d_error);
	}
	else{
		return AttenuateSteer(p_error + d_error);
	}
}

