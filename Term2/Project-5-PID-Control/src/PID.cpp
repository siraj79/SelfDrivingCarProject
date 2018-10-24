#include "PID.h"
#include <iostream>
#include <math.h>       /* fabs */
#include "Eigen/Dense"
using Eigen::VectorXd;
using namespace std;





PID::PID() {}

PID::~PID() {}

void PID::Init(double Kp, double Ki, double Kd) {
	this->Kp = Kp; // Proportionnal term
	this->Ki = Ki; // Integral term
	this->Kd = Kd; // Differential term
	this->prev_total_error = 0;
}

void PID::UpdateError(double cte) {
	d_error = cte - p_error;
	p_error = cte;
	i_error += cte;
}

double PID::TotalError() {
	//PID Computations 
	double total_error = -Kp * p_error - Kd * d_error - Ki * i_error;
	this->prev_total_error = total_error; //kept for smoothing
	return total_error;
}

double PID::TotalError(bool applyLimit, double lowerLimit, double upperLimit, bool applySmoothing, double lagcut) {

	//PID Computations 
	double total_error = -Kp * p_error - Kd * d_error - Ki * i_error;

	// Enforce output limits
	if (applyLimit) {
		if (total_error > upperLimit) {
			total_error = upperLimit;
		}
		if (total_error < lowerLimit) {
			total_error = lowerLimit;
		}
	}

	//Smoothing: Single pole filter.lagcut parameter between 0.0 and 1.0 to configure the cutoff frequency.
	if (applySmoothing) {
		//lagcut value of 0.15 to 0.35 usually works well.
		//The lower this cutoff level, the better the high frequency noise rejection 
		//but the more likely that effectiveness of the derivative term is reduced.
		total_error = lagcut * total_error + (1 - lagcut)*this->prev_total_error;
	}

	this->prev_total_error = total_error; //kept for smoothing
	return total_error;
}








twiddle::twiddle() {}

twiddle::~twiddle() {}

void twiddle::Init(VectorXd Params) {

	this->Params = Params;
	Prev_Params = Params;

	dParams << 0.0,0.0,0.0,0.0,0.0;

	number_of_error_added = 0;
	error = 0; //siraj
	best_err = 0; //siraj

	increase = 1;
	current_paramToOptimize_num = 0;
}

VectorXd twiddle::Optimize_Parameter() {

	double err = error / float(number_of_error_added);

	if (increase == 1) {
		if (best_err >= err) {
			best_err = err;
			dParams[current_paramToOptimize_num] *= 1.1;
		}
		else {
			Params[current_paramToOptimize_num] -= 2 * dParams[current_paramToOptimize_num];
			increase = 0;
		}
	}
	if (increase == 0) {
		if (best_err >= err) {
			best_err = err;
			dParams[current_paramToOptimize_num] *= 1.1;
		}
		else {
			dParams *= 0.9;
			Params[current_paramToOptimize_num] += dParams[current_paramToOptimize_num];
			increase = 1;
		}
	}
		
	current_paramToOptimize_num++;
	if (current_paramToOptimize_num == Params.size()) {
		current_paramToOptimize_num = 0;
	}

	return Params;
}

void twiddle::Add_to_Total_Error(double cte) {
	error = (cte*cte) + (0.8 *error);
	number_of_error_added++;
}


