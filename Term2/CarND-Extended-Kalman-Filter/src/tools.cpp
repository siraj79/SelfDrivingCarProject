#include <iostream>
#include "tools.h"

using Eigen::VectorXd;
using Eigen::MatrixXd;
using std::vector;

Tools::Tools() {}

Tools::~Tools() {}

VectorXd Tools::CalculateRMSE(const vector<VectorXd> &estimations, const vector<VectorXd> &ground_truth) {
	VectorXd rmse(4);
	rmse << 0, 0, 0, 0;

	// check the validity of the following inputs:
	//  * the estimation vector size should not be zero
	//  * the estimation vector size should equal ground truth vector size
	if (estimations.size() != ground_truth.size() || estimations.size() == 0) {
		cout << "Invalid estimation or ground_truth data" << endl;
		return rmse;
	}

	//accumulate squared residuals
	for (unsigned int i = 0; i < estimations.size(); ++i) {

		VectorXd residual = estimations[i] - ground_truth[i];

		//coefficient-wise multiplication
		residual = residual.array()*residual.array();
		rmse += residual;
	}

	//calculate the mean
	rmse = rmse / estimations.size();

	//calculate the squared root
	rmse = rmse.array().sqrt();

	//return the result
	return rmse;
}

MatrixXd Tools::CalculateJacobian(const VectorXd& x_state) {
	MatrixXd Hj(3, 4);
	//recover state parameters
	double px = x_state(0);
	double py = x_state(1);
	double vx = x_state(2);
	double vy = x_state(3);

	//pre-compute a set of terms to avoid repeated calculation
	double c1 = px * px + py * py;
	double c2 = sqrt(c1);
	double c3 = (c1*c2);

	//check division by zero
	if (fabs(c1) < 0.0001) {
		cout << "CalculateJacobian () - Error - Division by Zero" << endl;
		return Hj;
	}

	//compute the Jacobian matrix
	Hj << (px / c2),(py / c2), 0, 0,
		 -(py / c1),(px / c1), 0, 0,
		  py*(vx*py - vy * px) / c3, px*(px*vy - py * vx) / c3, px / c2, py / c2;

	return Hj;
}

VectorXd Tools::Convert_CartesianToPolar(const VectorXd& x_state) {

	VectorXd polar(3);

	const double px = x_state(0);
	const double py = x_state(1);
	const double vx = x_state(2);
	const double vy = x_state(3);

	const double rho = sqrt(px * px + py * py);
	const double phi = atan2(py, px);
	const double rho_dot = (rho != 0) ? (px * vx + py * vy) / rho : 0;

	polar << rho, phi, rho_dot;
	return polar;
}

VectorXd Tools::Convert_PolarToCartesian(const VectorXd& x_state) {

	VectorXd cartesian(4);

	const double rho = x_state(0);
	const double phi = x_state(1);
	const double rho_dot = x_state(2);

	const double px = rho * cos(phi);
	const double py = rho * sin(phi);
	const double vx = rho_dot * cos(phi);
	const double vy = rho_dot * sin(phi);

	cartesian << px, py, vx, vy;
	return cartesian;
}
