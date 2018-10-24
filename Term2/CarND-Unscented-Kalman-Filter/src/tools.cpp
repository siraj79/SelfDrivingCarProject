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

	//calculate mean
	rmse = rmse / estimations.size();

	//calculate squared root
	rmse = rmse.array().sqrt();

	return rmse;
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

VectorXd Tools::Convert_CartesianToPolarCTRV(const VectorXd& measurement) {
    
    VectorXd polar(3);
    
    const float px = measurement(0);
    const float py = measurement(1);
    const float v = measurement(2);
    const float yaw = measurement(3);
    
    const float v1 = cos(yaw)*v;
    const float v2 = sin(yaw)*v;
    
    const float rho = hypot(px, py);
    const float phi = atan2(py, px);
    const float rho_dot = (rho != 0) ? (px * v1 + py * v2) / rho : 0;
    
    polar << rho, phi, rho_dot;
    return polar;
}