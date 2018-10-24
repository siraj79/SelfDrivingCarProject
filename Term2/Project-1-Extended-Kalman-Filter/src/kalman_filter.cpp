#include "kalman_filter.h"

using Eigen::MatrixXd;
using Eigen::VectorXd;


KalmanFilter::KalmanFilter() {
	#pragma region initializing matrices
	R_laser_ = MatrixXd(2, 2);//measurement covariance matrix - laser
	R_laser_ <<
		0.0225, 0,
		0, 0.0225;

	R_radar_ = MatrixXd(3, 3);//measurement covariance matrix - radar
	R_radar_ <<
		0.09, 0, 0,
		0, 0.0009, 0,
		0, 0, 0.09;

	H_laser_ = MatrixXd(2, 4);
	H_laser_ <<
		1, 0, 0, 0,
		0, 1, 0, 0;

	H_radar_ = MatrixXd(3, 4);
	H_radar_ <<
		1, 0, 0, 0,
		0, 1, 0, 0,
		0, 0, 1, 0;

	F_ = Eigen::MatrixXd(4, 4);
	F_ <<
		1, 0, 1, 0,
		0, 1, 0, 1,
		0, 0, 1, 0,
		0, 0, 0, 1;

	P_ = Eigen::MatrixXd(4, 4);
	P_ <<
		1, 0, 0, 0,
		0, 1, 0, 0,
		0, 0, 1000, 0,
		0, 0, 0, 1000;

	Q_ = Eigen::MatrixXd(4, 4);
	Q_ <<
		0, 0, 0, 0,
		0, 0, 0, 0,
		0, 0, 0, 0,
		0, 0, 0, 0;
	#pragma endregion
}

KalmanFilter::~KalmanFilter() {}

void KalmanFilter::Init(VectorXd &x_in, MatrixXd &P_in, MatrixXd &F_in, MatrixXd &H_in, MatrixXd &R_in, MatrixXd &Q_in) {
  x_ = x_in;
  P_ = P_in;
  F_ = F_in;

  H_ = H_in;
  R_ = R_in;
  Q_ = Q_in;
}

void KalmanFilter::Predict(double dt, float noise_ax, float noise_ay) {

	// Prediction Predicts the state and the state covariance
	// using the process model
	// @param delta_T Time between k and k+1 in s


	// Update the state transition matrix F according to the new elapsed time.
	// Time is measured in seconds.
	// Update the process noise covariance matrix.
	// noise_ax = 9 and noise_ay = 9 for Q matrix.


	//1. Updating the F matrix
	F_(0, 2) = dt;
	F_(1, 3) = dt;


	//2. Updating the Q matrix
	noise_ax = (noise_ax != 0) ? noise_ax : 1;// avoiding zero
	noise_ay = (noise_ay != 0) ? noise_ay : 1;// avoiding zero
	const double r0c0 = pow(dt, 4) / 4 * noise_ax;
	const double r0c1 = 0;
	const double r0c2 = pow(dt, 3) / 2 * noise_ax;
	const double r0c3 = 0;
	const double r1c0 = 0;
	const double r1c1 = pow(dt, 4) / 4 * noise_ay;
	const double r1c2 = 0;
	const double r1c3 = pow(dt, 3) / 2 * noise_ay;
	const double r2c0 = r0c2;
	const double r2c1 = 0;
	const double r2c2 = pow(dt, 2) * noise_ax;
	const double r2c3 = 0;
	const double r3c0 = 0;
	const double r3c1 = r1c3;
	const double r3c2 = 0;
	const double r3c3 = pow(dt, 2) * noise_ay;

	Q_ <<
		r0c0, r0c1, r0c2, r0c3,
		r1c0, r1c1, r1c2, r1c3,
		r2c0, r2c1, r2c2, r2c3,
		r3c0, r3c1, r3c2, r3c3;


    //3. predict the state
	x_ = F_ * x_;
	MatrixXd Ft = F_.transpose();
	P_ = F_ * P_ * Ft + Q_;
}

void KalmanFilter::Update(const MeasurementPackage& package) {

	Eigen::VectorXd y;
	Eigen::VectorXd z = package.raw_measurements_;


	if (package.sensor_type_ == MeasurementPackage::RADAR) {
		Eigen::VectorXd cartesianMeas = tools_.Convert_PolarToCartesian(package.raw_measurements_);
		H_ = tools_.CalculateJacobian(cartesianMeas);
		R_ = R_radar_;

		VectorXd z_pred = tools_.Convert_CartesianToPolar(x_);
		y = z - z_pred;
		y(1) = fmod(y(1), M_PI);//Normalizing 

		//Took John Youn advice for Normalization formula
	    //https://carnd.slack.com/files/U3C0DLVPU/F5R5S9XEU/pasted_image_at_2017_06_10_02_39_pm.png

		//const double PI2 = 2 * M_PI;

		//// normalize the angle between -pi to pi
		//while (y(1) > M_PI) {
		//	y(1) -= PI2;
		//}

		//while (y(1) < -M_PI) {
		//	y(1) += PI2;
		//}
	}
	if (package.sensor_type_ == MeasurementPackage::LASER) {
		H_ = H_laser_;
		R_ = R_laser_;
		VectorXd z_pred = H_ * x_;
		y = z - z_pred;
	}


	// Calculate K Matrix
	MatrixXd Ht = H_.transpose();
	MatrixXd S = H_ * P_ * Ht + R_;
	MatrixXd Si = S.inverse();
	MatrixXd PHt = P_ * Ht;
	MatrixXd K = PHt * Si;

	// New estimate
	x_ = x_ + (K * y);
	long x_size = x_.size();
	MatrixXd I = MatrixXd::Identity(x_size, x_size);
	P_ = (I - K * H_) * P_;
}
