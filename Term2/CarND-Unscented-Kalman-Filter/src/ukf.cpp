#include "ukf.h"
#include "Eigen/Dense"
#include <iostream>
#include "tools.h"

using namespace std;
using Eigen::MatrixXd;
using Eigen::VectorXd;
using std::vector;


struct SigmaPoint {
    double p_x;
    double p_y;
    double v;
    double yaw;
    double yawd;
    double nu_a;
    double nu_yawdd;
};


/**
 * Initializes Unscented Kalman filter
 * This is scaffolding, do not modify
 */
UKF::UKF() {

  //DO NOT MODIFY measurement noise values below these are provided by the sensor manufacturer.
  // Laser measurement noise standard deviation position1 in m
  std_laspx_ = 0.15;

  // Laser measurement noise standard deviation position2 in m
  std_laspy_ = 0.15;

  // Radar measurement noise standard deviation radius in m
  std_radr_ = 0.3;

  // Radar measurement noise standard deviation angle in rad
  std_radphi_ = 0.03;

  // Radar measurement noise standard deviation radius change in m/s
  std_radrd_ = 0.3;
  //DO NOT MODIFY measurement noise values above these are provided by the sensor manufacturer.





  // initially set to false, set to true in first call of ProcessMeasurement
  is_initialized_ = false;

  // Augmented state dimension
  n_aug_ = 7;
    
  // Sigma point spreading parameter
  lambda_ = 3 - n_aug_;
    
  // State dimension
  n_x_ = 5;

  // if this is false, laser measurements will be ignored (except during init)
  use_laser_ = true;

  // if this is false, radar measurements will be ignored (except during init)
  use_radar_ = true;

  // initial state vector
  x_ = VectorXd(n_x_);

  // initial covariance matrix
  P_ = MatrixXd(n_x_, n_x_);

  // Process noise standard deviation longitudinal acceleration in m/s^2
  std_a_ = .449;//30; //

  // Process noise standard deviation yaw acceleration in rad/s^2
  std_yawdd_ = .77;//30; //





  /**
  TODO:

  Complete the initialization. See ukf.h for other member properties.

  Hint: one or more values initialized above might be wildly off...
  */

  H_laser_ = Eigen::MatrixXd(2, n_x_);
  H_laser_.fill(0.);
  for (int i = 0; i < H_laser_.rows(); ++i) {
      H_laser_(i, i) = 1.;
  }
    
  R_laser_ = Eigen::MatrixXd(2, 2);
  R_laser_ <<
      pow(std_laspx_, 2), 0.,
      0.,                  pow(std_laspy_, 2);
    
    
  R_radar_ = MatrixXd(3, 3);
  R_radar_ <<
      pow(std_radr_, 2),  0,                      0,
      0,                  pow(std_radphi_, 2),    0,
      0,                  0,                      pow(std_radrd_, 2);


  //set vector for weights
  weights_ = VectorXd(2*n_aug_+1);
  weights_(0) = lambda_/(lambda_+n_aug_);
  for (int i=1; i<2*n_aug_+1; i++) {  
    weights_(i) = 0.5/(n_aug_+lambda_);
  }    
}

UKF::~UKF() {}




/**
 * @param {MeasurementPackage} meas_package The latest measurement data of
 * either radar or laser.
 */
void UKF::ProcessMeasurement(MeasurementPackage meas_package) {

  if (!is_initialized_) {
    
    x_.fill(0.0);
    P_ = MatrixXd::Identity(n_x_, n_x_);        
    
    if (meas_package.sensor_type_ == MeasurementPackage::LASER && use_laser_) {
        //Initialize state.
        x_(0) = meas_package.raw_measurements_[0];
        x_(1) = meas_package.raw_measurements_[1];
    } else if (meas_package.sensor_type_ == MeasurementPackage::RADAR && use_radar_) {
        
        //Convert radar from polar to cartesian coordinates and initialize state.
        const Eigen::VectorXd init_state = tools.Convert_PolarToCartesian(meas_package.raw_measurements_);            
        x_(0) = init_state(0);
        x_(1) = init_state(1);
    }
    



    prev_t_ = meas_package.timestamp_;      
    is_initialized_ = true;        
    return;
  }


  //compute the time elapsed between the current and previous measurements (Time is measured in seconds)
  const double dt = (meas_package.timestamp_ - prev_t_) / 1000000.0;  
  prev_t_ = meas_package.timestamp_;


  // Prediction
  Prediction(dt);

  // Measure_And_Update
  if (meas_package.sensor_type_ == MeasurementPackage::LASER && use_laser_) 
      Measure_And_Update_Lidar(meas_package);
  else if (meas_package.sensor_type_ == MeasurementPackage::RADAR && use_radar_) 
      Measure_And_Update_Radar(meas_package);  
}

/**
 * Predicts sigma points, the state, and the state covariance matrix.
 * @param {double} delta_t the change in time (in seconds) between the last
 * measurement and this one.
 */
void UKF::Prediction(double delta_t) {

  //Step 1 : Augmented sigma points
  MatrixXd Xsig_aug = AugSigmaPoints();
  
  //Step 2 : Sigma points predictions
  Predict_SigmaPoints(Xsig_aug, delta_t);
  
  //Step 3 : Predict mean and covariance matrix
  Predict_Mean_And_Covariance();
}

/**
 * Updates the state and the state covariance matrix using a laser measurement.
 * @param {MeasurementPackage} meas_package
 */
void UKF::Measure_And_Update_Lidar(MeasurementPackage meas_package) {

  // Measurement
  VectorXd y = meas_package.raw_measurements_ - H_laser_ * x_;

  // Calculate K Matrix
  MatrixXd Ht = H_laser_.transpose();
  MatrixXd PHt = P_ * Ht;
  MatrixXd S = H_laser_ * PHt + R_laser_;
  MatrixXd Si = S.inverse();
  MatrixXd K = PHt * Si;
  
  // New estimate
  x_ = x_ + (K * y);
  long x_size = x_.size();
  MatrixXd I = MatrixXd::Identity(x_size, x_size);
  P_ = (I - K * H_laser_) * P_;
  
  nis_laser_ = y.transpose() * Si * y; //siraj
}

/**
 * Updates the state and the state covariance matrix using a radar measurement.
 * @param {MeasurementPackage} meas_package
 */
void UKF::Measure_And_Update_Radar(MeasurementPackage meas_package) {

  int n_z = 3; //set measurement dimension, radar can measure r, phi, and r_dot
  
  MatrixXd Zsig = MatrixXd(n_z, 2 * n_aug_ + 1);  // Matrix for sigma points in measurement space
  VectorXd z_pred = VectorXd(n_z);                // Mean predicted measurement
  MatrixXd S = MatrixXd(n_z, n_z);                // Measurement covariance matrix S
  MatrixXd Tc = MatrixXd(n_x_, n_z);              // Matrix for cross correlation Tc  
  z_pred.fill(0.0);
  S.fill(0.0);
  Tc.fill(0.0);  

  // Measurement
  for (int i = 0; i < 2 * n_aug_ + 1; ++i) {      
      const Eigen::VectorXd sigma_point = Xsig_pred_.col(i); //siraj

      //transform sigma points into measurement space
      const Eigen::VectorXd polar_point = tools.Convert_CartesianToPolarCTRV(sigma_point);
      
      // measurement model
      Zsig(0,i) = polar_point(0); //rho
      Zsig(1,i) = polar_point(1); //phi
      Zsig(2,i) = polar_point(2); //r_dot
      
      //mean predicted measurement
      z_pred += weights_(i) * Zsig.col(i);
  }  

  //calculate cross correlation matrix (for update step)
  for (int i = 0; i < 2 * n_aug_ + 1; ++i) {
      
      VectorXd z_diff = Zsig.col(i) - z_pred;         //Residual
      while (z_diff(1)> M_PI) z_diff(1)-=2.*M_PI;     while (z_diff(1)<-M_PI) z_diff(1)+=2.*M_PI;  //angle normalization
      S += weights_(i) * z_diff * z_diff.transpose();
      
      VectorXd x_diff = Xsig_pred_.col(i) - x_;       //State difference          
      while (x_diff(3)> M_PI) x_diff(3)-=2.*M_PI;   while (x_diff(3)<-M_PI) x_diff(3)+=2.*M_PI;  //angle normalization
      Tc += weights_(i) * x_diff * z_diff.transpose();
  }  

  S += R_radar_;
  Eigen::MatrixXd Si = S.inverse();
  
  MatrixXd K = Tc * Si;     //Kalman gain K;                                      
  VectorXd z_diff = meas_package.raw_measurements_ - z_pred;  //Residual  
  z_diff(1) = fmod(z_diff(1), M_PI);  //angle normalizationb                       
  
  //update state mean and covariance matrix
  x_ += K * z_diff;
  P_ -= K * S * K.transpose();
  
  nis_radar_ = z_diff.transpose() * Si * z_diff; //siraj
}




void UKF::Predict_SigmaPoints(MatrixXd &Xsig_aug, double delta_t) {
  Xsig_pred_ = MatrixXd(n_x_, 2 * n_aug_ + 1);
  
  //predict sigma points
  for (int i = 0; i< 2*n_aug_+1; i++)
  {   
    //GetSigmaPoint
    SigmaPoint source_point = SigmaPoint();    
    source_point.p_x = Xsig_aug.col(i)(0);;
    source_point.p_y = Xsig_aug.col(i)(1);;
    source_point.v = Xsig_aug.col(i)(2);;
    source_point.yaw = Xsig_aug.col(i)(3);;
    source_point.yawd = Xsig_aug.col(i)(4);
    source_point.nu_a = Xsig_aug.col(i)(5);
    source_point.nu_yawdd = Xsig_aug.col(i)(6);

    //Noises
    const float v_noise = source_point.nu_a * delta_t;
    const float yaw_noise = 0.5 * source_point.nu_yawdd * pow(delta_t, 2);
    const float yawd_noise = source_point.nu_yawdd * delta_t;
    const float px_noise = 0.5 * source_point.nu_a * pow(delta_t, 2) * cos(source_point.yaw);
    const float py_noise = 0.5 * source_point.nu_a * pow(delta_t, 2) * cos(source_point.yaw);


    //predicted state 
    SigmaPoint predicted_point; 
  
    //Adding noise inline
    predicted_point.v = source_point.v + v_noise;
    predicted_point.yaw = source_point.yaw + source_point.yawd * delta_t  + yaw_noise;
    predicted_point.yawd = source_point.yawd + yawd_noise;
    
    //avoid exploding
    if (fabs(source_point.yawd) > 0.001) {
        
        predicted_point.p_x = source_point.p_x + source_point.v / source_point.yawd * (sin(source_point.yaw + source_point.yawd * delta_t) - sin(source_point.yaw));
        predicted_point.p_y = source_point.p_y + source_point.v / source_point.yawd * (cos(source_point.yaw) - cos(source_point.yaw + source_point.yawd * delta_t));
    }
    else {
        predicted_point.p_x = source_point.p_x + source_point.v * delta_t * cos(source_point.yaw);
        predicted_point.p_y = source_point.p_y + source_point.v * delta_t * sin(source_point.yaw);
    }
    
    //add noise
    predicted_point.p_x += px_noise;
    predicted_point.p_y += py_noise;

    //SetSigmaPoint
    Xsig_pred_(0, i) = predicted_point.p_x;
    Xsig_pred_(1, i) = predicted_point.p_y;
    Xsig_pred_(2, i) = predicted_point.v;
    Xsig_pred_(3, i) = predicted_point.yaw;
    Xsig_pred_(4, i) = predicted_point.yawd;
    
    if (Xsig_pred_.rows() > 5) {
        Xsig_pred_(5, i) = predicted_point.nu_a;
        Xsig_pred_(6, i) = predicted_point.nu_yawdd;
    }
  }
}

void UKF::Predict_Mean_And_Covariance() {
  //predicted state mean
  x_.fill(0.0);

  //iterate over sigma points
  for (int i = 0; i < 2 * n_aug_ + 1; i++) {  
      x_ += weights_(i) * Xsig_pred_.col(i);
  }
  
  //predicted state covariance matrix
  P_.fill(0.0);

  //iterate over sigma points
  for (int i = 0; i < 2 * n_aug_ + 1; i++) {
      
      // state difference
      VectorXd x_diff = Xsig_pred_.col(i) - x_;

      //angle normalization
      x_diff(3) = fmod(x_diff(3), M_PI);
      
      P_ += weights_(i) * x_diff * x_diff.transpose() ;
  }
}

MatrixXd UKF::AugSigmaPoints() {

  //create sigma point matrix
  MatrixXd Xsig_aug = MatrixXd(n_aug_, 2 * n_aug_ + 1);
  Xsig_aug.fill(0.0);


  //set state 
  VectorXd x_aug = VectorXd(n_aug_);
  x_aug.fill(0.0);
  x_aug.head(n_x_) = x_;  
  for (int i = n_x_; i< n_aug_; ++i) {
      x_aug(i) = 0;
  }

  
  //create augmented state covariance
  MatrixXd P_aug = MatrixXd(n_aug_, n_aug_);  
  P_aug.fill(0.0);
  P_aug.topLeftCorner(n_x_, n_x_) = P_;  
  P_aug(n_x_, n_x_) = pow(std_a_, 2);
  P_aug(n_x_+1, n_x_+1) = pow(std_yawdd_, 2);

  
  //calculate square root of P
  MatrixXd A = P_aug.llt().matrixL();  

  //set first column of sigma point matrix
  Xsig_aug.col(0) = x_aug;
  
  //set remaining sigma points
  for (int i = 1; i <= n_aug_; ++i)
  {
      Xsig_aug.col(i)        = x_aug + sqrt(lambda_ + n_aug_) * A.col(i-1);
      Xsig_aug.col(i+n_aug_) = x_aug - sqrt(lambda_ + n_aug_) * A.col(i-1);
  }
  return Xsig_aug;
}