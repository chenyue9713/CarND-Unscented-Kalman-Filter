#include "ukf.h"
#include "Eigen/Dense"
#include <iostream>

using namespace std;
using Eigen::MatrixXd;
using Eigen::VectorXd;
using std::vector;

/**
 * Initializes Unscented Kalman filter
 * This is scaffolding, do not modify
 */
UKF::UKF() {
  // if this is false, laser measurements will be ignored (except during init)
  use_laser_ = true;

  // if this is false, radar measurements will be ignored (except during init)
  use_radar_ = true;

  // initial state vector
  x_ = VectorXd(5);

  // initial covariance matrix
  P_ = MatrixXd(5, 5);

  // Process noise standard deviation longitudinal acceleration in m/s^2
  std_a_ = 30;

  // Process noise standard deviation yaw acceleration in rad/s^2
  std_yawdd_ = 30;
  
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
  
  /**
  TODO:
  Complete the initialization. See ukf.h for other member properties.
  Hint: one or more values initialized above might be wildly off...
  */
  time_us_ = 0.f;
  
  is_initialized_ = False;
	
  n_x_ = 5;
  
  n_aug_ = 7;
  
  lambda_ = 3 - n_aug_;
  
  
  
}

UKF::~UKF() {}

/**
 * @param {MeasurementPackage} meas_package The latest measurement data of
 * either radar or laser.
 */
void UKF::ProcessMeasurement(MeasurementPackage meas_package) {
  /**
  TODO:
  Complete this function! Make sure you switch between lidar and radar
  measurements.
  */
    
  if(!is_initialized_)
  {
    // Maybe need to modify, try differernt setting
  	x_ << 0.0, 0.0, 2.0, 0.5, 0.3; 
  	// Initialization began from 1
  	P_ << 1, 0, 0, 0, 0,
  		0, 1, 0, 0, 0,
  		0, 0, 1, 0, 0,
  		0, 0, 0, 1, 0,
  		0, 0, 0, 0, 1;
    
    previous_timestamp_ = meas_package.timestamp_;
    
    if(meas_package.sensor_type_ == MeasurementPackage::RADAR)
    {
      float ro = meas_package.raw_measurements_[0];
      float theta = meas_package.raw_measurements_[1];
      float ro_dot = meas_package.raw_measurements_[2];
      float px = ro * cos(theta);
      float py = ro * sin(theta);
      
      x_[0] = px;
      x_[1] = py;
      x_[3] = theta;
      
        
    }
    else if(meas_package.sensor_type_ == MeasurementPackage::LASER)
    {
      float px = meas_package.raw_measurements_[0];
      float py = meas_package.raw_measurements_[1];
      x_[0] = px;
      x_[1] = py;
      
    }
    is_initialized_ = true;
    return;
    
  
  }
  float delta_t = (meas_package.timestamp_ - previous_timestamp_)/1000000.0;
  previous_timestamp_ = meas_package.timestamp_;
  
  Prediction(dt);

  if(meas_package.sensor_type_ = MeasurementPackage::LASER)
  {
	  UpdateLidar(meas_package);
  }
  else if(meas_package.sensor_type_ = MeasurementPackage::RADAR)
  {
	  UpdateRadar(meas_package);
  }


}


/**
 * Predicts sigma points, the state, and the state covariance matrix.
 * @param {double} delta_t the change in time (in seconds) between the last
 * measurement and this one.
 */
void UKF::Prediction(double delta_t) {
  /**
  TODO:
  Complete this function! Estimate the object's location. Modify the state
  vector, x_. Predict sigma points, the state, and the state covariance matrix.
  */  

  // Set Sigma Points
  VectorXd x_aug = VectorXd(n_aug_);
  
  MatrixXd P_aug = MatrixXd(n_aug_,n_aug_);
  
  MatrixXd Xsig_aug = MatrixXd(n_aug_, 2*n_aug_ + 1);
  
  x_aug.head(5) = x_;
  x_aug(5) = 0;
  x_aug(6) = 0;
  
  P_aug.fill(0.0);
  p_aug.topLeftCorner(5,5) = P_;
  P_aug(5,5) = std_a_ * std_a_;
  P_aug(6,6) = std_yawdd_ * std_yawdd_;
  
  MatrixXd L = P_aug.llt().matrixL();
  
  Xsig_aug.col(0) = x_aug;
  for(int i = 0; i < n_aug_; i++)
  {
    Xsig_aug.col(i+1)  = x_aug + sqrt(lambda_ + n_aug_) * L.col(i);
    Xsig_aug.col(i+1+n_aug_)  = x_aug - sqrt(lambda_ + n_aug_) * L.col(i);
  }
  
  //Predict Sigma Points
  Xsig_pred_ = MatrixXd(n_x_, 2 * n_aug_ + 1);

  for(int i = 0; i < 2 * n_aug_ + 1; i++)
  {
    double p_x      = Xsig_aug(0,i);
    double p_y      = Xsig_aug(1,i);
    double v        = Xsig_aug(2,i);
    double yaw      = Xsig_aug(3,i);
    double yawd     = Xsig_aug(4,i);
    double nu_a     = Xsig_aug(4,i);
    double nu_yawdd = Xsig_aug(4,i);

    double px_p, py_p;

    if(fabs(yawd) > 0.001)
    {
	px_p = p_x + v/yawd * (sin(yaw + yaw*delta_t) - sin(yaw));
	py_p = p_y + v/yawd * (cos(yaw) - cos(yaw + yawd*delta_t));
    }
    else
    { 
	px_p = p_x + v*delta_t*cos(yaw);
	py_p = p_y + v*delta_t*sin(yaw);
    }

    double v_p = v;
    double yaw_p = yaw + yawd*delta_t;
    double yawd_p = yawd;

    px_p = px_p + 0.5*nu_a**delta_t*delta_t*cos(yaw);
    py_p = py_p + 0.5*nu_a**delta_t*delta_t*sin(yaw);
    v_p = v_p + nu_a*delta_t;

    yaw_p = yaw_p + 0.5*nu_yawdd*delta_t*delta_t;
    yawd_p = yawd_p + nu_yawdd*delta_t;

    Xsig_pred_(0,i) = px_p;
    Xsig_pred_(1,i) = py_p;
    Xsig_pred_(2,i) = v_p;
    Xsig_pred_(3,i) = yaw_p;
    Xsig_pred_(4,i) = yawd_p;
  }  
  

  // Predicted Mean and Covariance
  weights_ = VectorXd(2*n_aug_+1);
  
  double weight_0 = lambda_/(lambda_+n_aug_);
  weights_(0) = weight_0;
  for(int i = 1; i < 2*n_aug_ + 1; i++)
  {
    double weight = 0.5/(n_aug_+lambda_);
    weights_(i) = weight;
  }
  
  x_.fill(0.0);
  for (int i = 0; i < 2 * n_aug_ + 1; i++)
  {
    x_ = x_ + weights_(i) * Xsig_pred_.col(i);
  }
  
  P_.fill(0.0);
  for (int i = 0; i < 2 * n_aug_ + 1; i++)
  {
    VectorXd x_diff = Xsig_pred_.col(i) - x_;
    while(x_diff(3) > M_PI) x_diff(3) -= 2.*M_PI;
    while(x_diff(3) < M_PI) x_diff(3) += 2.*M_PI;

    P_ = P_ + weights_(i) * x_diff * x_diff.transpose();
  }
  
}

/**
 * Updates the state and the state covariance matrix using a laser measurement.
 * @param {MeasurementPackage} meas_package
 */
void UKF::UpdateLidar(MeasurementPackage meas_package) {
  /**
  TODO:
  Complete this function! Use lidar data to update the belief about the object's
  position. Modify the state vector, x_, and covariance, P_.
  You'll also need to calculate the lidar NIS.
  */
  //Predicted Radar Measurement
  int n_z = 2;
  MatrixXd Zsig = MatrixXd(n_z,2*n_aug_ +  1);

  for(int i = 0; i < 2*n_aug_ + 1; i++)
  {
	  double p_x = Xsig_pred_(0,i);
	  double p_y = Xsig_pred_(1,i);

	  Zsig(0,i) = p_x;
	  Zsig(1,i) = p_y;
  }

  VectorXd z_pred = VectorXd(n_z);
  z_pred.fill(0.0);
  for(int i = 0; i < 2*n_aug_+1; i++)
  {
	  z_pred = z_pred + weights_(i)*Zsig.col(i);
  }
  MatrixXd S = MatrixXd(n_z,n_z);
  S.fill(0.0);
  for(int i = 0; i < 2*n_aug_+1; i++)
  {
	  VectorXd z_diff = Zsig.col(i) - z_pred;
	  
	  while(z_diff(1) > M_PI) z_diff(1) -= 2.*M_PI;
	  while(z_diff(1) < -M_PI) z_diff(1) += 2.*M_PI;
	  
	  S = S + weights_(i) * z_diff * z_diff.transpose();
  }
  
  
  MatrixXd R = MatrixXd(n_z,n_z);
  R = MatrixXd(n_z,n_z);
  R << std_laspx_*std_laspx_, 0,
       0, std_laspy_*std_laspy_;
  S = S + R;

  VectorXd z = VectorXd(n_z);
  z <<  meas_package.raw_measurements_[0], 
	meas_package.raw_measurements_[1];
  
  MatrixXd Tc = MatrixXd(n_x_, n_z);
  Tc.fill(0.0);
  for (int i = 0; i < 2 * n_aug_ + 1; i++) {  

    //residual
    VectorXd z_diff = Zsig.col(i) - z_pred;
    //angle normalization
    while (z_diff(1)> M_PI) z_diff(1)-=2.*M_PI;
    while (z_diff(1)<-M_PI) z_diff(1)+=2.*M_PI;

    // state difference
    VectorXd x_diff = Xsig_pred.col(i) - x_;
    //angle normalization
    while (x_diff(3)> M_PI) x_diff(3)-=2.*M_PI;
    while (x_diff(3)<-M_PI) x_diff(3)+=2.*M_PI;

    Tc = Tc + weights(i) * x_diff * z_diff.transpose();
  }
   
  MatrixXd K = Tc * S.inverse();
  
  VectorXd z_diff = z - z_pred;
	
  while (z_diff(1)> M_PI) z_diff(1)-=2.*M_PI;
  while (z_diff(1)<-M_PI) z_diff(1)+=2.*M_PI;
	
  x_ = x_ + K * z_diff;
  P_ = P_ - K*S*K.transpose();
  
  NIS_laser = z_diff.transpose() * S.inverse() * z_diff;


}

/**
 * Updates the state and the state covariance matrix using a radar measurement.
 * @param {MeasurementPackage} meas_package
 */
void UKF::UpdateRadar(MeasurementPackage meas_package) {
  /**
  TODO:
  Complete this function! Use radar data to update the belief about the object's
  position. Modify the state vector, x_, and covariance, P_.
  You'll also need to calculate the radar NIS.
  */
  int n_z = 3
  MatrixXd Zsig = MatrixXd(n_z, 2 * n_aug_ + 1);
	
  for(int i = 0 ; i < 2 * n_aug_ + 1; i++)
  {
	  double p_x = Xsig_pred_(0,i);
	  double p_y = Xsig_pred_(1,i);
	  double v = Xsig_pred_(2,i);
	  double yaw = Xsig_pred_(3,i);
	 
	  double v1 = cos(yaw)*v;
	  double v2 = sin(yaw)*v;
	  
	  Zsig(0,i) = sqrt(p_x*p_x + p_y*p_y);
	  Zsig(1,i) = atan2(p_y,p_x);
	  Zsig(2,i) = (p_x*v1 + p_y*v2) / sqrt(p_x*p_x + p_y*p_y);
  }
	
  VectorXd z_pred = VectorXd(n_z);
  z_pred.fill(0.0);
  for(int i = 0; i < 2*n_aug_+1; i++)
  {
	  z_pred = z_pred + weights_(i)*Zsig.col(i);
  }
  MatrixXd S = MatrixXd(n_z,n_z);
  S.fill(0.0);
  for(int i = 0; i < 2*n_aug_+1; i++)
  {
	  VectorXd z_diff = Zsig.col(i) - z_pred;
	  
	  while(z_diff(1) > M_PI) z_diff(1) -= 2.*M_PI;
	  while(z_diff(1) < -M_PI) z_diff(1) += 2.*M_PI;
	  
	  S = S + weights_(i) * z_diff * z_diff.transpose();
  }
  
  
  MatrixXd R = MatrixXd(n_z,n_z);
  R << std_radr_*std_radr_, 0, 0,
       0, std_radphi_*std_radphi_, 0,
       0, 0, std_radrd_*std_radrd_; 
	
  S = S + R;
	
  VectorXd z = VectorXd(n_z);
  z <<  meas_package.raw_measurements_[0], 
	meas_package.raw_measurements_[1], 
	meas_package.raw_measurements_[2];
  
  MatrixXd Tc = MatrixXd(n_x_, n_z);
  Tc.fill(0.0);
  for (int i = 0; i < 2 * n_aug_ + 1; i++) {  

    //residual
    VectorXd z_diff = Zsig.col(i) - z_pred;
    //angle normalization
    while (z_diff(1)> M_PI) z_diff(1)-=2.*M_PI;
    while (z_diff(1)<-M_PI) z_diff(1)+=2.*M_PI;

    // state difference
    VectorXd x_diff = Xsig_pred.col(i) - x_;
    //angle normalization
    while (x_diff(3)> M_PI) x_diff(3)-=2.*M_PI;
    while (x_diff(3)<-M_PI) x_diff(3)+=2.*M_PI;

    Tc = Tc + weights(i) * x_diff * z_diff.transpose();
  }
   
  MatrixXd K = Tc * S.inverse();
  
  VectorXd z_diff = z - z_pred;
	
  while (z_diff(1)> M_PI) z_diff(1)-=2.*M_PI;
  while (z_diff(1)<-M_PI) z_diff(1)+=2.*M_PI;
	
  x_ = x_ + K * z_diff;
  P_ = P_ - K*S*K.transpose();
  
  NIS_radar = z_diff.transpose() * S.inverse() * z_diff;
 
}

