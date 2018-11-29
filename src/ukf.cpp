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
  
   
  int n_x = 5;
  
  int n_aug = 7;
  
  double lambda = 3 - n_aug;
  
  

  // Set Sigma Points
  VectorXd x_aug = VectorXd(7);
  
  MatrixXd P_aug = MatrixXd(7,7);
  
  MatrixXd Xsig_aug = MatrixXd(n_aug, 2*n_aug + 1);
  
  x_aug.head(5) = x;
  x_aug(5) = 0;
  x_aug(6) = 0;
  
  P_aug.fill(0.0);
  p_aug.topLeftCorner(5,5) = P;
  P_aug(5,5) = std_a_ * std_a_;
  P_aug(6,6) = std_yawdd_ * std_yawdd_;
  
  MatrixXd L = P_aug.llt().matrixL();
  
  Xsig_aug.col(0) = x_aug;
  for(int i = 0; i < n_aug; i++)
  {
    Xsig_aug.col(i+1)  = X_aug + sqrt(lambda + n_aug) * L.col(i);
    Xsig_aug.col(i+1+n_aug)  = X_aug - sqrt(lambda + n_aug) * L.col(i);
  }
  
  //Predict Sigma Points
  MatrixXd Xsig_pred = MatrixXd(n_x, 2 * n_aug + 1);

  for(int i = 0; i < 2 * n_aug + 1; i++)
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

    Xsig_pred(0,i) = px_p;
    Xsig_pred(1,i) = py_p;
    Xsig_pred(2,i) = v_p;
    Xsig_pred(3,i) = yaw_p;
    Xsig_pred(4,i) = yawd_p;
  }  
  

  // Predicted Mean and Covariance
  VectorXd weights = VectorXd(2*n_aug+1);
  
  double weight_0 = lambda/(lambda+n_aug);
  weight(0) = weight_0;
  for(int i = 1; i < 2*n_aug + 1; i++)
  {
    double weight = 0.5/(n_aug+lambda);
    weights(i) = weight;
  }
  
  x_.fill(0.0);
  for (int i = 0; i < 2 * n_aug + 1; i++)
  {
    x_ = x_ + weight(i) * Xsig_pred.col(i);
  }
  
  P_.fill(0.0);
  for (int i = 0; i < 2 * n_aug + 1; i++)
  {
    VectorXd x_diff = Xsig_pred.col(i) - x_;
    while(x_diff(3) > M_PI) x_diff(3) -= 2.*M_PI;
    while(x_diff(3) < M_PI) x_diff(3) += 2.*M_PI;

    P_ = P_ + weights(i) * x_diff * x_diff.transpose();
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
}
