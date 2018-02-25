#include "ukf.h"
#include "Eigen/Dense"
#include <iostream>

using namespace std;
using Eigen::MatrixXd;
using Eigen::VectorXd;
using std::vector;

//Ham
//asdf

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

  // Process noise standard deviation longitudinal acceleration in m/s^2 (tune these)
  std_a_ = 30;

  // Process noise standard deviation yaw acceleration in rad/s^2 (tune these)
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
  
  //set the state dimension:
  n_x_   = 5;
  
  //set the augmented state dimension:
  n_aug_ = 7;
  
  //Time in u_s (initialize to 0);
  time_us_ = 0;
 
  //Process noise standard deviation (m/s^2):
  std_a_ = 0.2;
  
  //Process noise standard deviation yaw acceleration (rad/s^2):
  std_yawdd_ = 0.2;
  
  //intitialize lambda (the spreading parameter):
  lambda_ = 3 -n_aug_;
  
  //Initiate the (state vector) x matrix (non-augmented): 
  VectorXd x = VectorXd(n_x_);
  x <<  0,
		0,
		0,
		0,
		0;
  
  //Initiate the (process co-variance) P matrix
  MatrixXd p = MatrixXd(n_x_, n_x_);
  
  
  //create augmented mean vector
  VectorXd x_aug = VectorXd(7);
  /*
  //Create the augmented mean state vector:
  VectorXd x_aug = VectorXd(n_aug_);
  x_aug.head(5) = x;
  x_aug(5) = 0;
  x_aug(6) = 0;
  */
 
  //create augmented state covariance
  MatrixXd P_aug = MatrixXd(7, 7);

  //create sigma point matrix
  MatrixXd Xsig_aug = MatrixXd(n_aug_, 2 * n_aug_ + 1);

  
  /***Verify if this is the right spot to initialize these or
  should it be done down in the section:
  void UKF::ProcessMeasurement(MeasurementPackage meas_package) */
 
  p.fill(0.0);
  //Check this one later and see if it works
  p.setIdentity(n_x_,n_x_);
  
  //Create the augmented mean state vector:
  VectorXd x_aug_ = VectorXd(n_aug_);
  x_aug_.head(5) = x;
  x_aug_(5) = 0;
  x_aug_(6) = 0;
  
  
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
