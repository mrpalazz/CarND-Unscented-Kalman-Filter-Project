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
  lambda_ = 3 - n_aug_;
  

  //Initiate the (process co-variance) P matrix
  MatrixXd P_ = MatrixXd(n_x_, n_x_);
  
  
  //create augmented mean vector
  //VectorXd x_aug = VectorXd(n_aug_);
  /*
  //Create the augmented mean state vector:
  VectorXd x_aug = VectorXd(n_aug_);
  x_aug.head(5) = x_;
  x_aug(5) = 0;
  x_aug(6) = 0;
  */
 
  //create augmented state covariance
  //MatrixXd P_aug = MatrixXd(n_aug_, n_aug_);

  //create sigma point matrix
  MatrixXd Xsig_aug = MatrixXd(n_aug_, 2 * n_aug_ + 1);

  
  /***Verify if this is the right spot to initialize these or
  should it be done down in the section:
  void UKF::ProcessMeasurement(MeasurementPackage meas_package) */
 
  P_.fill(0.0);
  //Check this one later and see if it works
  P_.setIdentity(n_x_,n_x_);
  

  
  
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
  

    
/*****************************************************************************
*  Initialization of Unscented Kalman Filter
****************************************************************************/
  if (!is_initialized_) {
  
 
    /**
    TODO:
    * Initialize the state x_ with the first measurement.
    * Create the covariance matrix.
    * Remember: you'll need to convert radar from polar to cartesian coordinates.
    */
    /**


    Initialize state.
    *///Initialize the values of the state vector x
    x_ << 0,0,0,0,0;

    //Initialize the co-variance matrix
    P_ << 0.5, 0.0, 0.0, 0.0, 0.0,
          0.0, 0.5, 0.0, 0.0, 0.0,
          0.0, 0.0, 0.5, 0.0, 0.0,
          0.0, 0.0, 0.0, 0.5, 0.0,
          0.0, 0.0, 0.0, 0.0, 0.5;


    //Initialize timestamp variable
    time_us_ = meas_package.timestamp_;


    //Check if the point is a LIDAR or RADAR measurement
    if (meas_package.sensor_type_ == MeasurementPackage::LASER && use_laser_) {
      /**
       Initialize state.
      */
      x_(0) = meas_package.raw_measurements_(0);
      x_(1) = meas_package.raw_measurements_(1);
    }

    else if (meas_package.sensor_type_ == MeasurementPackage::RADAR) {
      /**
       Convert radar from polar to cartesian coordinates and initialize state.
      */
      float ro     = meas_package.raw_measurements_(0);
      float phi    = meas_package.raw_measurements_(1);
      float ro_dot = meas_package.raw_measurements_(2);
      x_(0) = ro     * cos(phi);
      x_(1) = ro     * sin(phi);      
      //x_(2) = ro_dot * cos(phi);
      //x_(3) = ro_dot * sin(phi);
    }


    // done initializing, no need to predict or update
    is_initialized_ = true;

    return;
  
  }

/*****************************************************************************
*  Prediction Step of Unscented Kalman Filter
****************************************************************************/


  //compute the time elapsed (delta t) between the current and previous measurement
  float delta_t = (meas_package.timestamp_ - time_us_) / 1000000.0;	//dt - expressed in seconds
  time_us_ = meas_package.timestamp_;

  Prediction(delta_t);



/*****************************************************************************
*  Update the Lidar or Radar with the measurement package then proceed
****************************************************************************/
  if (meas_package.sensor_type_ == MeasurementPackage::LASER){
  	UpdateLidar(meas_package);
  }

  else if (meas_package.sensor_type_ == MeasurementPackage::RADAR){
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
  
/*****************************************************************************
*  Create the sigma points
****************************************************************************/

//create sigma point matrix
MatrixXd Xsig = MatrixXd( n_x_, 2 * n_x_+ 1 );

//calculate the square root of the P matrix
MatrixXd A = P_.llt().matrixL();


//Build up the new state dimension array
Xsig.col(0) = x_;

for (int i = 0; i < n_x_; i++) 
{
	Xsig.col(i+1)      = Xsig.col(0) + sqrt(lambda_ + n_x_)*A.col(i);
	Xsig.col(i+1+n_x_) = Xsig.col(0) - sqrt(lambda_ + n_x_)*A.col(i);
}
  
/*****************************************************************************
*  Create the  Augmented sigma points
****************************************************************************/

//populate the augmented mean state
VectorXd x_aug = VectorXd(n_aug_);


//create augmented state covariance
MatrixXd P_aug = MatrixXd(n_aug_, n_aug_);

//create augmented sigma point matrix
MatrixXd Xsig_aug = MatrixXd(n_aug_, 2 * n_aug_ + 1);

//create augmented mean state
x_aug.head(5) = x_;
x_aug(5) = 0;
x_aug(6) = 0;

//create augmented covariance matrix
P_aug.fill(0.0);
P_aug.topLeftCorner(n_x_,n_x_) = P_;
P_aug(5,5) = std_a_ * std_a_;
P_aug(6,6) = std_yawdd_ * std_yawdd_;

//create the square root matrix
MatrixXd L = P_aug.llt().matrixL();

//create the augmented sigma points
Xsig_aug.col(0) = x_aug;


for (int i = 0; i < n_aug_; i++){
    Xsig_aug.col(i+1)       = x_aug + sqrt(lambda_+n_aug_) * L.col(i);
    Xsig_aug.col(i+1+n_aug_) = x_aug - sqrt(lambda_+n_aug_) * L.col(i);
} 



//predict sigma points
for (int i = 0; i< 2*n_aug_ + 1; i++)
{
	//extract values for better readability
	double p_x      = Xsig_aug(0,i);
	double p_y      = Xsig_aug(1,i);
	double v        = Xsig_aug(2,i);
	double yaw      = Xsig_aug(3,i);
	double yawd     = Xsig_aug(4,i);
	double nu_a     = Xsig_aug(5,i);
	double nu_yawdd = Xsig_aug(6,i);


    //predicted state values
    double px_p, py_p;

    //avoid division by zero
if (fabs(yawd) > 0.001) {	
    px_p = p_x + v/yawd * ( sin (yaw + yawd*delta_t) - sin(yaw));
    py_p = p_y + v/yawd * ( cos(yaw) - cos(yaw+yawd*delta_t) );
    }

else {
    px_p = p_x + v*delta_t*cos(yaw);
    py_p = p_y + v*delta_t*sin(yaw);
	}

    double v_p = v;
    double yaw_p = yaw + yawd*delta_t;
    double yawd_p = yawd;

    //add noise
    px_p = px_p + 0.5*nu_a*delta_t*delta_t * cos(yaw);
    py_p = py_p + 0.5*nu_a*delta_t*delta_t * sin(yaw);
    v_p = v_p + nu_a*delta_t;

    yaw_p = yaw_p + 0.5 * nu_yawdd * delta_t * delta_t;
    yawd_p = yawd_p + nu_yawdd * delta_t;

    //write predicted sigma point into right column
    Xsig_pred_(0,i) = px_p;
    Xsig_pred_(1,i) = py_p;
    Xsig_pred_(2,i) = v_p;
    Xsig_pred_(3,i) = yaw_p;
    Xsig_pred_(4,i) = yawd_p;
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
