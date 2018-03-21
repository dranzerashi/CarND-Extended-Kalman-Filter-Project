#include "FusionEKF.h"
#include "tools.h"
#include "Eigen/Dense"
#include <iostream>

using namespace std;
using Eigen::MatrixXd;
using Eigen::VectorXd;
using std::vector;

/*
 * Constructor.
 */
FusionEKF::FusionEKF() {
  is_initialized_ = false;

  previous_timestamp_ = 0;

  // initializing matrices
  R_laser_ = MatrixXd(2, 2);
  R_radar_ = MatrixXd(3, 3);
  H_laser_ = MatrixXd(2, 4);
  Hj_ = MatrixXd(3, 4);

  //measurement covariance matrix - laser
  R_laser_ << 0.0225, 0,
        0, 0.0225;

  //measurement covariance matrix - radar
  R_radar_ << 0.09, 0, 0,
        0, 0.0009, 0,
        0, 0, 0.09;

  /**
  TODO:
    * Finish initializing the FusionEKF.
    * Set the process and measurement noises
  */
  H_laser_ << 1,0,0,0,
              0,1,0,0;

}

/**
* Destructor.
*/
FusionEKF::~FusionEKF() {}

void FusionEKF::ProcessMeasurement(const MeasurementPackage &measurement_pack) {


  /*****************************************************************************
   *  Initialization
   ****************************************************************************/
  if (!is_initialized_) {
    /**
    TODO:
      * Initialize the state ekf_.x_ with the first measurement.
      * Create the covariance matrix.
      * Remember: you'll need to convert radar from polar to cartesian coordinates.
    */
    // first measurement
    cout << "EKF: " << endl;
    ekf_.x_ = VectorXd(4);
    ekf_.x_ << 1, 1, 1, 1;
    
    // Initialize 
    ekf_.P_ = MatrixXd(4, 4);
    ekf_.P_ << 1,0,0,0,
                0,1,0,0,
                0,0,1000,0,
                0,0,0,1000;
    
    
    // Initialize State transition Matrix
    ekf_.F_ = MatrixXd::Identity(4, 4);
    
    //Set currently read timestamp as the previous_timestamp
    previous_timestamp_ = measurement_pack.timestamp_;

    if (measurement_pack.sensor_type_ == MeasurementPackage::RADAR) {
      /**
      Convert radar from polar to cartesian coordinates and initialize state.
      */
      float rho = measurement_pack.raw_measurements_(0);
      float phi = measurement_pack.raw_measurements_(1);
      float radial_velocity = measurement_pack.raw_measurements_(2);
      
      float px = rho*cos(phi);
      float py = rho*sin(phi);
      float vx = radial_velocity*cos(phi);
      float vy = radial_velocity*sin(phi);
      
      ekf_.x_<< px, py, vx, vy;
      
      
    }
    else if (measurement_pack.sensor_type_ == MeasurementPackage::LASER) {
      /**
      Initialize state.
      */
      ekf_.x_(0) = measurement_pack.raw_measurements_(0);
      ekf_.x_(1) = measurement_pack.raw_measurements_(1);
      
    }

    // done initializing, no need to predict or update
    is_initialized_ = true;
    return;
  }

  /*****************************************************************************
   *  Prediction
   ****************************************************************************/

  /**
   TODO:
     * Update the state transition matrix F according to the new elapsed time.
      - Time is measured in seconds.
     * Update the process noise covariance matrix.
     * Use noise_ax = 9 and noise_ay = 9 for your Q matrix.
   */
  float noise_ax = 9;
  float noise_ay = 9;
  
  //Initialize delta t as previous timestamp - current timestamp (divide by 10**6 to convert to seconds)
  float del_t = (measurement_pack.timestamp_ - previous_timestamp_) / 1000000.0;
  
  float del_t_pow_2 = del_t*del_t;
  float del_t_pow_3 = del_t_pow_2*del_t;
  float del_t_pow_4 = del_t_pow_2*del_t_pow_2;

  
  ekf_.F_(0,2) = del_t;
  ekf_.F_(1,3) = del_t;
  
  ekf_.Q_ = MatrixXd(4,4);
  ekf_.Q_ << del_t_pow_4*noise_ax/4, 0, del_t_pow_3*noise_ax/2, 0,
              0, del_t_pow_4*noise_ay/4, 0, del_t_pow_3*noise_ay/2,
              del_t_pow_3*noise_ax/2, 0, del_t_pow_2*noise_ax, 0,
              0, del_t_pow_3*noise_ay/2, 0, del_t_pow_2*noise_ay;
    
    
  ekf_.Predict();

  /*****************************************************************************
   *  Update
   ****************************************************************************/

  /**
   TODO:
     * Use the sensor type to perform the update step.
     * Update the state and covariance matrices.
   */

  if (measurement_pack.sensor_type_ == MeasurementPackage::RADAR) {
    // Radar updates
    Tools t;
    ekf_.H_ = t.CalculateJacobian(measurement_pack.raw_measurements_);
    ekf_.R_ = R_radar_;
    ekf_.UpdateEKF(measurement_pack.raw_measurements_);
    
  } else {
    // Laser updates
    ekf_.H_ = H_laser_;
    ekf_.R_ = R_laser_;
    ekf_.Update(measurement_pack.raw_measurements_);
  }

  // print the output
  cout << "x_ = " << ekf_.x_ << endl;
  cout << "P_ = " << ekf_.P_ << endl;
}
