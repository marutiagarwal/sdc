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

    1. initialize variables and matrices (x, F, H_laser, H_jacobian, P, etc.)
    2. initialize the Kalman filter position vector with the first sensor measurements
    3. modify the F and Q matrices prior to the prediction step based on the elapsed time between measurements
    4. call the update step for either the lidar or radar sensor measurement. Because the update step for lidar and 
       radar are slightly different, there are different functions for updating lidar and radar.

  you will see references to a variable called ekf_. The ekf_ variable is an instance of the KalmanFilter class. 
  You will use ekf_ to store your Kalman filter variables (x, P, F, H, R, Q) and call the predict and update functions.

 */
FusionEKF::FusionEKF() {
  is_initialized_ = false;

  previous_timestamp_ = 0;

  // initializing matrices
  R_laser_ = MatrixXd(2, 2);
  R_radar_ = MatrixXd(3, 3);
  H_laser_ = MatrixXd(2, 4);
  Hj_ = MatrixXd(3, 4);

  //measurement noise covariance matrix - laser
  R_laser_ << 0.0225, 0,
              0, 0.0225;

  //measurement noise covariance matrix - radar
  R_radar_ << 0.09, 0, 0,
              0, 0.0009, 0,
              0, 0, 0.09;

  /**
  TODO:
    * Finish initializing the FusionEKF.
    * Set the process and measurement noises
  */

  // measurement matrix
  H_laser_ << 1, 0, 0, 0,
              0, 1, 0, 0;

  //create a 4D state vector, we don't know yet the values of the x state
  ekf_.x_ = VectorXd(4);

  //state covariance matrix P
  ekf_.P_ = MatrixXd(4, 4);
  ekf_.P_ << 1, 0, 0, 0,
             0, 1, 0, 0,
             0, 0, 1000, 0,
             0, 0, 0, 1000;

  //the initial transition matrix F_
  ekf_.F_ = MatrixXd(4, 4);
  ekf_.F_ << 1, 0, 1, 0,
             0, 1, 0, 1,
             0, 0, 1, 0,
             0, 0, 0, 1;

  //define the process covariance matrix Q
  ekf_.Q_ = MatrixXd(4, 4);     
          
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
    // ekf_.x_ = VectorXd(4);
    ekf_.x_ << 1, 1, 1, 1;

    float px, py, vx, vy;

    if (measurement_pack.sensor_type_ == MeasurementPackage::RADAR) {
      /**
        Convert radar from polar to cartesian coordinates and initialize state.
      */
      cout << "init radar..." << endl;
      float rho = measurement_pack.raw_measurements_[0];
      float phi = measurement_pack.raw_measurements_[1];
      // float rho_dot = measurement_pack.raw_measurements_[2];
      px = rho * cos(phi);
      py = rho * sin(phi);  
      // vx = rho_dot * cos(phi);
      // vy = rho_dot * sin(phi);
    }
    else if (measurement_pack.sensor_type_ == MeasurementPackage::LASER) {
      /**
        Initialize state.        
      */
      cout << "init laser..." << endl;

      px = measurement_pack.raw_measurements_[0];
      py = measurement_pack.raw_measurements_[1];
    }

    // set the state with the initial location and zero initial velocity
    ekf_.x_ << px, py, 0, 0;
    previous_timestamp_ = measurement_pack.timestamp_;

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

  cout << "Start predicting" << endl;
  float dt = (measurement_pack.timestamp_ - previous_timestamp_) / 1000000.0; //dt - expressed in seconds
  cout << "dt: " << dt << endl;

  // update previous timestamp
  previous_timestamp_ = measurement_pack.timestamp_;

  // Modify the F matrix so that the time is integrated
  ekf_.F_(0, 2) = dt;
  ekf_.F_(1, 3) = dt; 
  cout << "updated F_: " << ekf_.F_ << endl;

  float dt_2 = dt * dt;
  float dt_3 = dt_2 * dt;
  float dt_4 = dt_3 * dt;
  
  // noise values
  const float noise_ax = 9;
  const float noise_ay = 9;
  
  // Update the process covariance matrix Q
  ekf_.Q_ <<  dt_4/4 * noise_ax, 0, dt_3/2 * noise_ax, 0,
              0, dt_4/4 * noise_ay, 0, dt_3/2 * noise_ay,
              dt_3/2 * noise_ax, 0, dt_2 * noise_ax, 0,
              0, dt_3/2 * noise_ay, 0, dt_2 * noise_ay;
  cout << "updated Q." << endl;

  // UPDATE F & Q for PREDICTION
  ekf_.Predict();
  cout << "Predicted" << endl;


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
    cout << "Radar updates..." << endl;
    
    // set H_ to Hj when updating with a radar measurement
    Hj_ = tools.CalculateJacobian(ekf_.x_);
    
    // don't update measurement if we can't compute the Jacobian
    if (Hj_.isZero(0)){
      cout << "Hj is zero" << endl;
      return;
    }
    
    ekf_.H_ = Hj_;
    ekf_.R_ = R_radar_;
    ekf_.UpdateEKF(measurement_pack.raw_measurements_);
  } else {
    // Laser updates
    cout << "Laser updates..." << endl;

    ekf_.H_ = H_laser_;
    ekf_.R_ = R_laser_;
    ekf_.Update(measurement_pack.raw_measurements_);          
  }

  // print the output
  cout << "x_ = " << ekf_.x_ << endl;
  cout << "P_ = " << ekf_.P_ << endl;
}





