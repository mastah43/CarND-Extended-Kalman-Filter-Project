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

  //measurement covariance matrix - laser
  R_laser_ << 0.0225, 0,
              0, 0.0225;

  //measurement covariance matrix - radar
  R_radar_ << 0.09, 0, 0,
              0, 0.0009, 0,
              0, 0, 0.09;

  // laser measurement transformation
  H_laser_ << 1, 0, 0, 0,
              0, 1, 0, 0;
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
      * Initialize the state ekf_.x_ with the first measurement.
      * Create the covariance matrix.
      * convert radar from polar to cartesian coordinates.
    */

    VectorXd x_in(4);

    if (measurement_pack.sensor_type_ == MeasurementPackage::RADAR) {
      // Initialization with radar can be done for position and velocity.
      // Convert radar from polar to cartesian coordinates and initialize state.
      double range = measurement_pack.raw_measurements_(0);
      double angle    = measurement_pack.raw_measurements_(1);
      double rangeRate = measurement_pack.raw_measurements_(2);
      double x = range * cos(angle);
      double y = range * sin(angle);
      double vx = rangeRate * cos(angle);
      double vy = rangeRate * sin(angle);
      x_in << x, y, vx, vy;
    }
    else if (measurement_pack.sensor_type_ == MeasurementPackage::LASER) {
      // Initialization with lidar can only be done for position
      double x = measurement_pack.raw_measurements_(0);
      double y = measurement_pack.raw_measurements_(1);
      x_in << x, y, 0, 0;
    }

    if (fabs(x_in(0)) < 0.0001) {
      x_in(0) = 0.0001;
    }
    if (fabs(x_in(1)) < 0.0001) {
      x_in(1) = 0.0001;
    }

    ekf_.Init(x_in);

    previous_timestamp_ = measurement_pack.timestamp_;

    // done initializing, no need to predict or update
    is_initialized_ = true;
    return;
  }

  /*****************************************************************************
   *  Prediction
   ****************************************************************************/

  /**
     * Update the state transition matrix F according to the new elapsed time.
      - Time is measured in seconds.
     * Update the process noise covariance matrix.
     * Use noise_ax = 9 and noise_ay = 9 for your Q matrix.
   */
  //cout << "time:" << (measurement_pack.timestamp_/1000000.) << endl;
  double dt = (measurement_pack.timestamp_ - previous_timestamp_) / 1000000.;
  previous_timestamp_ = measurement_pack.timestamp_;


  ekf_.Predict(dt);

  /*****************************************************************************
   *  Update
   ****************************************************************************/

  /**
   * Use the sensor type to perform the update step.
   * Update the state and covariance matrices.
   */

  if (measurement_pack.sensor_type_ == MeasurementPackage::RADAR) {
    // Radar updates
    //cout << "radar measurement" << endl;
    ekf_.H_ = tools.CalculateJacobian(ekf_.x_);
    ekf_.R_ = R_radar_;
    ekf_.UpdateEKF(measurement_pack.raw_measurements_);
  } else {
    // Laser updates
    //cout << "lidar measurement" << endl;
    ekf_.H_ = H_laser_;
    ekf_.R_ = R_laser_;
    ekf_.Update(measurement_pack.raw_measurements_);
  }

  // print the output
  //cout << "x_ = " << ekf_.x_ << endl;
  //cout << "P_ = " << ekf_.P_ << endl;

}
