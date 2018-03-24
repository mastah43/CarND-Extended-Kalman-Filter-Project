#ifndef KALMAN_FILTER_H_
#define KALMAN_FILTER_H_
#include "Eigen/Dense"
#include "tools.h"

using Eigen::VectorXd;
using Eigen::MatrixXd;

class KalmanFilter {
public:

  // state vector
  VectorXd x_;

  // measurement matrix
  MatrixXd H_;

  // measurement covariance matrix
  MatrixXd R_;

  // state covariance matrix
  MatrixXd P_;

  /**
   * Constructor
   */
  KalmanFilter();

  /**
   * Destructor
   */
  virtual ~KalmanFilter();

  /**
   * Init Initializes Kalman filter
   * @param x_in Initial state
   */
  void Init(VectorXd &x_in);

  /**
   * Prediction Predicts the state and the state covariance
   * using the process model
   * @param dt Time between k and k+1 in s
   */
  void Predict(double dt);

  /**
   * Updates the state by using standard Kalman Filter equations
   * @param z The measurement at k+1
   */
  void Update(const VectorXd &z);

  /**
   * Updates the state by using Extended Kalman Filter equations
   * @param z The measurement at k+1
   */
  void UpdateEKF(const VectorXd &z);

private:
  Tools tools_;

  // state transition matrix
  MatrixXd F_;

  // process covariance matrix
  MatrixXd Q_;

  /**
   * Updates state of kalman filter using error between measured and predicted state
   */
  void Filter(const VectorXd &y);

};

#endif /* KALMAN_FILTER_H_ */
