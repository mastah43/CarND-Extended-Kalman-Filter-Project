#include <iostream>
#include "kalman_filter.h"

using Eigen::MatrixXd;
using Eigen::VectorXd;
using namespace std;


// Please note that the Eigen library does not initialize 
// VectorXd or MatrixXd objects with zeros upon creation.

KalmanFilter::KalmanFilter() {}

KalmanFilter::~KalmanFilter() {}

void KalmanFilter::Init(VectorXd &x_in) {
  x_ = x_in;
  P_ = MatrixXd(4,4);
  P_ << 1, 0, 0, 0,
      0, 1, 0, 0,
      0, 0, 1000, 0,
      0, 0, 0, 1000;

  F_ = MatrixXd::Identity(4, 4);
  Q_ = MatrixXd(4,4);
}

void KalmanFilter::Predict(double dt) {
  double dt_2 = dt * dt;
  double dt_3 = dt_2 * dt;
  double dt_4 = dt_3 * dt;

  // Modify the F matrix so that the time is integrated
  F_(0, 2) = dt;
  F_(1, 3) = dt;

  // Update process noise covariance matrix using elapsed time
  double noise_ax = 9;
  double noise_ay = 9;
  Q_ <<  dt_4/4*noise_ax, 0, dt_3/2*noise_ax, 0,
          0, dt_4/4*noise_ay, 0, dt_3/2*noise_ay,
          dt_3/2*noise_ax, 0, dt_2*noise_ax, 0,
          0, dt_3/2*noise_ay, 0, dt_2*noise_ay;

  x_ = F_ * x_;
  MatrixXd Ft = F_.transpose();
  P_ = F_ * P_ * Ft + Q_;
}

void KalmanFilter::Filter(const VectorXd &y) {
  cout << "y: " << y << endl;
  cout << "H: " << H_ << endl;
  MatrixXd Ht = H_.transpose();
  cout << "Ht: " << Ht << endl;
  MatrixXd S = H_ * P_ * Ht + R_;
  cout << "S: " << S << endl;
  MatrixXd Si = S.inverse();
  cout << "Si: " << Si << endl;
  MatrixXd PHt = P_ * Ht;
  cout << "PHt: " << PHt << endl;
  MatrixXd K = PHt * Si;
  cout << "K: " << K << endl;

  //new estimate
  x_ = x_ + (K * y);
  long x_size = x_.size();
  MatrixXd I = MatrixXd::Identity(x_size, x_size);
  P_ = (I - K * H_) * P_;
}

void KalmanFilter::Update(const VectorXd &z) {
  VectorXd z_pred = H_ * x_;
  VectorXd y = z - z_pred;
  Filter(y);
}

void KalmanFilter::UpdateEKF(const VectorXd &z) {
  double px = x_(0);
  double py = x_(1);
  double vx = x_(2);
  double vy = x_(3);

  double range = sqrt(px*px + py*py);
  double angleRad = atan2(py, px);
  while (angleRad > M_PI) angleRad -= M_PI*2;
  while (angleRad < -M_PI) angleRad += M_PI*2;

  double rangeRate = (range < 0.0001) ? 0 : (px*vx + py*vy) / range;
  VectorXd h = VectorXd(3);
  h << range, angleRad, rangeRate;

  VectorXd y = z - h;

  Filter(y);
}
