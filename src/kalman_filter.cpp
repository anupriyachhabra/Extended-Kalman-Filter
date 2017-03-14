#include "kalman_filter.h"
#include <iostream>

using Eigen::MatrixXd;
using Eigen::VectorXd;
using namespace std;

KalmanFilter::KalmanFilter() {}

KalmanFilter::~KalmanFilter() {}

void KalmanFilter::Init(VectorXd &x_in, MatrixXd &P_in, MatrixXd &F_in,
                        MatrixXd &H_in, MatrixXd &R_in, MatrixXd &Q_in) {
  x_ = x_in;
  P_ = P_in;
  F_ = F_in;
  H_ = H_in;
  R_ = R_in;
  Q_ = Q_in;
}

void KalmanFilter::Predict() {
  x_ = F_ * x_;
  cout << "hello1" << endl;
  MatrixXd Ft = F_.transpose();
  cout << "hello2" << endl;
  P_ = F_ * P_ * Ft + Q_;
  cout << "hello3" << endl;
}

void KalmanFilter::Update(const VectorXd &z) {
  VectorXd z_pred = H_ * x_;
  cout << "hello4" << endl;
  VectorXd y = z - z_pred;
  cout << "hello5" << endl;
  UpdateCommon(y);
}

void KalmanFilter::UpdateEKF(const VectorXd &z) {
  cout << "ekf update" << endl;
  VectorXd z_pred = tools.CalculateMeasurement(x_);
  cout << "hello6" << endl;
  VectorXd y = z - z_pred;
  cout << "hello7" << endl;
  y(1) = tools.NormalizeAngle(y(1));
  UpdateCommon(y);
}

void KalmanFilter::UpdateCommon(Eigen::VectorXd &y) {
  MatrixXd Ht = H_.transpose();
  MatrixXd S = H_ * P_ * Ht + R_;
  cout << "hello8" << endl;
  MatrixXd Si = S.inverse();
  cout << "hello9" << endl;
  MatrixXd PHt = P_ * Ht;
  cout << "hello10" << endl;
  MatrixXd K = PHt * Si;

  //new estimate
  x_ = x_ + (K * y);
  long x_size = x_.size();
  MatrixXd I = MatrixXd::Identity(x_size, x_size);
  P_ = (I - K * H_) * P_;
}
