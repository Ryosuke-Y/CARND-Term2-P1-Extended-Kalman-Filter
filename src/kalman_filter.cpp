#include "kalman_filter.h"
#include <iostream>
#include "tools.h"
#include <math.h>
#include <stdio.h>

using Eigen::MatrixXd;
using Eigen::VectorXd;

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
  /**
  TODO:
    * predict the state
  */
  x_ = F_ * x_;
  MatrixXd Ft = F_.transpose();
  P_ = F_ * P_ * Ft + Q_;
}

void KalmanFilter::Update(const VectorXd &z) {
  /**
  TODO:
    * update the state by using Kalman Filter equations
  */
  VectorXd z_pred = H_ * x_;
  VectorXd y = z - z_pred;
  MatrixXd Ht = H_.transpose();
  MatrixXd S = H_ * P_ * Ht + R_;
  MatrixXd Si = S.inverse();
  MatrixXd PHt = P_ * Ht;
  MatrixXd K = PHt * Si;

  //new estimate
  x_ = x_ + (K * y);
  long x_size = x_.size();
  MatrixXd I = MatrixXd::Identity(x_size, x_size);
  P_ = (I - K * H_) * P_;
}

void KalmanFilter::UpdateEKF(const VectorXd &z) {
  /**
  TODO:
    * update the state by using Extended Kalman Filter equations
  */
  VectorXd h_x(3);
  //recover state parameters
  double px = x_(0);
  double py = x_(1);
  double vx = x_(2);
  double vy = x_(3);

  //pre-compute a set of terms to avoid repeated calculation
  double c1 = px*px+py*py;
  double c2 = sqrt(c1);

  //check division by zero
  if(fabs(c1) < 0.0001){
      std::cout << "UpdateEKF () - Error - Division by Zero" << std::endl;
      c1 = 0.000001; // very small value instead of zero!
      c2 = sqrt(c1);
  }

  //compute h(x') function
  h_x << c2,
          atan2(py,px),
          (px*vx + py*vy)/c2;

  VectorXd y = z - h_x;

  // Update the Jacobian Matrix Hj
  Tools tool;
  H_ = tool.CalculateJacobian(x_);


  // Make sure the phi value in the y vector is between -pi and pi
  // printf ( "remainder of %f / %f is %f\n", y[1],M_PI,fmod (y[1],M_PI) );
  y[1] = remainder (y[1],M_PI);

  MatrixXd Ht = H_.transpose();
  MatrixXd S = H_ * P_ * Ht + R_;
  MatrixXd Si = S.inverse();
  MatrixXd PHt = P_ * Ht;
  MatrixXd K = PHt * Si;

  //new estimate
  x_ = x_ + (K * y);
  long x_size = x_.size();
  MatrixXd I = MatrixXd::Identity(x_size, x_size);
  P_ = (I - K * H_) * P_;

}
