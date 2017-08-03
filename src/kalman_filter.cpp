#include "kalman_filter.h"
#include <iostream>
#include <math.h>

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
    // Calculate state transition function
    x_ = F_ * x_;
    
    // new postion
    MatrixXd Ft = F_.transpose();
    P_ = F_ * P_ * Ft + Q_;
}

void KalmanFilter::Update(const VectorXd &z) {
  /**
  TODO:
    * update the state by using Kalman Filter equations
  */
    // Predicted Measurement Vector
    VectorXd z_pred = H_ * x_;
    VectorXd y = z - z_pred;
    MatrixXd Ht = H_.transpose();
    MatrixXd S = H_ * P_ * Ht + R_;
    MatrixXd Si = S.inverse();
    MatrixXd PHt = P_ * Ht;
    MatrixXd K = PHt * Si;
    
    // New Estimate
    x_ = x_ + K * y;
    
    long x_size = x_.size();
    MatrixXd I = MatrixXd::Identity(x_size, x_size);
    P_ = (I - K * H_) * P_;
}

void KalmanFilter::UpdateEKF(const VectorXd &z) {
  /**
  TODO:
    * update the state by using Extended Kalman Filter equations
  */
    // predicted Measurment Vector
    float rho = sqrt(x_(0) * x_(0) + x_(1) * x_(1));
    float phi, rho_dot;
    
    // Checking for division by zero
    if (fabs(rho) < 0.0001)
    {
        std::cout << " Here we entered Division by Zero" << std::endl;
        rho_dot = 0;
        phi = 0;
    }
    else
    {
        phi = atan2(x_(1), x_(0));
        rho_dot = ((x_(0) * x_(2)) + (x_(1) * x_(3))) / rho;
    }
    
    
    VectorXd z_pred(3);
    z_pred << rho, phi, rho_dot;
    VectorXd y = z - z_pred;
    
    // nomarlising the angle as atan2 returns between -pi to +pi
    while(y(1) < -M_PI)
    {
        y(1) += 2.0 * M_PI;
    }
    
    while(y(1) > M_PI)
    {
        y(1) -= 2 * M_PI;
    }
    
    MatrixXd Ht = H_.transpose();
    MatrixXd S = H_ * P_ * Ht + R_;
    MatrixXd Si = S.inverse();
    MatrixXd PHtran = P_ * Ht;
    MatrixXd K = PHtran * Si;
    
    // New Estimate
    x_ = x_ + K * y;
    
    long x_size = x_.size();
    MatrixXd I = MatrixXd::Identity(x_size, x_size);
    P_ = (I - K * H_) * P_;
    
}


