#include "kalman_filter.h"

using namespace std;
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
  cout << "KalmanFilter::Update()" << endl;
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
  cout << "KalmanFilter::UpdateEKF()" << endl;
  
  toPolarCoordinates();

  // VectorXd z_pred = H_ * x_;
  VectorXd z_pred = hx_;

  VectorXd y = z - z_pred;

  // update angle
  y(1) = rectifyAngle(y(1));

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

void KalmanFilter::toPolarCoordinates() {
    float px = x_[0];
    float py = x_[1];
    float vx = x_[2];
    float vy = x_[3];

    float rho;
    float phi;
    float rhodot;

    if(fabs(px) < 0.0001 or fabs(py) < 0.0001){
      if(fabs(px) < 0.0001){
        px = 0.0001;
      }

      if(fabs(py) < 0.0001){
        py = 0.0001;
      }
      
      rho = sqrt(px*px + py*py);
      if (rho < 0.0001){
        rho = 0.0001;
      }
      phi = 0;
      rhodot = 0;      
      
    }
    else {
      rho = sqrt(px*px + py*py);
      phi = atan2(py,px); //  arc tangent of y/x, in the interval [-pi,+pi] radians.
      rhodot = (px*vx + py*vy) /rho;
    }
    // record the updated measurements to be used for Update step in Kalman filter
    hx_ = VectorXd(3);
    hx_ << rho, phi, rhodot;
}

float KalmanFilter::rectifyAngle(const float phi) {
  /**
    keep the angle phi in the range [-pi,pi]
  **/
  float phi_modified = phi;
  bool in_pi = false;

  while (in_pi == false) {
    if (phi_modified > 3.14159) {
      phi_modified -=  6.2831;
    }
    else if (phi_modified < -3.14159) {
      phi_modified += 6.2831;
    } else {
      in_pi = true;
    }
  }
  return phi_modified;
}



