#include "kalman_filter.h"

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
// Kalman prediction function for both Kalman and Extended Kalman
    x_ = F_ * x_;
    MatrixXd Ft = F_.transpose();
    P_ = F_ * P_ * Ft + Q_;
}

void KalmanFilter::Update(const VectorXd &z) {
// Update the state using Kalman Filter equations (for lidar measurements)
    VectorXd y = z - H_ * x_;
    Estimate(y);
}

void KalmanFilter::UpdateEKF(const VectorXd &z) {
// Update the state using Kalman Filter equations (for radar measurements)
// Recalculate x object state to rho, theta, rho_dot coordinates
    VectorXd h = VectorXd(3); // h(x_)
    double rho = sqrt(x_(0) * x_(0) + x_(1) * x_(1));
    double new_rho = rho;
    if (new_rho < 0.0001) new_rho = 0.0001;
    double theta = atan2(x_(1),x_(0));
    double rho_dot = (x_(0) * x_(2) + x_(1) * x_(3)) / new_rho;
    
    h << rho, theta, rho_dot;
    VectorXd y = z - h;
    
    if(y(1) > M_PI){
        y(1) -= 2. * M_PI;
    } else if(y(1) < -M_PI){
        y(1) += 2. * M_PI;
    }
    
    Estimate(y);
}

void KalmanFilter::Estimate(const VectorXd &y){
// Matrix calculations
    MatrixXd Ht = H_.transpose();
    MatrixXd S = H_ * P_ * Ht + R_; // H_ contains the Jacobian (for RADAR) or H matrix (for LIDAR)
    MatrixXd Si = S.inverse();
    MatrixXd K =  P_ * Ht * Si;
// New estimates
    x_ = x_ + (K * y);
    long x_size = x_.size();
    MatrixXd I = MatrixXd::Identity(x_size, x_size);
    P_ = (I - K * H_) * P_;
}
