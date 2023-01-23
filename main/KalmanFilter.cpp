#include "KalmanFilter.h"


KalmanFilter::KalmanFilter(Eigen::VectorXd x, Eigen::MatrixXd A, Eigen::MatrixXd H, Eigen::MatrixXd Q, Eigen::MatrixXd R) {
    this->x = x;
    this->A = A;
    this->H = H;
    this->Q = Q;
    this->R = R;
}

void KalmanFilter::predict() {
    x = A * x;
    P = A * P * A.transpose() + Q;
}

void KalmanFilter::update(Eigen::Vector3d z) {
    Eigen::VectorXd y = z - H * x;
    Eigen::MatrixXd S = H * P * H.transpose() + R;
    Eigen::MatrixXd K = P * H.transpose() * S.inverse();
    x = x + K * y;
    P = (Eigen::MatrixXd::Identity(P.rows(), P.cols()) - K * H) * P;
}

Eigen::VectorXd KalmanFilter::state() {
    return x;
}

