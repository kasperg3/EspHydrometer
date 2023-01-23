#ifndef KALMANFILTER_H
#define KALMANFILTER_H

#pragma once    
#include "eigen3/Eigen/Dense"

class KalmanFilter
{
public:
    KalmanFilter(Eigen::VectorXd x, Eigen::MatrixXd A, Eigen::MatrixXd H,Eigen::MatrixXd Q,Eigen::MatrixXd R);
    ~KalmanFilter();
    void predict();
    void update(Eigen::Vector3d z);
    Eigen::VectorXd state();

private:

    Eigen::VectorXd x;
    Eigen::MatrixXd A;
    Eigen::MatrixXd H;
    Eigen::MatrixXd Q;
    Eigen::MatrixXd R;
    Eigen::MatrixXd P;
};

#endif