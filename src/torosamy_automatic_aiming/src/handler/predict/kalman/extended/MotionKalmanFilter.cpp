//
// Created by torosamy on 25-3-30.
//
#include "torosamy_automatic_aiming/handler/predict/kalman/extended/MotionKalmanFilter.h"

MotionKalmanFilter::MotionKalmanFilter(const YAML::Node& fileReader) :
    x_pri(9),
    x_post(9),
    rYaw(fileReader["rYaw"].as<double>()),
    rX(fileReader["rX"].as<double>()),
    I(Eigen::MatrixXd::Identity(9, 9)){
    Eigen::DiagonalMatrix<double, 9> p0;
    p0.setIdentity();
    P_post = p0;
}

Eigen::VectorXd MotionKalmanFilter::process(const double& time) const{
    Eigen::VectorXd x_new = x_post;
    //std::cout<<"dt_"<<dt_<<std::endl;
    x_new(0) += x_post(1) * time;
    x_new(2) += x_post(3) * time;
    x_new(4) += x_post(5) * time;
    x_new(6) += x_post(7) * time;
    return x_new;
}

Eigen::VectorXd MotionKalmanFilter::observe() const {
    Eigen::VectorXd z(4);
    z(0) = x_pri(0) - x_pri(8) * cos(x_pri(6));  // xa
    z(1) = x_pri(2) - x_pri(8) * sin(x_pri(6));  // ya
    z(2) = x_pri(4);               // za
    z(3) = x_pri(6);               // yaw
    return z;
}
Eigen::MatrixXd MotionKalmanFilter::processJacobian(const double& time) const {
    Eigen::MatrixXd f(9, 9);
    f <<  1,   time, 0,   0,   0,   0,   0,   0,   0,
        0,   1,   0,   0,   0,   0,   0,   0,   0,
        0,   0,   1,   time, 0,   0,   0,   0,   0,
        0,   0,   0,   1,   0,   0,   0,   0,   0,
        0,   0,   0,   0,   1,   time, 0,   0,   0,
        0,   0,   0,   0,   0,   1,   0,   0,   0,
        0,   0,   0,   0,   0,   0,   1,   time, 0,
        0,   0,   0,   0,   0,   0,   0,   1,   0,
        0,   0,   0,   0,   0,   0,   0,   0,   1;
    return f;
}


Eigen::MatrixXd MotionKalmanFilter::observeJacobian() const {
    Eigen::MatrixXd h(4, 9);
    const double yaw = x_pri(6);
    const double r = x_pri(8);
    //    xc   v_xc yc   v_yc za   v_za yaw          v_yaw  r
    h <<  1,   0,   0,   0,   0,   0,   r*sin(yaw),  0,     -cos(yaw),
          0,   0,   1,   0,   0,   0,   -r*cos(yaw), 0,     -sin(yaw),
          0,   0,   0,   0,   1,   0,   0,           0,     0,
          0,   0,   0,   0,   0,   0,   1,           0,     0;
    return h;
}

Eigen::MatrixXd MotionKalmanFilter::getQ(const double& time) const {
    const double x = 5e-2;
    const double y = 1e1;
    const double r = 80.0;

    Eigen::MatrixXd q(9, 9);
    const double q_x_x = pow(time, 4) / 4 * x;
    const double q_x_vx = pow(time, 3) / 2 * x;
    const double q_vx_vx = pow(time, 2) * x;
    const double q_y_y = pow(time, 4) / 4 * y;
    const double q_y_vy = pow(time, 3) / 2 * y;
    const double q_vy_vy = pow(time, 2) * y;
    const double q_r = pow(time, 4) / 4 * r;

    //    xc      v_xc    yc      v_yc    za      v_za    yaw     v_yaw   r
    q <<  q_x_x,  q_x_vx, 0,      0,      0,      0,      0,      0,      0,
          q_x_vx, q_vx_vx,0,      0,      0,      0,      0,      0,      0,
          0,      0,      q_x_x,  q_x_vx, 0,      0,      0,      0,      0,
          0,      0,      q_x_vx, q_vx_vx,0,      0,      0,      0,      0,
          0,      0,      0,      0,      q_x_x,  q_x_vx, 0,      0,      0,
          0,      0,      0,      0,      q_x_vx, q_vx_vx,0,      0,      0,
          0,      0,      0,      0,      0,      0,      q_y_y,  q_y_vy, 0,
          0,      0,      0,      0,      0,      0,      q_y_vy, q_vy_vy,0,
          0,      0,      0,      0,      0,      0,      0,      0,      q_r;
    return q;
}

Eigen::MatrixXd MotionKalmanFilter::getR(const Eigen::VectorXd& z) const {
    Eigen::DiagonalMatrix<double, 4> r;
    r.diagonal() << abs(rX * z[0]), abs(rX * z[1]), abs(rX * z[2]), rYaw;
    return r;
}

void MotionKalmanFilter::setState(const Eigen::VectorXd & x0) {
    x_post = x0;
}

Eigen::MatrixXd MotionKalmanFilter::predict(const double& gapTime) {
    F = processJacobian(gapTime);
    Q = getQ(gapTime);

    x_pri = process(gapTime);
    P_pri = F * P_post * F.transpose() + Q;

    // handle the case when there will be no measurement before the next predict
    x_post = x_pri;
    P_post = P_pri;
    return x_pri;
}

Eigen::MatrixXd MotionKalmanFilter::update(const Eigen::VectorXd& z) {
    H = observeJacobian();
    R = getR(z);

    K = P_pri * H.transpose() * (H * P_pri * H.transpose() + R).inverse();
    // std::cout<<"KKKK"<< K(0) <<std::endl;
    x_post = x_pri + K * (z - observe());

    P_post = (I - K * H) * P_pri;

    return x_post;
}

bool MotionKalmanFilter::isTurnRight() const {
    return x_pri(7) > 0;
}