#include "torosamy_automatic_aiming/handler/predict/kalman/linear/PointKalmanFilter.h"
#include "utils/MessageUtils.h"






void PointKalmanFilter::updateX(const cv::Point2f& point) {
    this->x[0] = point.x;
    this->x[1] = point.y;
}
void PointKalmanFilter::updateX(const double& VxRate, const double& VyRate) {
    this->x[2] *= VxRate;
    this->x[3] *= VyRate;
}


PointKalmanFilter::PointKalmanFilter() : x(4), P(4, 4), A(4, 4), H(2, 4), Q(4, 4), R(2, 2), K(4, 2), I(4, 4) {
    // 初始化状态向量，假设初始位置和初始速度为0
    x << 0, 0, 0, 0;

    // 初始化状态协方差矩阵 P
    P = Eigen::MatrixXd::Identity(4, 4);

    // 初始化观测矩阵 H
    H << 1, 0, 0, 0,
        0, 1, 0, 0;

    // 初始化过程噪声协方差矩阵 Q
    Q = Eigen::MatrixXd::Identity(4, 4) * 0.1; // 根据实际情况调整

    // 初始化测量噪声协方差矩阵 R
    R = Eigen::MatrixXd::Identity(2, 2) * 0.1; // 根据实际情况调整

    // 初始化单位矩阵 I
    I = Eigen::MatrixXd::Identity(4, 4);
}

void PointKalmanFilter::predict(double dt) {
    // 更新状态转移矩阵 A
    A << 1, 0, dt, 0,
        0, 1, 0, dt,
        0, 0, 1, 0,
        0, 0, 0, 1;

    // 预测状态向量
    x = A * x;

    // 预测模型的精度
    //  预测协方差矩阵
    P = A * P * A.transpose() + Q;
}

void PointKalmanFilter::update(double meas_x, double meas_y) {
    // 获取当前测量值
    Eigen::VectorXd z(2);
    z << meas_x, meas_y;

    // 计算卡尔曼增益
    Eigen::MatrixXd S = H * P * H.transpose() + R; // 分母
    K = P * H.transpose() * S.inverse();

    // 更新状态向量
    x = x + K * (z - H * x);

    // 更新协方差矩阵
    P = (I - K * H) * P;
}

Eigen::Vector2d PointKalmanFilter::getPredictedCenter() const {
    return x.head<2>();
}

Eigen::Vector2d PointKalmanFilter::predictFuturePosition(double future_time) {
    // 状态转移矩阵 A 只用于预测未来位置，不更新滤波器状态
    Eigen::MatrixXd future_A(4, 4);
    future_A << 1, 0, future_time, 0,
        0, 1, 0, future_time,
        0, 0, 1, 0,
        0, 0, 0, 1;

    // 预测未来的状态向量
    Eigen::VectorXd future_x = future_A * x;
    return future_x.head<2>();
}

// bool PointKalmanFilter::isTurnRight() const {
//     // TODO
//     return lastX.[0] > x[0];
// }