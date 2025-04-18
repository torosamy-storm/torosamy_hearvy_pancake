#ifndef POINTKALMANFILTER_H
#define POINTKALMANFILTER_H
#include <Eigen/Dense>
#include <opencv2/opencv.hpp>
#include <pod/manager/PacketManager.h>
#include <utils/MessageUtils.h>

class PointKalmanFilter {
public:
    PointKalmanFilter();
    void updateX(const cv::Point2f& point);
    void updateX(const double& VxRate, const double& VyRate);
    // 预测步骤
    void predict(double dt);
    // 更新步骤
    void update(double meas_x, double meas_y);
    // 获取预测的中心位置
    Eigen::Vector2d getPredictedCenter() const;
    // 获取未来 t 秒的预测位置
    Eigen::Vector2d predictFuturePosition(double time);
private:
    Eigen::VectorXd x; // 状态向量 [x, y, dx, dy]
    Eigen::MatrixXd P; // 状态协方差矩阵
    Eigen::MatrixXd A; // 状态转移矩阵
    Eigen::MatrixXd H; // 观测矩阵
    Eigen::MatrixXd Q; // 过程噪声协方差矩阵
    Eigen::MatrixXd R; // 测量噪声协方差矩阵
    Eigen::MatrixXd K; // 卡尔曼增益矩阵
    Eigen::MatrixXd I; // 单位矩阵
};



#endif
