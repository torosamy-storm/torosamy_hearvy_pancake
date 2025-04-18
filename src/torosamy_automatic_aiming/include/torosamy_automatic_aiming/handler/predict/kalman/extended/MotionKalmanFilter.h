//
// Created by torosamy on 25-3-30.
//

#ifndef MOTIONKALMANFILTER_H
#define MOTIONKALMANFILTER_H

#include <Eigen/Dense>
#include "utils/MessageUtils.h"
#include "yaml-cpp/yaml.h"


class MotionKalmanFilter {
public:
    MotionKalmanFilter(const YAML::Node& fileReader);
    // Set the initial state
    void setState(const Eigen::VectorXd & x0);

    // Compute a predicted state
    Eigen::MatrixXd predict(const double& gapTime);

    // Update the estimated state based on measurement
    Eigen::MatrixXd update(const Eigen::VectorXd & z);
    bool isTurnRight() const;
private:
    Eigen::VectorXd process(const double& time) const;
    Eigen::VectorXd observe() const;
    Eigen::MatrixXd observeJacobian() const;
    Eigen::MatrixXd processJacobian(const double& time) const;
    Eigen::MatrixXd getQ(const double& time) const;
    Eigen::MatrixXd getR(const Eigen::VectorXd& z) const;

    Eigen::MatrixXd F;

    Eigen::MatrixXd H;

    Eigen::MatrixXd Q;

    Eigen::MatrixXd R;

    Eigen::MatrixXd P_pri;

    Eigen::MatrixXd P_post;

    Eigen::MatrixXd K;

    Eigen::MatrixXd I;

    Eigen::VectorXd x_pri;

    Eigen::VectorXd x_post;


    const double rYaw;
    const double rX;
};

#endif //MOTIONKALMANFILTER_H
