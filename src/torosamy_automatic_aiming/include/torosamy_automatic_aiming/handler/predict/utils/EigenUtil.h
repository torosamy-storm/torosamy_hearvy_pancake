//
// Created by torosamy on 25-3-29.
//

#ifndef EIGENUTIL_H
#define EIGENUTIL_H
#include "opencv2/opencv.hpp"
#include "Eigen/Dense"

class EigenUtil {
public:
    static Eigen::Quaterniond getQuaternion(const double& pitch, const double& yaw);
    static Eigen::Quaterniond getQuaternion(const double& pitch, const double& yaw, const double& roll);

    static Eigen::Matrix3d getCameraMatrix(const cv::Mat& cameraMatrix);
    static Eigen::Matrix3d getRotationMatrix(const double& yawAngle);
    static Eigen::Matrix4d getPose(const Eigen::Matrix3d& rvec, const Eigen::Vector3d& tvec);
    static Eigen::Matrix3d yawMatrix3d(const double& yaw);
    static Eigen::Matrix3d rollMatrix3d(const double& roll);
    static Eigen::Matrix3d pitchMatrix3d(const double& pitch);
private:
    EigenUtil() = default;
    ~EigenUtil() = delete;
    EigenUtil(const EigenUtil&) = delete;
    EigenUtil& operator=(const EigenUtil&) = delete;
    EigenUtil(EigenUtil&&) = delete;
    EigenUtil& operator=(EigenUtil&&) = delete;
};

#endif //EIGENUTIL_H
