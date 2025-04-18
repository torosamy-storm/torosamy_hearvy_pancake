//
// Created by torosamy on 25-3-29.
//

#include "torosamy_automatic_aiming/handler/predict/utils/EigenUtil.h"
//TODO
Eigen::Matrix3d EigenUtil::getCameraMatrix(const cv::Mat& cameraMatrix) {
    Eigen::Matrix3d K;
    K << cameraMatrix.at<double>(0,0),cameraMatrix.at<double>(0,1),cameraMatrix.at<double>(0,2),
         cameraMatrix.at<double>(1,0),cameraMatrix.at<double>(1,1),cameraMatrix.at<double>(1,2),
         cameraMatrix.at<double>(2,0),cameraMatrix.at<double>(2,1),cameraMatrix.at<double>(2,2);
    return K;
}

Eigen::Quaterniond EigenUtil::getQuaternion(const double& pitch, const double& yaw) {
    return getQuaternion(pitch, yaw, 0);
}

Eigen::Quaterniond EigenUtil::getQuaternion(const double& pitch, const double& yaw, const double& roll) {
    // 创建四元数（顺序：Yaw -> Pitch -> Roll）
    Eigen::Quaterniond q =
        // 转换为弧度
        Eigen::AngleAxisd(yaw * M_PI / 180.0,   Eigen::Vector3d::UnitY()) *  // Yaw (Z轴)
        Eigen::AngleAxisd(pitch * M_PI / 180.0, Eigen::Vector3d::UnitX()) *
        Eigen::AngleAxisd(roll * M_PI / 180.0,  Eigen::Vector3d::UnitZ()); // Pitch (Y轴)

    q.normalize();
    return q;
}


Eigen::Matrix3d EigenUtil::getRotationMatrix(const double& yawAngle) {
    const double yaw = yawAngle * CV_PI / 180.;//yaw>0更倾向于左边
    const double pitch = -15 * CV_PI / 180.;
    //double roll = 0 * CV_PI / 180.;

    return yawMatrix3d(yaw) * pitchMatrix3d(pitch);
}

Eigen::Matrix4d EigenUtil::getPose(const Eigen::Matrix3d& rvec,const Eigen::Vector3d& tvec) {
    Eigen::Matrix4d pose;
    pose << rvec(0,0),rvec(0,1),rvec(0,2),tvec(0),
        rvec(1,0),rvec(1,1),rvec(1,2),tvec(1),
        rvec(2,0),rvec(2,1),rvec(2,2),tvec(2),
        0,0,0,1;
    return pose;
}

Eigen::Matrix3d EigenUtil::yawMatrix3d(const double& yaw) {
    Eigen::Matrix3d result;
    result <<  cos(yaw),0,sin(yaw),
                0,1,0,
        -sin(yaw),0,cos(yaw);
    return result;
}

Eigen::Matrix3d EigenUtil::rollMatrix3d(const double& roll) {
    Eigen::Matrix3d result;
    result <<  cos(roll),-sin(roll),0,
        sin(roll),cos(roll),0,
        0,0,1;
    return result;
}

Eigen::Matrix3d EigenUtil::pitchMatrix3d(const double& pitch) {
    Eigen::Matrix3d result;
    result <<  1,0,0,
        0,cos(pitch),-sin(pitch),
        0,sin(pitch),cos(pitch);
    return result;
}