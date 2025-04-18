//
// Created by torosamy on 25-3-30.
//

#ifndef ARMORPNPRESULT_H
#define ARMORPNPRESULT_H
#include "torosamy_automatic_aiming/armor/Armor.h"

class ArmorPnpResult {
public:
    ArmorPnpResult(
        const cv::Mat& cameraMatrix,
        const cv::Mat& distCoeffs,
        const float& pitch,
        const float& yaw,
        const Armor& armor
    );
    ArmorPnpResult(const Eigen::Vector3d& tvec, const Armor& armor);
    const Eigen::Vector3d& getPose() const;
    ArmorType getArmorType() const;
    float getArmorYaw() const;
    float getDistance() const;
    const Eigen::Vector3d& getWorldCenter() const;
private:
    void initPnpDistance(const cv::Mat& cameraMatrix, const cv::Mat& distCoeffs);
    void initArmorYaw(const cv::Mat& cameraMatrix);
    void initWorldCenter(const float& gainPitch, const float& gainYaw);

    std::vector<cv::Point2f> generateTransPoint(const Eigen::Matrix3d& rvec, const Eigen::Matrix3d& cameraMatrix) const;
    double generateCost(const std::vector<cv::Point2f>& points) const;

    static std::vector<cv::Point2f> generateRealPoints(const Armor& armor);
    static std::vector<cv::Point3f> generateWorldPoints(const Armor& armor);


    const ArmorType mArmorType;
    float mArmorYaw;
    float mDistance;
    Eigen::Vector3d mTvec;
    Eigen::Vector3d mWorldCenter;
    std::vector<cv::Point2f> mProjectionPoints;
    const std::vector<cv::Point2f> mRealPoints;
    const std::vector<cv::Point3f> mWorldPoints;
};

#endif //ARMORPNPRESULT_H
