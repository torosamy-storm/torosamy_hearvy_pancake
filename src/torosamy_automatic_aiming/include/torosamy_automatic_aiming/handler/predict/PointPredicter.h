//
// Created by torosamy on 25-3-31.
//

#ifndef POINTPREDICTER_H
#define POINTPREDICTER_H
#include "torosamy_automatic_aiming/handler/predict/kalman/linear/PointKalmanFilter.h"
#include "torosamy_automatic_aiming/armor/Armor.h"
#include "yaml-cpp/yaml.h"



class PointPredicter {
public:
    PointPredicter(const YAML::Node& fileReader);
    cv::Point2f getPredictPoint(const double& filghtTime, const Armor& armor);
    cv::Point2f getPredictPoint(const double& filghtTime, const Armor& armor, const float& yaw, const float& pitch);
    // bool isTurnRight() const;
    // bool doKalman();
private:
    enum class PointKalmanMode {
        ARMOR,
        OUTPOST
    };
    bool isPointValid(const Armor& armor, const PointKalmanMode& mode);
    PointKalmanFilter mArmorKalman;
    PointKalmanFilter mOutpostKalman;
    // 上一次更新时间
    Torosamy::TIME mArmorStartTime;
    Torosamy::TIME mOutpostStartTime;
    const float mArmorVxRate;
    const float mArmorVyRate;

    const float mOutpostVxRate;
    const float mOutpostVyRate;

    float mLastOutpostYaw;
    float mLastOutpostPitch;

    const float mOutpostDt;
    const float mArmorDt;
};

#endif //POINTPREDICTER_H
