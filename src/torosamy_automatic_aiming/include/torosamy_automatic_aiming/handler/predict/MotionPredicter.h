//
// Created by torosamy on 25-3-31.
//

#ifndef MOTIONPREDICTER_H
#define MOTIONPREDICTER_H

#include "torosamy_automatic_aiming/armor/ArmorPnpResult.h"
#include "torosamy_automatic_aiming/handler/predict/kalman/extended/MotionKalmanFilter.h"

class MotionPredicter {
public:
    MotionPredicter(const YAML::Node& fileReader);
    void init(const ArmorPnpResult& armorPnpResult);
    void updateGapTime();
    bool update(const std::vector<ArmorPnpResult>& armorHandlers);
    const Eigen::VectorXd& getHandleResult() const;
    Eigen::Vector3d generatePredictTvec(const double& filghtTime, const float& gainPitch, const float& gainYaw) const;
    bool isTurnRight() const;
private:
    void handlerUpdateResult();
    void handleArmorJump(const Eigen::Vector3d& worldCenter, const float& armorYaw);
    double orientationToYaw(const float& armorYaw);
    Eigen::Vector3d getArmorPositionFromState(const Eigen::VectorXd & x);

    const float mDt;
    double mGapTime;
    Torosamy::TIME mStartTime; // 上一次更新时间
    bool mInited;
    double mDz;
    double mAnotherR;
    double mLastYaw;
    ArmorType mTrackedType;
    Eigen::VectorXd mHandleResult;
    MotionKalmanFilter mArmorKalmanFilter;

    const float mMaxMatchDistance;
    const float mMaxMatchYawDiff;
    const float mTrackingThres;
    const float mMaxTargetHeight;
};

#endif //MOTIONPREDICTER_H
