//
// Created by torosamy on 25-3-31.
//
#include "torosamy_automatic_aiming/handler/predict/PointPredicter.h"
#include "utils/MessageUtils.h"
PointPredicter::PointPredicter(const YAML::Node& fileReader) :
    mArmorVxRate(fileReader["armor"]["VxRate"].as<float>()),
    mArmorVyRate(fileReader["armor"]["VyRate"].as<float>()),
    mOutpostVxRate(fileReader["outpost"]["VxRate"].as<float>()),
    mOutpostVyRate(fileReader["outpost"]["VyRate"].as<float>()),
    mLastOutpostYaw(0),
    mLastOutpostPitch(0),
    mArmorStartTime(Torosamy::MessageUtils::getTimePoint()),
    mOutpostStartTime(Torosamy::MessageUtils::getTimePoint()),
    mOutpostDt(fileReader["outpost"]["dt"].as<float>()),
    mArmorDt(fileReader["armor"]["dt"].as<float>()){
}

bool PointPredicter::isPointValid(const Armor& armor, const PointKalmanMode& mode) {
    const Eigen::Vector2d x = mode == PointKalmanMode::ARMOR ? mArmorKalman.getPredictedCenter() : mOutpostKalman.getPredictedCenter();
    if (x[0] == 0 && x[1] == 0) return true;
    return norm(cv::Point2f(x[0], x[1]) - armor.getCenter()) < armor.getHeight() * 1.5;
}



cv::Point2f PointPredicter::getPredictPoint(const double& filghtTime, const Armor& armor) {
    if(armor.getArmorType() == ArmorType::OUTPOST) {
        throw std::invalid_argument("Armor type is OUTPOST");
    }

    if (!isPointValid(armor, PointKalmanMode::ARMOR)){
        mArmorKalman.updateX(armor.getCenter());
        mArmorKalman.updateX(0.8, 0.8);
    }

    Torosamy::TIME endTime = Torosamy::MessageUtils::getTimePoint();
    const double gapTime = Torosamy::MessageUtils::getTimeByPoint(mArmorStartTime, endTime);
    mArmorStartTime = endTime;

    mArmorKalman.predict(gapTime);
    mArmorKalman.updateX(mArmorVxRate,mArmorVyRate);
    mArmorKalman.update(armor.getCenter().x, armor.getCenter().y);

    Eigen::Vector2d futurePosition = mArmorKalman.predictFuturePosition(gapTime + filghtTime + mArmorDt);
    return cv::Point2f(futurePosition(0),futurePosition(1));
}
cv::Point2f PointPredicter::getPredictPoint(const double& filghtTime, const Armor& armor, const float& yaw, const float& pitch) {
    if(armor.getArmorType() != ArmorType::OUTPOST) {
        throw std::invalid_argument("Armor type is not OUTPOST");
    }

    if (!isPointValid(armor, PointKalmanMode::OUTPOST)){
        mOutpostKalman.updateX(armor.getCenter());
        mOutpostKalman.updateX(1.1064, 1.1064);
        mLastOutpostYaw = yaw;
        mLastOutpostPitch = pitch;
    }

    Torosamy::TIME endTime = Torosamy::MessageUtils::getTimePoint();
    const double gapTime = Torosamy::MessageUtils::getTimeByPoint(mOutpostStartTime, endTime);
    mOutpostStartTime = endTime;
    mOutpostKalman.predict(gapTime);
    mOutpostKalman.updateX(mOutpostVxRate,mOutpostVyRate);

    double dyaw = (mLastOutpostYaw - yaw) * 1.28;
    double dpitch = (mLastOutpostPitch - pitch)* 5.10;
    if(dyaw <= -7.5 && dyaw >= 7.5) dyaw = 0;
    if(dpitch <= -4.5 && dpitch >= 4.5) dpitch = 0;

    mOutpostKalman.update(
        armor.getCenter().x + dyaw,
        armor.getCenter().y + dpitch
    );

    Eigen::Vector2d futurePosition = mOutpostKalman.predictFuturePosition(gapTime + filghtTime +  mOutpostDt);
    return cv::Point2f(futurePosition(0),futurePosition(1));
}


// bool PointPredicter::isTurnRight() const {
//     return mP
// }