//
// Created by torosamy on 25-3-19.
//

#include "torosamy_automatic_aiming/light/LightFilter.h"


LightFilter::LightFilter(const YAML::Node& fileReader) :
    mConditionArea(
        fileReader["Area"]["max"].as<float>(),
        fileReader["Area"]["min"].as<float>()
    ),
    mConditionRate(
        fileReader["Rate"]["max"].as<float>(),
        fileReader["Rate"]["min"].as<float>()
    ),
    mConditionAngle(
        fileReader["Angle"]["max"].as<float>(),
        fileReader["Angle"]["min"].as<float>()
    ){
}

bool LightFilter::isLight(const Light& light) const{
    if(!checkArea(light.getArea())) return false;
    if(!checkRate(light.getWidth() / light.getLength())) return false;
    if(!checkAngle(light.getAngle())) return false;
    return true;
}

bool LightFilter::checkAngle(const float& angle) const{
    const float maxAngle = mConditionAngle.first;
    const float minAngle = mConditionAngle.second;
    return angle < maxAngle && angle > minAngle;
}

bool LightFilter::checkArea(const float& area) const{
    const float maxArea = mConditionArea.first;
    const float minArea = mConditionArea.second;

    return area < maxArea && area > minArea;
}

bool LightFilter::checkRate(const float& rate) const{
    const float maxRate = mConditionRate.first;
    const float minRate = mConditionRate.second;
    return rate < maxRate && rate > minRate;
}