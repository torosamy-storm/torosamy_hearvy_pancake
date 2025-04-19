//
// Created by torosamy on 25-3-19.
//
#include "torosamy_automatic_aiming/armor/ArmorFilter.h"

ArmorFilter::ArmorFilter(const YAML::Node& fileReader) :
    mConditionLightLengthRate(
        fileReader["LengthRate"]["max"].as<double>(),
        fileReader["LengthRate"]["min"].as<double>()
    ),
    mConditionAngle(
        fileReader["Angle"]["max"].as<double>(),
        fileReader["Angle"]["min"].as<double>()
    ),
    mConditionAngleMate(
        fileReader["AngleMate"]["max"].as<double>(),
        fileReader["AngleMate"]["min"].as<double>()
    ),
    mConditionRate(
        fileReader["Rate"]["max"].as<double>(),
        fileReader["Rate"]["min"].as<double>()
    ),
    mConditionArea(
        fileReader["Area"]["max"].as<double>(),
        fileReader["Area"]["min"].as<double>()
    ){
}


bool ArmorFilter::isArmor(const Armor& armor) const {
    if(!checkLightLengthRate(armor.getLightLengthRate())) return false;
    if(!checkAngle(armor.getAngle())) return false;
    if(!checkAngleMate(armor.getAngleMate())) return false;
    if(!checkRate(armor.getRate())) return false;
    if(!checkArea(armor.getArea())) return false;
    return true;
}

bool ArmorFilter::checkLightLengthRate(const float& lightLengthRate) const {
    const float max = mConditionLightLengthRate.first;
    const float min = mConditionLightLengthRate.second;
    return (lightLengthRate >= min && lightLengthRate <= max);
}

bool ArmorFilter::checkAngle(const float& angle) const {
    const float max = mConditionAngle.first;
    const float min = mConditionAngle.second;
    return (angle >= min && angle <= max);
}

bool ArmorFilter::checkAngleMate(const float& angleMate) const {
    const float max = mConditionAngleMate.first;
    const float min = mConditionAngleMate.second;
    return (angleMate >= min && angleMate <= max);
}

bool ArmorFilter::checkRate(const float& rate) const {
    const float max = mConditionRate.first;
    const float min = mConditionRate.second;
    return (rate >= min && rate <= max);
}

bool ArmorFilter::checkArea(const float& area) const {
    const float max = mConditionArea.first;
    const float min = mConditionArea.second;
    return (area >= min && area <= max);
}
