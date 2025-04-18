//
// Created by torosamy on 25-3-19.
//

#ifndef ARMORFILTER_H
#define ARMORFILTER_H
#include <torosamy_automatic_aiming/armor/Armor.h>
#include <yaml-cpp/node/node.h>
class ArmorFilter {
public:
    ArmorFilter(const YAML::Node& fileReader);

    bool isArmor(const Armor& armor) const;
    bool checkLightLengthRate(const float& lightLengthRate) const;
    bool checkAngle(const float& angle) const;
    bool checkAngleMate(const float& angleMate) const;
    bool checkRate(const float& rate) const;
    bool checkArea(const float& area) const;
private:
    const std::pair<float, float> mConditionLightLengthRate;
    const std::pair<float, float> mConditionAngle;
    const std::pair<float, float> mConditionAngleMate;
    const std::pair<float, float> mConditionRate;
    const std::pair<float, float> mConditionArea;
};

#endif //ARMORFILTER_H
