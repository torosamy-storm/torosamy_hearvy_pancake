#ifndef LIGHTFILTER_H
#define LIGHTFILTER_H

#include <torosamy_automatic_aiming/light/Light.h>
#include <yaml-cpp/node/convert.h>

class LightFilter {
public:
    LightFilter(const YAML::Node& fileReader);
    bool isLight(const Light& light) const;
    bool checkArea(const float& area) const;
    bool checkRate(const float& rate) const;
    bool checkAngle(const float& angle) const;

private:
    const std::pair<float,float> mConditionArea;
    const std::pair<float,float> mConditionRate;
    const std::pair<float,float> mConditionAngle;
};

#endif
