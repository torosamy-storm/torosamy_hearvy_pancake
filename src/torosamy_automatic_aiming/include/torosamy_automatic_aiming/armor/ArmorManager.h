#ifndef ARMORMANAGER_H
#define ARMORMANAGER_H

#include <torosamy_automatic_aiming/armor/Armor.h>
#include <torosamy_automatic_aiming/armor/ArmorFilter.h>
#include <yaml-cpp/node/node.h>
#include "torosamy_automatic_aiming/handler/classify/ClassifyHandler.h"


class ArmorManager {
public:
    ArmorManager(const YAML::Node& fileReader);
    bool findArmors(const std::vector<Light>& lights);
    bool findArmors(const cv::Mat& src);
    void drawArmors(cv::Mat& outSrc) const;
    const std::vector<Armor>& getArmors() const;
    void setArmors(const std::vector<Armor>& armors);
    const Armor& getTargetArmor() const;
    void drawTarget(cv::Mat& outSrc) const;
    bool selectTarget();
    void resetTargetArmorType();
private:
    const bool mEnableDraw;
    std::vector<Armor> mArmors;
    ArmorFilter mArmorFilter;
    ClassifyHandler mClassifyHandler;
    Armor mTargetArmor;
    std::vector<bool> mDebugOptions;
};

#endif
