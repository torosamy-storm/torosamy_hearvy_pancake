#ifndef LIGHTMANAGER_H
#define LIGHTMANAGER_H


#include <vector>
#include <yaml-cpp/node/node.h>

#include "torosamy_automatic_aiming/light/LightFilter.h"
#include "torosamy_automatic_aiming/light/Light.h"
#include "packet/ColorType.h"

class LightManager {
public:
    LightManager(const YAML::Node& fileReader);
    const std::vector<Light>& getLights() const;
    bool findLights(const cv::Mat& src, const Torosamy::ColorType& colorType);
    void drawLights(cv::Mat& outSrc) const;


private:
    bool mSplitedMask;
    bool mFilterMake;
    LightFilter mLightFilter;

    std::vector<Light> mLights;

    cv::Mat getLightMask(const cv::Mat& src, const Torosamy::ColorType& colorType) const;
    cv::Mat getLightCompare(const cv::Mat& src, const Torosamy::ColorType& colorType) const;
    cv::Mat getLightSplit(const cv::Mat& src, const Torosamy::ColorType& colorType) const;
    cv::Mat fillHole(const cv::Mat& src) const;
};



#endif
