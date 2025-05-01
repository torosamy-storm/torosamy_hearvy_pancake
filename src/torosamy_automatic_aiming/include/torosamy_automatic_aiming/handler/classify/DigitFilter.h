//
// Created by torosamy on 25-3-29.
//

#ifndef DIGITFILTER_H
#define DIGITFILTER_H
#include <torosamy_automatic_aiming/armor/Armor.h>

class DigitFilter {
public:
    DigitFilter(const Armor& armor,const cv::Mat& src);
    cv::Mat generateNumberMask() const;
    std::pair<ArmorType, float> classify(const std::vector<float>& minConfidences) const;
private:
    const Armor mArmor;
    const cv::Mat mSrc;
    std::vector<cv::Point2f> getExtendPoints() const;
    static std::map<int, cv::Mat> initTemplates();
    static std::map<int, cv::Mat> mTemplates;
};

#endif //DIGITFILTER_H
