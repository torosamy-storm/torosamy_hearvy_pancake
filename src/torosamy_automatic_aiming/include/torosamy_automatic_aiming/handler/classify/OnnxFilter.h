//
// Created by torosamy on 25-3-19.
//

#ifndef ONNXFILTER_H
#define ONNXFILTER_H

#include <torosamy_automatic_aiming/armor/Armor.h>

class OnnxFilter {
public:
	  OnnxFilter(const Armor& armor,const cv::Mat& src);
    cv::Mat generateNumberMask() const;
    std::pair<ArmorType, float> classify();
private:
    const Armor mArmor;
    const cv::Mat mSrc;

  	static cv::dnn::Net mNet;
    std::vector<cv::Point2f> getSortPoints(const std::vector<cv::Point2f>& points) const;
    std::vector<cv::Point2f> getExtendPoints() const;
    std::pair<float,float> getAffineOffestPointsParam() const;
    std::vector<float> getAffineOffestPointsX(const std::vector<cv::Point2f>& points) const;
    std::vector<float> getAffineOffestPointsY(const std::vector<cv::Point2f>& points) const;
};

#endif //ONNXFILTER_H
