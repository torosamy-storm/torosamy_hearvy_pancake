#ifndef LIGHTFACTORY_H
#define LIGHTFACTORY_H


#include <torosamy_automatic_aiming/light/Light.h>
#include <torosamy_automatic_aiming/light/LightFilter.h>

class LightFactory {
public:
    static cv::Mat getMaskImg(const std::vector<cv::Point2i>& maskContour, const cv::Rect& rect);
    static std::vector<cv::Point2i> getMaskContour(const std::vector<cv::Point2i>& contour, const cv::Rect& rect);

    static Light extendLight(const Light& oldLight);
    static Light makeLight(const std::vector<cv::Point2i>& contour, const LightFilter& filter);
    static Light makeLight(const std::vector<cv::Point2i>& contour);
private:
    LightFactory() = default;
    ~LightFactory() = delete;
    LightFactory(const LightFactory&) = delete;
    LightFactory& operator=(const LightFactory&) = delete;
    LightFactory(LightFactory&&) = delete;
    LightFactory& operator=(LightFactory&&) = delete;
};


#endif //LIGHTFACTORY_H
