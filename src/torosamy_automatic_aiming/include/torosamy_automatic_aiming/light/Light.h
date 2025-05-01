#ifndef LIGHT_H
#define LIGHT_H
#include <opencv2/opencv.hpp>
#include <vector>

class Light {
public:
    Light() = default;

    void drawLight(cv::Mat& outSrc) const;
    void drawLight(cv::Mat& outSrc, const bool& isFilterMode, const std::vector<bool>& debugOptions) const;

    Light& setArea(const float& area);
    Light& setAngle(const float& angle);
    Light& setLength(const float& length);
    Light& setWidth(const float& width);
    Light& setTopPoint(const cv::Point2f& point);
    Light& setBottomPoint(const cv::Point2f& point);
    Light& setPoints(const std::vector<cv::Point2f>& points);
    Light& setCenter(const cv::Point2f& center);

    float getAngle() const;
    float getLength() const;
    float getWidth() const;
    float getArea() const;
    const cv::Point2f& getTopPoint() const;
    const cv::Point2f& getBottomPoint() const;
    const cv::Point2f& getCenter() const;



    const std::vector<cv::Point2f>& getPoints() const;

private:
    std::vector<cv::Point2f> mPoints;
    cv::Point2f mTopPoint;
    cv::Point2f mBottomPoint;
    cv::Point2f mCenter;
    float mLength;
    float mWidth;
    float mArea;
    float mAngle;
};



#endif //LIGHT_H
