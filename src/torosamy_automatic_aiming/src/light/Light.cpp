#include "torosamy_automatic_aiming/light/Light.h"
#include "utils/MessageUtils.h"


// Light::Light() : mArea(-1) {}
void Light::drawLight(cv::Mat& outSrc) const{
    for (int i = 0; i < 4; i++) Torosamy::MessageUtils::line(outSrc, this->mPoints.at(i), this->mPoints.at((i + 1) % 4));
}
// void drawArmor(cv::Mat& outSrc) const;
// void drawArmor(cv::Mat& outSrc, const std::vector<bool>& debugOptions) const;
void Light::drawLight(cv::Mat& outSrc, const bool& isFilterMode, const std::vector<bool>& debugOptions) const {
    using namespace Torosamy;    
    
    if(!isFilterMode) {
        MessageUtils::line(outSrc, this->mTopPoint, this->mBottomPoint);
    }else {
        for (int i = 0; i < 4; i++) MessageUtils::line(outSrc, this->mPoints.at(i), this->mPoints.at((i + 1) % 4));
    }
    

    float x = mPoints.at(0).x + 50;
    float y = mPoints.at(0).y;

    if(debugOptions.at(0)) {
        MessageUtils::putText(outSrc, "angle = " + std::to_string(mAngle), cv::Point2f(x, y));
        y += 20;
    }


    if(debugOptions.at(1)) {
        MessageUtils::putText(outSrc, "rate = " + std::to_string(mWidth / mLength), cv::Point2f(x, y));
        y += 20;
    }

    if(debugOptions.at(2)) {
        MessageUtils::putText(outSrc, "area = " + std::to_string(mArea), cv::Point2f(x, y));
        y += 20;
    }

}
Light& Light::setArea(const float& area) {
    this->mArea = area;
    return *this;
}
Light& Light::setAngle(const float& angle) {
    this->mAngle = angle;
    return *this;
}
Light& Light::setLength(const float& length) {
    this->mLength = length;
    return *this;
}
Light& Light::setWidth(const float& width) {
    this->mWidth = width;
    return *this;
}

Light& Light::setTopPoint(const cv::Point2f& point) {
    this->mTopPoint = point;
    return *this;
}
Light& Light::setBottomPoint(const cv::Point2f& point) {
    this->mBottomPoint = point;
    return *this;
}

Light& Light::setCenter(const cv::Point2f& center) {
    this->mCenter = center;
    return *this;
}
Light& Light::setPoints(const std::vector<cv::Point2f>& points) {
    this->mPoints = points;
    return *this;
}

const cv::Point2f& Light::getTopPoint() const{
    return this->mTopPoint;
}

const cv::Point2f& Light::getBottomPoint() const{
    return this->mBottomPoint;
}

float Light::getAngle() const {
    return mAngle;
}
float Light::getWidth() const {
    return mWidth;
}

float Light::getArea() const {
    return mArea;
}

float Light::getLength() const{
    return this->mLength;
}

const cv::Point2f& Light::getCenter() const{
    return this->mCenter;
}

const std::vector<cv::Point2f>& Light::getPoints() const{
    return this->mPoints;
}
