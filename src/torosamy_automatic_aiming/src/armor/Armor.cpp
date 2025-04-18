//
// Created by torosamy on 25-3-19.
//
#include <torosamy_automatic_aiming/armor/Armor.h>
#include "utils/MessageUtils.h"
Armor::Armor(const Light& light1, const Light& light2) :
    mType(ArmorType::NEGATIVE) {
    mLeftLight = light1.getCenter().x < light2.getCenter().x ? light1 : light2;
    mRightLight = light1.getCenter().x > light2.getCenter().x ? light1 : light2;


    mCenter = (mLeftLight.getCenter() + mRightLight.getCenter()) / 2;

    mPoints.push_back(mLeftLight.getBottomPoint());  // 左下
    mPoints.push_back(mLeftLight.getTopPoint());     // 左上
    mPoints.push_back(mRightLight.getTopPoint());    // 右上
    mPoints.push_back(mRightLight.getBottomPoint()); // 右下

    mHeight = norm(mLeftLight.getTopPoint() - mLeftLight.getBottomPoint());
    mLength = norm(mLeftLight.getBottomPoint() - mRightLight.getBottomPoint());

    mRate = mHeight / mLength;

    // 粗略计算面积,用于优先击打判断
    mArea = contourArea(mPoints);


    mLightLengthRate = mLeftLight.getLength() < mRightLight.getLength() ? mLeftLight.getLength() / mRightLight.getLength() : mRightLight.getLength() / mLeftLight.getLength();



    cv::Point2f diff = mLeftLight.getCenter() - mRightLight.getCenter();
    mAngle = fabs(atan(diff.y / diff.x)) / CV_PI * 180;

    // 求灯条夹角
    cv::Point2f diffLeft = mLeftLight.getTopPoint() - mLeftLight.getBottomPoint();
    cv::Point2f diffRight = mRightLight.getTopPoint() - mRightLight.getBottomPoint();
    float angleLeft = atan(diffLeft.y / diffLeft.x) / CV_PI * 180;
    float angleRight = atan(diffRight.y / diffRight.x) / CV_PI * 180;
    if (angleLeft < 0) angleLeft += 180;
    if (angleRight < 0) angleRight += 180;
    mAngleMate = fabs(angleLeft - angleRight);

}

Armor Armor::generatePredictArmor(const cv::Point2f& center) const{
    const cv::Point2f& dPoint = center - mCenter;

    Armor result = *this;
    result.mCenter = center;
    // result.setCenter(center);
    for (cv::Point2f& point : result.mPoints) {
        point += dPoint;
    }
    return result;
}


void Armor::setCenter(const cv::Point2f& center) {
    mCenter = center;
}

void Armor::setArmorType(const ArmorType& armorType) {
	mType = armorType;
}

float Armor::getHeight() const {
    return mHeight;
}
float Armor::getLength() const {
    return mLength;
}

float Armor::getArea() const {
    return this->mArea;
}
float Armor::getLightLengthRate() const {
    return this->mLightLengthRate;
}
float Armor::getAngle() const {
    return this->mAngle;
}
float Armor::getAngleMate() const {
    return this->mAngleMate;
}
float Armor::getRate() const {
    return this->mRate;
}
ArmorType Armor::getArmorType() const {
    return this->mType;
}

const cv::Point2f& Armor::getCenter() const {
    return this->mCenter;
}
void Armor::drawArmor(cv::Mat& outSrc) const {
    using namespace Torosamy;
 
    for (int i = 0; i < 4; i++) MessageUtils::line(outSrc, mPoints.at(i), mPoints.at((i + 1) % 4));

    MessageUtils::circle(outSrc, mCenter);
}
const Light& Armor::getLeftLight() const{
    return this->mLeftLight;
}
const Light& Armor::getRightLight() const{
    return this->mRightLight;
}

bool Armor::isBigArmor() const {
    return this->mRate < 0.3;
}

void Armor::drawArmor(cv::Mat& outSrc, const std::vector<bool>& debugOptions) const {
    using namespace Torosamy;
    //置信度
    float x = mPoints.at(0).x + 50;
    float y = mPoints.at(0).y;

    //MessageUtils::putText(outSrc, mClassifyResult, cv::Point2f(x+15, y));


    if(debugOptions.at(0)) {
        MessageUtils::putText(outSrc, "angle = " + std::to_string(mAngle), cv::Point2f(x, y));
        y += 20;
    }

    if(debugOptions.at(1)) {
        MessageUtils::putText(outSrc, "angle_mate = " + std::to_string(mAngleMate), cv::Point2f(x, y));
        y += 20;
    }
    if(debugOptions.at(2)) {
        MessageUtils::putText(outSrc, "rate = " + std::to_string(mRate), cv::Point2f(x, y));
        y += 20;
    }
    if(debugOptions.at(3)) {
        MessageUtils::putText(outSrc, "length_rate = " + std::to_string(mLightLengthRate), cv::Point2f(x, y));
        y += 20;
    }
    if(debugOptions.at(4)) {
        MessageUtils::putText(outSrc, "area = " + std::to_string(mArea), cv::Point2f(x, y));
        y += 20;
    }


    for (int i = 0; i < 4; i++) MessageUtils::line(outSrc, mPoints.at(i), mPoints.at((i + 1) % 4));

    MessageUtils::circle(outSrc, mCenter);
}
