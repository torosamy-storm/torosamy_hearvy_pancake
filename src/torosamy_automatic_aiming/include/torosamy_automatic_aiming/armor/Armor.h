#ifndef ARMOR_H
#define ARMOR_H

#include "torosamy_automatic_aiming/light/Light.h"
#include "Eigen/Dense"

enum class ArmorType {
    ONE,
    TWO,
    THREE,
    FOUR,
    FIVE,
    OUTPOST,
    GUARD,
    BASE,
    NEGATIVE
};

class Armor {
public:
    Armor() = default;
    Armor(const Light& light1, const Light& light2);
    void drawArmor(cv::Mat& outSrc) const;
    void drawArmor(cv::Mat& outSrc, const std::vector<bool>& debugOptions) const;

    Armor generatePredictArmor(const cv::Point2f& center) const;
    void setArmorType(const ArmorType& armorType);

    bool isBigArmor() const;
    float getArea() const;
    float getLightLengthRate() const;
    float getAngle() const;
    float getAngleMate() const;
    float getRate() const;
    float getHeight() const;
    float getLength() const;
    const Light& getLeftLight() const;
    const Light& getRightLight() const;
    const cv::Point2f& getCenter() const;
    ArmorType getArmorType() const;
private:
    void setCenter(const cv::Point2f& center);

    std::vector<cv::Point2f> mPoints;
    cv::Point2f mCenter;
    ArmorType mType;
    Light mLeftLight;
    Light mRightLight;
    float mArea;
    float mLightLengthRate;
    float mAngle;
    float mAngleMate;
    float mRate;
    float mHeight;
    float mLength;
};

#endif //ARMOR_H
