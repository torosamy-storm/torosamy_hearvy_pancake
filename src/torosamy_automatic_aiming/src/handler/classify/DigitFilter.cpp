//
// Created by torosamy on 25-3-29.
//
#include "torosamy_automatic_aiming/handler/classify/DigitFilter.h"
#include "module/TorosamyModule.h"
std::map<int, cv::Mat> DigitFilter::mTemplates = initTemplates();

std::map<int, cv::Mat> DigitFilter::initTemplates() {
    std::map<int, cv::Mat>  templates;
    std::string path = Torosamy::TorosamyModule::getConfigLocation("armor") + "digit/";
    for(int i = 0; i < 8;i++) {
        cv::Mat img = cv::imread(path + std::to_string(i) + ".png", cv::IMREAD_GRAYSCALE);
        cv::Mat resizedImg;
        cv::resize(img, resizedImg, cv::Size(), 0.5, 0.5, cv::INTER_AREA);  // 降采样到一半分辨率
        templates[i] = resizedImg;
    }
    return templates;
}

DigitFilter::DigitFilter(const Armor& armor,const cv::Mat& src):
    mArmor(armor),
    mSrc(src){
}

std::pair<ArmorType, float> DigitFilter::classify(const std::vector<float>& minConfidences) const{
    const cv::Mat mask = generateNumberMask();
    cv::Mat resizedMask;

    cv::resize(mask, resizedMask, cv::Size(), 0.5, 0.5, cv::INTER_AREA);

    int resultType = 8;
    double resultConfidence = 0;


    #pragma omp parallel for
    for(const auto& templatePair : mTemplates) {
        const int& digit = templatePair.first;
        const cv::Mat& templateImg = templatePair.second;
        cv::Mat result;
        cv::matchTemplate(resizedMask, templateImg, result, cv::TM_CCOEFF_NORMED);

        double confidence;
        cv::Point classIdPoint;
        cv::minMaxLoc(result, nullptr, &confidence, nullptr, &classIdPoint);
        confidence *= 100;

        if (confidence < minConfidences.at(digit)) continue;


        if (resultConfidence < confidence) {
            resultConfidence = confidence;
            resultType = digit;
            // std::cout << 1<< std::endl;
        }

    }
    // cv::imshow("roi",mask); 
    // std::cout <<resultType <<std::endl;
    return {static_cast<ArmorType>(resultType), resultConfidence};
}


cv::Mat DigitFilter::generateNumberMask() const{
    const std::vector<cv::Point2f> newArmorPoints = getExtendPoints();
    cv::Mat result;

    const float x = newArmorPoints.at(0).x - newArmorPoints.at(1).x;
    const float y = newArmorPoints.at(0).y - newArmorPoints.at(3).y;
    cv::Point2f affinePoints1[4];
    affinePoints1[0] = cv::Point2f(newArmorPoints.at(0).x-0.2 * x, newArmorPoints.at(0).y);
    affinePoints1[1] = cv::Point2f(newArmorPoints.at(1).x+0.2 * x, newArmorPoints.at(1).y);
    affinePoints1[2] = cv::Point2f(newArmorPoints.at(2).x+0.2 * x, newArmorPoints.at(2).y);
    affinePoints1[3] = cv::Point2f(newArmorPoints.at(3).x-0.2 * x, newArmorPoints.at(3).y);


    cv::Point2f affinePoints2[4] = {
        cv::Point2f(0, 0),
        cv::Point2f(36, 0),
        cv::Point2f(36, 48),
        cv::Point2f(0, 48)
    };

    warpPerspective(mSrc, result, getPerspectiveTransform(affinePoints1, affinePoints2), cv::Size(36, 48), cv::INTER_CUBIC);
    // Binarize二值化
    cv::GaussianBlur(result, result, cv::Size(5, 5), 0);

    cv::Mat gray;
    cv::cvtColor(result, gray, cv::COLOR_BGR2GRAY);
    cv::threshold(gray,gray, 0, 255, cv::THRESH_BINARY | cv::THRESH_OTSU);

    return gray;
}


std::vector<cv::Point2f> DigitFilter::getExtendPoints() const {
     std::vector<cv::Point2f> result(4);

     const cv::Point2f leftTopPoint = mArmor.getLeftLight().getTopPoint();
     const cv::Point2f rightTopPoint = mArmor.getRightLight().getTopPoint();
     const cv::Point2f leftBottomPoint = mArmor.getLeftLight().getBottomPoint();
     const cv::Point2f rightBottomPoint = mArmor.getRightLight().getBottomPoint();

     const double Left_diff = std::abs(leftTopPoint.y - leftBottomPoint.y) / 2;
     const double Right_diff = std::abs(rightTopPoint.y - rightBottomPoint.y) / 2;

     result[0] = cv::Point2f(leftTopPoint.x,leftTopPoint.y - Left_diff); // 左上
     result[1] = cv::Point2f(rightTopPoint.x,rightTopPoint.y - Right_diff);//右上
     result[2] = cv::Point2f(rightBottomPoint.x,rightBottomPoint.y + Right_diff);//右下
     result[3] = cv::Point2f(leftBottomPoint.x,leftBottomPoint.y + Left_diff);//左下
     return result;
}
