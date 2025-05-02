#include "torosamy_automatic_aiming/light/LightManager.h"
#include <torosamy_automatic_aiming/light/LightFactory.h>


LightManager::LightManager(const YAML::Node& fileReader) :
    mEnableDraw(fileReader["DrawAll"].as<bool>()),
    mLightFilter(fileReader) {
    mSplitedMask = fileReader["Splited"].as<bool>();
    mFilterMake = fileReader["FilterMake"].as<bool>();

    mDebugOptions.emplace_back(fileReader["Debug"]["angle"].as<bool>());
    mDebugOptions.emplace_back(fileReader["Debug"]["rate"].as<bool>());
    mDebugOptions.emplace_back(fileReader["Debug"]["area"].as<bool>());
}


bool LightManager::findLights(const cv::Mat& src, const Torosamy::ColorType& colorType) {

    const cv::Mat mask = getLightMask(src,colorType);
    std::vector<std::vector<cv::Point2i>> contours;
    cv::findContours(mask, contours, cv::RETR_EXTERNAL, cv::CHAIN_APPROX_SIMPLE);

    mLights.clear();
    for (const std::vector<cv::Point2i>& contour : contours) {
        if (contour.size() < 5) continue;

        if (!mFilterMake) {
            Light light = LightFactory::makeLight(contour);
            if (mLightFilter.isLight(light)) mLights.emplace_back(light);
            continue;
        }

        try{
            mLights.emplace_back(LightFactory::makeLight(contour,mLightFilter));
        }catch([[maybe_unused]] const std::runtime_error& e) {
        }
    }


    if (!mLights.empty()) return true;
    return false;
}

const std::vector<Light>& LightManager::getLights() const{
    return mLights;
}

void LightManager::drawLights(cv::Mat& outSrc) const {
    if (!mEnableDraw) return;
    for (const Light& light : mLights) {
        light.drawLight(outSrc, mFilterMake, mDebugOptions);
    }
}










cv::Mat LightManager::getLightMask(const cv::Mat& src, const Torosamy::ColorType& colorType) const {
    if (mSplitedMask) return getLightSplit(src, colorType);
    return getLightCompare(src, colorType);
}

cv::Mat LightManager::getLightSplit(const cv::Mat& src, const Torosamy::ColorType& colorType) const{
    using ColorType = Torosamy::ColorType;
    cv::Mat cvtGray;
    cvtColor(src, cvtGray, cv::COLOR_BGR2GRAY);
    //0.0012~0.0025
    cv::Mat subGray;
    std::vector<cv::Mat> splited;
    split(src, splited);

    //0.0008~0.001
    if(colorType == ColorType::RED) {
        threshold(cvtGray, cvtGray, 70, 255, cv::THRESH_BINARY);
        subtract(splited[2], splited[0], subGray);
        threshold(subGray, subGray, 112, 255, cv::THRESH_BINARY);
    }
    else{
        threshold(cvtGray, cvtGray, 170, 255, cv::THRESH_BINARY);

        subtract(splited[0], splited[2], subGray);

        threshold(subGray, subGray, 145, 255, cv::THRESH_BINARY);//110
    }
    //0.0002~0.0003
    dilate(subGray, subGray, getStructuringElement(cv::MORPH_ELLIPSE, cv::Size(5, 5)));
    cv::Mat result =  fillHole(subGray & cvtGray);
    return result;
}


cv::Mat LightManager::getLightCompare(const cv::Mat& src, const Torosamy::ColorType& colorType) const{
    using ColorType = Torosamy::ColorType;
    cv::Mat cvtGray;
    cv::cvtColor(src, cvtGray, cv::COLOR_BGR2GRAY);

    cv::Mat subGray, channel1, channel2;
    if (colorType == ColorType::RED) {
        cv::extractChannel(src, channel1, 2);  // 红通道
        cv::extractChannel(src, channel2, 0);  // 蓝通道
    } else {
        cv::extractChannel(src, channel1, 0);  // 蓝通道
        cv::extractChannel(src, channel2, 2);  // 红通道
    }
    cv::subtract(channel1, channel2, subGray); // 通道相减

    cv::Mat gray_mask, sub_mask;
    cv::compare(cvtGray, 100, gray_mask, cv::CMP_GE);
    cv::compare(subGray, (colorType == ColorType::RED) ? 100 : 50, sub_mask, cv::CMP_GE); // 颜色阈值

    // 位运算合并结果
    cv::Mat result = fillHole(gray_mask & sub_mask);
    // 调试显示应禁用
    return result;
}


cv::Mat LightManager::fillHole(const cv::Mat& src) const{
    cv::Mat cutMat;
    cv::Mat broadImg = cv::Mat::zeros(src.rows + 2, src.cols + 2, CV_8UC1);
    src.copyTo(broadImg(cv::Range(1, src.rows + 1), cv::Range(1, src.cols + 1)));
    floodFill(broadImg, cv::Point(0, 0), cv::Scalar(255));
    broadImg(cv::Range(1, src.rows + 1), cv::Range(1, src.cols + 1)).copyTo(cutMat);
    return src | (~cutMat);
}
