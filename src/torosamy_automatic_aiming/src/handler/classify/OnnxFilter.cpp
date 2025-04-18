//
// Created by torosamy on 25-3-19.
//
#include <torosamy_automatic_aiming/handler/classify/OnnxFilter.h>
#include <torosamy_automatic_aiming/light/LightFactory.h>
#include "module/TorosamyModule.h"
#include <yaml-cpp/node/node.h>
cv::dnn::Net OnnxFilter::mNet = cv::dnn::readNetFromONNX(Torosamy::TorosamyModule::getConfigLocation("armor")+"onnx/NeuralNetworkModle.onnx");

OnnxFilter::OnnxFilter(const Armor& armor,const cv::Mat& src):
    mArmor(armor),
    mSrc(src){
        
}


std::pair<ArmorType, float> OnnxFilter::classify() {
    cv::Mat blob;
    // 输入图像进行处理
    const cv::Mat mask = generateNumberMask();
    cv::dnn::blobFromImage(mask / 255, blob, 1.0, cv::Size(20, 28));
    // 种类预测
    mNet.setInput(blob);
	cv::Mat outputs = mNet.forward();
    // 最可能分类输出
    float maxProb = *std::max_element(outputs.begin<float>(), outputs.end<float>());
    cv::Mat softMaxProb;
    exp(outputs - maxProb, softMaxProb);
    softMaxProb /= static_cast<float>(sum(softMaxProb)[0]);

    // 计算最大可能性
    double confidence;
    cv::Point classIdPoint;
    cv::minMaxLoc(softMaxProb.reshape(1, 1), nullptr, &confidence, nullptr, &classIdPoint);

    return {static_cast<ArmorType>(classIdPoint.x), confidence * 100};
}

cv::Mat OnnxFilter::generateNumberMask() const{
    cv::Mat result;
    std::vector<cv::Point2f> newArmorPoints = getExtendPoints();
    // 设定偏移量，确保稳定提取图片
    std::vector<float> offsetArrX = getAffineOffestPointsX(newArmorPoints);
    std::vector<float> offsetArrY = getAffineOffestPointsY(newArmorPoints);


    cv::Point2f affinePoints1[4];
    for (int i = 0; i < 4; i++) {
        affinePoints1[i] = cv::Point2f(
              newArmorPoints[i].x + offsetArrX[i],
              newArmorPoints[i].y + offsetArrY[i]
        );
    }

    cv::Point2f affinePoints2[4] = {
        cv::Point2f(0, 0),
        cv::Point2f(48, 0),
        cv::Point2f(48, 48),
        cv::Point2f(0, 48)
    };
    warpPerspective(mSrc, result, getPerspectiveTransform(affinePoints1, affinePoints2), cv::Size(48, 48), cv::INTER_CUBIC);
    cvtColor(result, result, cv::COLOR_RGB2GRAY);
    threshold(result, result, 20, 255, cv::THRESH_BINARY | cv::THRESH_OTSU);
    // 去除椒盐噪声
    medianBlur(result, result, 5);


    // 制作掩码，遮去左右两侧噪点提高识别率
    cv::Mat mask(48, 48, CV_8UC1, cv::Scalar(0, 0, 0));
    rectangle(mask, cv::Rect(0, 0, 4, 48), cv::Scalar(255, 255, 255), -1);
    rectangle(mask, cv::Rect(45, 0, 48, 30), cv::Scalar(255, 255, 255), -1);
    rectangle(mask, cv::Rect(45, 42, 48, 48), cv::Scalar(255, 255, 255), -1);

    return result - mask;
}

std::vector<cv::Point2f> OnnxFilter::getExtendPoints() const {
    //if(!mAccepted) throw std::invalid_argument("Onnx filter does not have accepted parameters");

    std::vector<cv::Point2f> result;
    Light newLeftLight = LightFactory::extendLight(mArmor.getLeftLight());
    Light newRightlight = LightFactory::extendLight(mArmor.getRightLight());

    std::vector<cv::Point2f> leftLightPoints = getSortPoints(newLeftLight.getPoints());
    std::vector<cv::Point2f> rightLightPoints = getSortPoints(newRightlight.getPoints());

    result.push_back(leftLightPoints.at(1));
    result.push_back(rightLightPoints.at(0));
    result.push_back(rightLightPoints.at(3));
    result.push_back(leftLightPoints.at(2));
    return result;
}


std::vector<cv::Point2f> OnnxFilter::getSortPoints(const std::vector<cv::Point2f>& points) const {
    if (points.empty()) return {};
    std::vector<cv::Point2f> result = points;

    for (int i = 0; i < 4; i++) {
        for (int j = i + 1; j < 4; j++) {
            if (result.at(i).y > result.at(j).y) {
                swap(result.at(i), result.at(j));
            }
        }
    }
    if (result.at(0).x > result.at(1).x) {
        swap(result.at(0), result.at(1));
    }
    if (result.at(2).x < result.at(3).x) {
        swap(result.at(2), result.at(3));
    }
    return result;
}



std::pair<float,float> OnnxFilter::getAffineOffestPointsParam()  const{
    //if(!mAccepted) throw std::invalid_argument("Onnx filter does not have accepted parameters");

    float param0 = mArmor.isBigArmor() ? 0.35 : 0.117;
    float param1 = mArmor.isBigArmor() ? 0.95 * 0.31 : 0.85 * 0.117;
    return {param0, param1};
}


std::vector<float> OnnxFilter::getAffineOffestPointsX(const std::vector<cv::Point2f>& points) const {
    std::pair<float,float> paramPair = getAffineOffestPointsParam();
    float param0 = paramPair.first;
    float param1 = paramPair.second;

    std::vector<float> offsetArrX;
    offsetArrX.push_back(param0 * fabs(points.at(1).x - points.at(0).x));
    offsetArrX.push_back(-(param1 * fabs(points.at(1).x - points.at(0).x)));
    offsetArrX.push_back(-(param1 * fabs(points.at(2).x - points.at(3).x)));
    offsetArrX.push_back(param0 * fabs(points.at(2).x - points.at(3).x));

    return offsetArrX;
}

std::vector<float> OnnxFilter::getAffineOffestPointsY(const std::vector<cv::Point2f>& points) const {
    std::pair<float,float> paramPair = getAffineOffestPointsParam();
    float param0 = paramPair.first;
    float param1 = paramPair.second;

    std::vector<float> offsetArrY;

    offsetArrY.push_back(param0 * fabs(points.at(1).y - points.at(0).y));
    offsetArrY.push_back(-(param1 * fabs(points.at(1).y - points.at(0).y)));
    offsetArrY.push_back(-(param1 * fabs(points.at(2).y - points.at(3).y)));
    offsetArrY.push_back(param0 * fabs(points.at(2).y - points.at(3).y));

    return offsetArrY;
}


