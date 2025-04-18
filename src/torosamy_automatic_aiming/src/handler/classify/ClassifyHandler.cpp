//
// Created by torosamy on 25-3-29.
//
#include "torosamy_automatic_aiming/handler/classify/ClassifyHandler.h"
#include "torosamy_automatic_aiming/handler/classify/OnnxFilter.h"
#include "torosamy_automatic_aiming/handler/classify/DigitFilter.h"

ClassifyHandler::ClassifyHandler(const YAML::Node& fileReader) :
	mClassifyMode(static_cast<ClassifyMode>(fileReader["mode"].as<int>())),
    mEnable(fileReader["enable"].as<bool>()),
    mOnnxMinConfidence(fileReader["minConfidence"]["onxx"].as<float>()),
    mDigitMinConfidence(fileReader["minConfidence"]["digit"].as<float>()) {
}

std::pair<ArmorType, float> ClassifyHandler::classify(const cv::Mat& src, const Armor& armor) const {
    switch (mClassifyMode) {
        case ClassifyMode::ONNX:
            return OnnxFilter(armor, src).classify();
        case ClassifyMode::DIGIT:
            return DigitFilter(armor, src).classify(mDigitMinConfidence);
        default:
            throw std::runtime_error("unknown classify mode");
    }
}


bool ClassifyHandler::enabled() const {
    return mEnable;
}

float ClassifyHandler::getMinConfidence() const {
    switch (mClassifyMode) {
        case ClassifyMode::ONNX:
            return mOnnxMinConfidence;
        case ClassifyMode::DIGIT:
            return mDigitMinConfidence;
        default:
            throw std::runtime_error("unknown classify mode");
    }
}